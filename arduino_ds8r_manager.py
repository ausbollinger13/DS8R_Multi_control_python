import serial
import serial.tools.list_ports
import time
import os
import ctypes
import threading
from PySide6.QtWidgets import QDialog, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDoubleSpinBox, QPushButton, QComboBox, QGroupBox, QTabWidget
from PySide6.QtCore import Qt
from ctypes import *

# --- Digitimer DLL Path  ---
current_dir = os.path.dirname(os.path.abspath(__file__))
dll_dir = os.path.abspath(os.path.join(current_dir, "..", "src/ds8r"))
DLL_PATH = os.path.join(dll_dir, "D128RProxy.dll")
D128API_DLL_PATH = "C:\\Windows\\System32\\D128API.dll"

# Constants from official Digitimer code
D128_MAXIMUMDEVICES = 5
D128_ENABLE = 0x01
D128_DISABLE = 0x02
D128_MODE_MONO = 0x01
D128_MODE_BIPH = 0x02
D128_POLARITY_POS = 0x01
D128_POLARITY_NEG = 0x02
D128_POLARITY_ALT = 0x03
D128_SOURCE_INT = 0x01
D128_SOURCE_EXT = 0x02
D128_AUTO_ZERO = 0x01
D128_TRIGGER = 0x01
D128_BUZZER_EN = 0x00
D128_BUZZER_DIS = 0x01

class D128Exception(Exception):
    """Custom exception for D128 API errors"""
    pass

class Arduino_DS8R_Manager:
    """
    Unified DS8R Manager that handles all DS8R devices through a single Arduino connection
    and shared D128API instance. This approach reduces resource conflicts and improves
    connection stability for long stimulation runs.
    """
    
    def __init__(self):
        # Single Arduino connection and D128API instance
        self.ser = None
        self.port = None
        self.d128api = None
        self.is_initialized = False
        
        # Device mappings - up to 3 devices supported
        self.devices = {
            0: {
                "target_id": 348,
                "device_index": None,
                "location": "Cymba",        # Keep original default
                "last_frequency": 25.0,     # Keep original default 
                "last_duration": 0.5,       # Keep original default
                "arduino_id": 0  # T0, S0, STOP0 commands
            },
            1: {
                "target_id": 362,
                "device_index": None,
                "location": "Earlobe", 
                "last_frequency": 25.0,
                "last_duration": 0.5,
                "arduino_id": 1  # T1, S1, STOP1 commands
            },
            2: {
                "target_id": 387,  
                "device_index": None,
                "location": "Arm",
                "last_frequency": 25.0,
                "last_duration": 0.5,
                "arduino_id": 2  # T2, S2, STOP2 commands
            }
        }
        
        # Connection health tracking
        self._last_dll_check_time = 0
        self._last_dll_status = False

        # Concurrency and pacing controls
        self._serial_lock = threading.Lock()
        self._dll_lock = threading.Lock()
        self._last_serial_send = 0.0
        self._last_dll_call = 0.0
        # Increase to 150ms minimum between ALL DLL calls to respect 10Hz limit
        self._dll_min_interval = 0.15  # Changed from 0.12 to 0.15
        self._serial_min_interval = 0.15  # Also increase serial interval for safety

        # Background keepalive thread for DLL
        self._keepalive_stop = threading.Event()
        self._keepalive_thread = None

        # Failure/circuit breaker tracking
        self._failure_count = 0
        self._last_failure_time = 0.0
        self._circuit_open_until = 0.0
        
    def initialize_connection(self):
        """
        Initialize Arduino connection and D128API. Returns True if successful.
        This replaces the individual device initialization.
        """
        try:
            print("[DS8R_Manager] Initializing Arduino connection...")
            
            # Find and connect to Arduino
            self.port = self._find_arduino_port()
            if not self.port:
                print("[DS8R_Manager] Arduino not found")
                return False
                
            # Establish serial connection
            if self.ser is None or not self.ser.is_open:
                self.ser = serial.Serial(self.port, baudrate=9600, timeout=1)
                time.sleep(0.5)  # Brief settling time
                print(f"[DS8R_Manager] Arduino connected on {self.port}")
            
            # Initialize D128API
            print("[DS8R_Manager] Initializing D128API...")
            success = self._initialize_ds8r_api()
            if not success:
                print("[DS8R_Manager] D128API initialization failed - continuing with Arduino-only mode")
            
            # Map device IDs to indices
            self._map_device_indices()
            
            # Start DLL keepalive thread ONLY if D128API is working
            if self.d128api and success:
                # Wait a bit before starting keepalive to let initialization settle
                time.sleep(1.0)
                if self._keepalive_thread is None or not self._keepalive_thread.is_alive():
                    self._start_keepalive_thread()
            
            self.is_initialized = True
            print("[DS8R_Manager] Initialization complete")
            return True
            
        except Exception as e:
            print(f"[DS8R_Manager] Initialization failed: {e}")
            self._cleanup_partial_init()
            return False

    def _start_keepalive_thread(self):
        def _runner():
            # Wait initial period to let main initialization fully complete
            time.sleep(2.0)
            
            # Periodically keep DLL connection alive to prevent timeouts
            while not self._keepalive_stop.is_set():
                try:
                    if self.d128api:
                        with self._dll_lock:
                            # Respect DLL pacing
                            self._respect_rate_limit('dll')
                            try:
                                # Use a lighter keepalive check - just get device count
                                device_count = self.d128api.DeviceCount()
                                if device_count <= 0:
                                    # Only attempt recovery if device count suggests real problem
                                    print("[D128API] Connection health check failed, attempting recovery...")
                                    self.d128api.recover_connection()
                                    self._map_device_indices()
                            except Exception as e:
                                # Only attempt recovery on actual errors, not timeouts
                                if "timeout" not in str(e).lower():
                                    print(f"[D128API] Connection error: {e}")
                                    try:
                                        self.d128api.recover_connection()
                                        self._map_device_indices()
                                    except Exception:
                                        pass
                except Exception:
                    pass
                finally:
                    # Sleep longer between keepalives to reduce interference
                    self._keepalive_stop.wait(3.0)  # 3 second intervals instead of 1
        
        self._keepalive_stop.clear()
        self._keepalive_thread = threading.Thread(target=_runner, name="DS8R-DLL-Keepalive", daemon=True)
        self._keepalive_thread.start()
        print("[DS8R_Manager] DLL keepalive thread started")

    def _respect_rate_limit(self, kind: str):
        now = time.time()
        if kind == 'serial':
            elapsed = now - self._last_serial_send
            if elapsed < self._serial_min_interval:
                time.sleep(self._serial_min_interval - elapsed)
            self._last_serial_send = time.time()
        elif kind == 'dll':
            elapsed = now - self._last_dll_call
            if elapsed < self._dll_min_interval:
                time.sleep(self._dll_min_interval - elapsed)
            self._last_dll_call = time.time()

    def _write_serial(self, command: str, expect_ack: bool = False, ack_timeout: float = 0.3) -> bool:
        if not (self.ser and self.ser.is_open):
            print("[DS8R_Manager] Serial not available for command:", command.strip())
            return False
        try:
            with self._serial_lock:
                self._respect_rate_limit('serial')
                self.ser.write(command.encode())
                self.ser.flush()
                print(f"[DS8R_Manager] Serial sent: {command.strip()}")
                
                if expect_ack:
                    # Try to read a simple ACK line from Arduino if firmware supports it
                    start = time.time()
                    while time.time() - start < ack_timeout:
                        if self.ser.in_waiting:
                            line = self.ser.readline().decode(errors='ignore').strip()
                            if line:
                                print(f"[DS8R_Manager] Serial recv: {line}")
                                # Treat any non-empty line as ack
                                return True
                        time.sleep(0.01)
                    # No ACK received - not fatal
                    return True
                else:
                    return True
        except Exception as e:
            print(f"[DS8R_Manager] Serial write failed: {e}")
            self._note_failure()
            return False

    def _note_failure(self):
        now = time.time()
        # Reset window if last failure long ago
        if now - self._last_failure_time > 5.0:
            self._failure_count = 0
        self._failure_count += 1
        self._last_failure_time = now
        if self._failure_count >= 3:
            # Open circuit for a short cooldown to avoid watchdog conditions
            self._circuit_open_until = now + 2.0
            print("[DS8R_Manager] Too many recent failures - pausing commands for 2s to recover")

    def _circuit_allows_commands(self) -> bool:
        if time.time() < self._circuit_open_until:
            print("[DS8R_Manager] Circuit breaker active - skipping command")
            return False
        return True

    def _find_arduino_port(self):
        """Find Arduino port using the same logic as the original classes"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if ("Arduino" in port.description or
                "CH340" in port.description or
                "USB Serial Device" in port.description or
                ("VID:PID=2341:0043" in port.hwid) or
                ("VID:PID=1A86:7523" in port.hwid)):
                return port.device
        return None
    
    def _initialize_ds8r_api(self):
        """Initialize D128API with error handling"""
        try:
            # Try to temporarily modify the module's path before importing
            import sys
            import os
            
            # Add the correct path to the environment
            old_path = os.environ.get('PATH', '')
            system32_path = "C:\\Windows\\System32"
            if system32_path not in old_path:
                os.environ['PATH'] = system32_path + os.pathsep + old_path
            
            try:
                # Import the D128API class from the original arduino_ds8r module
                from arduino_ds8r import D128API
                self.d128api = D128API()
                
                # Try to override DLL path if possible
                if hasattr(self.d128api, 'dll_path'):
                    self.d128api.dll_path = D128API_DLL_PATH
                    print(f"[DS8R_Manager] Updated DLL path to: {D128API_DLL_PATH}")
                
                # Give the DLL time to settle before initializing
                time.sleep(0.2)
                
                ret, call_result = self.d128api.Initialise()
                
                if ret != 0 or call_result != 0:
                    raise D128Exception(f"D128API initialization failed: ret={ret}, call_result={call_result}")
                
                # Additional settling time after successful initialization
                time.sleep(0.3)
                
                print("[DS8R_Manager] D128API initialized successfully")
                return True
                
            finally:
                # Restore original PATH
                os.environ['PATH'] = old_path
            
        except ImportError as e:
            print(f"[DS8R_Manager] Could not import D128API: {e}")
            self.d128api = None
            return False
        except Exception as e:
            print(f"[DS8R_Manager] D128API initialization failed: {e}")
            self.d128api = None
            return False

    def _map_device_indices(self):
        """Map device IDs to their indices in the D128API"""
        if not self.d128api:
            return
            
        try:
            # Give DLL time to settle before mapping
            time.sleep(0.1)
            
            # Get fresh state with retry logic
            max_retries = 3
            for attempt in range(max_retries):
                try:
                    if self.d128api.GetState():
                        break
                    else:
                        print(f"[DS8R_Manager] GetState failed on attempt {attempt + 1}")
                        if attempt < max_retries - 1:
                            time.sleep(0.5)
                        else:
                            print("[DS8R_Manager] Could not get DS8R state for device mapping")
                            return
                except Exception as e:
                    print(f"[DS8R_Manager] GetState exception on attempt {attempt + 1}: {e}")
                    if attempt < max_retries - 1:
                        time.sleep(0.5)
                    else:
                        print("[DS8R_Manager] Failed to get DS8R state after retries")
                        return
                
            device_count = self.d128api.DeviceCount()
            print(f"[DS8R_Manager] Found {device_count} DS8R devices")
            
            # Map devices 0 and 1 to their specific IDs
            for device_id in [0, 1]:
                target_id = self.devices[device_id]["target_id"]
                device_index = self._find_device_by_id(target_id)
                if device_index is not None:
                    self.devices[device_id]["device_index"] = device_index
                    print(f"[DS8R_Manager] Device {device_id} mapped to DS8R ID {target_id} at index {device_index}")
                else:
                    print(f"[DS8R_Manager] Device {device_id} (ID {target_id}) not found")
            
            # For device 2, find any available device not used by 0 or 1
            used_indices = [self.devices[0]["device_index"], self.devices[1]["device_index"]]
            used_indices = [idx for idx in used_indices if idx is not None]
            
            for i in range(device_count):
                if i not in used_indices:
                    try:
                        device = self.d128api.Device(i)
                        if device:
                            self.devices[2]["device_index"] = i
                            self.devices[2]["target_id"] = device.DEVICEID
                            print(f"[DS8R_Manager] Device 2 mapped to available DS8R ID {device.DEVICEID} at index {i}")
                            break
                    except Exception as e:
                        print(f"[DS8R_Manager] Error checking device {i}: {e}")
                        continue
                        
        except Exception as e:
            print(f"[DS8R_Manager] Error during device mapping: {e}")

    def _find_device_by_id(self, target_id):
        """Find device index by ID"""
        if not self.d128api:
            return None
            
        try:
            device_count = self.d128api.DeviceCount()
            for i in range(device_count):
                try:
                    device = self.d128api.Device(i)
                    if device and device.DEVICEID == target_id:
                        return i
                except Exception:
                    continue
            return None
        except Exception:
            return None
    
    def send_stimulation(self, device_id, frequency=None, duration=None):
        """Send stimulation command to specific device"""
        if not self._validate_device_id(device_id):
            return False
            
        if not self.ser or not self.ser.is_open:
            print(f"[DS8R_Manager] No Arduino connection for device {device_id}")
            return False
            
        if not self._circuit_allows_commands():
            return False
            
        try:
            arduino_id = self.devices[device_id]["arduino_id"]
            if frequency is not None and duration is not None:
                self.devices[device_id]["last_frequency"] = frequency
                self.devices[device_id]["last_duration"] = duration
                command = f"T{arduino_id},{frequency},{duration}\n"
            else:
                command = f"T{arduino_id}\n"
            
            ok = self._write_serial(command, expect_ack=False)
            # Small additional guard delay to avoid back-to-back serial/API collisions from UI
            time.sleep(0.02)
            if ok:
                # Reset failure streak on success
                self._failure_count = 0
                return True
            return False
            
        except Exception as e:
            print(f"[DS8R_Manager] Error sending stimulation to device {device_id}: {e}")
            self._note_failure()
            return False
    
    def set_stimulation_params(self, device_id, frequency, duration):
        """Set stimulation parameters without triggering"""
        if not self._validate_device_id(device_id):
            return False
            
        if not self.ser or not self.ser.is_open:
            print(f"[DS8R_Manager] No Arduino connection for device {device_id}")
            return False
            
        if not self._circuit_allows_commands():
            return False
            
        try:
            arduino_id = self.devices[device_id]["arduino_id"]
            self.devices[device_id]["last_frequency"] = frequency
            self.devices[device_id]["last_duration"] = duration
            
            command = f"S{arduino_id},{frequency},{duration}\n"
            ok = self._write_serial(command, expect_ack=False)
            if ok:
                self._failure_count = 0
                return True
            return False
            
        except Exception as e:
            print(f"[DS8R_Manager] Error setting params for device {device_id}: {e}")
            self._note_failure()
            return False
    
    def stop_stimulation(self, device_id):
        """Stop stimulation for specific device"""
        if not self._validate_device_id(device_id):
            return False
            
        if not self.ser or not self.ser.is_open:
            print(f"[DS8R_Manager] No Arduino connection for device {device_id}")
            return False
            
        if not self._circuit_allows_commands():
            return False
            
        try:
            arduino_id = self.devices[device_id]["arduino_id"]
            command = f"STOP{arduino_id}\n"
            ok = self._write_serial(command, expect_ack=False)
            # Give hardware a short settling time after STOP
            time.sleep(0.05)
            if ok:
                self._failure_count = 0
                return True
            return False
            
        except Exception as e:
            print(f"[DS8R_Manager] Error stopping device {device_id}: {e}")
            self._note_failure()
            return False
    
    def set_amplitude_and_pulsewidth(self, device_id, amplitude_ma, pulsewidth_us):
        """Set amplitude and pulse width for specific device via D128API with minimal interference"""
        if not self._validate_device_id(device_id):
            return False
            
        device_index = self.devices[device_id]["device_index"]
        
        if self.d128api and device_index is not None:
            try:
                with self._dll_lock:
                    # ALWAYS respect rate limit - this was missing before
                    self._respect_rate_limit('dll')
                    
                    # Round amplitude to avoid floating point precision issues
                    amplitude_rounded = round(amplitude_ma, 1)  # Round to 1 decimal place
                    pulsewidth_rounded = round(pulsewidth_us, 0)  # Round to nearest integer
                    
                    print(f"[D128API] Setting device {device_id}: amplitude={amplitude_rounded}mA, pulsewidth={pulsewidth_rounded}μs")
                    self.d128api.set_amplitude_and_width(device_index, amplitude_rounded, int(pulsewidth_rounded))
                    print(f"[DS8R_Manager] Device {device_id} amplitude/pulsewidth set: {amplitude_rounded}mA, {int(pulsewidth_rounded)}μs")
                    self._failure_count = 0
                    return True
                    
            except Exception as e:
                print(f"[DS8R_Manager] Error setting amplitude/pulsewidth for device {device_id}: {e}")
                self._note_failure()
                return False
        else:
            print(f"[DS8R_Manager] Device {device_id} not available for amplitude/pulsewidth setting")
            return False
    
    def send_single_pulse(self, device_id, amplitude_ma, pulsewidth_us):
        """Send a single pulse for H-reflex testing with optimized parameters"""
        if not self._validate_device_id(device_id):
            return False
        
        try:
            # Round values to avoid floating point precision issues
            amplitude_rounded = round(amplitude_ma, 1)
            pulsewidth_rounded = round(pulsewidth_us, 0)
            
            # Set amplitude and pulse width first
            if not self.set_amplitude_and_pulsewidth(device_id, amplitude_rounded, int(pulsewidth_rounded)):
                print(f"[DS8R_Manager] Failed to set parameters for single pulse on device {device_id}")
                return False
            
            # Configure for single pulse stimulation
            single_pulse_frequency = 1.0  # 1 Hz
            single_pulse_duration = 0.1  # 100ms for single pulse
            
            # Set single pulse parameters
            if not self.set_stimulation_params(device_id, single_pulse_frequency, single_pulse_duration):
                print(f"[DS8R_Manager] Failed to set single pulse timing for device {device_id}")
                return False
            
            # Send the single pulse
            success = self.send_stimulation(device_id, single_pulse_frequency, single_pulse_duration)
            if success:
                print(f"[DS8R_Manager] Single pulse sent to device {device_id}: {amplitude_rounded}mA, {int(pulsewidth_rounded)}μs")
                return True
            else:
                print(f"[DS8R_Manager] Failed to send single pulse to device {device_id}")
                return False
                
        except Exception as e:
            print(f"[DS8R_Manager] Error sending single pulse to device {device_id}: {e}")
            return False
    
    def get_device_status(self, device_id):
        """Get connection status for specific device"""
        if not self._validate_device_id(device_id):
            return False
            
        # Check Arduino connection
        arduino_ok = self.ser is not None and self.ser.is_open
        
        # Check DS8R device availability with caching to reduce flicker
        if self.d128api:
            try:
                current_time = time.time()
                
                # Only check DLL every 5 seconds to reduce interference with keepalive
                if current_time - self._last_dll_check_time > 5.0:
                    try:
                        with self._dll_lock:
                            self._respect_rate_limit('dll')
                            count = self.d128api.DeviceCount()
                            device_index = self.devices[device_id]["device_index"]
                            ds8r_ok = (device_index is not None and 
                                     count > device_index and 
                                     device_index >= 0)
                            self._last_dll_status = ds8r_ok
                            self._last_dll_check_time = current_time
                    except Exception:
                        # Keep the last known status if DLL call fails
                        self._last_dll_check_time = current_time
                
                return arduino_ok and self._last_dll_status
                
            except Exception:
                return arduino_ok
        else:
            # If no DS8R API, just check Arduino connection
            return arduino_ok
    
    def test_connection(self, device_id):
        """Test connection for specific device"""
        if not self._validate_device_id(device_id):
            return False
            
        if not self.ser or not self.ser.is_open:
            return False
            
        if not self._circuit_allows_commands():
            return False
            
        try:
            arduino_id = self.devices[device_id]["arduino_id"]
            command = f"T{arduino_id}\n"
            ok = self._write_serial(command, expect_ack=False)
            if ok:
                self._failure_count = 0
                print(f"[DS8R_Manager] Device {device_id} connection test sent")
                return True
            return False
        except Exception as e:
            print(f"[DS8R_Manager] Device {device_id} connection test failed: {e}")
            self._note_failure()
            return False
    
    def get_device_info(self, device_id):
        """Get device information"""
        if not self._validate_device_id(device_id):
            return None
            
        device = self.devices[device_id].copy()
        device["is_connected"] = self.get_device_status(device_id)
        device["arduino_connected"] = self.ser is not None and self.ser.is_open
        device["ds8r_available"] = self.d128api is not None and device["device_index"] is not None
        return device
    
    def _validate_device_id(self, device_id):
        """Validate device ID is in valid range"""
        if device_id not in self.devices:
            print(f"[DS8R_Manager] Invalid device ID: {device_id}")
            return False
        return True
    
    def _cleanup_partial_init(self):
        """Clean up partial initialization"""
        try:
            if self.d128api:
                try:
                    self.d128api.Close()
                except Exception:
                    pass
                self.d128api = None
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
        except Exception as e:
            print(f"[DS8R_Manager] Error during cleanup: {e}")
    
    def close(self):
        """Clean up all resources"""
        try:
            print("[DS8R_Manager] Closing connections...")
            
            # Stop keepalive first
            try:
                if self._keepalive_thread and self._keepalive_thread.is_alive():
                    self._keepalive_stop.set()
                    # Give it more time to stop gracefully
                    self._keepalive_thread.join(timeout=2.0)
                    if self._keepalive_thread.is_alive():
                        print("[DS8R_Manager] Keepalive thread did not stop gracefully")
            except Exception as e:
                print(f"[DS8R_Manager] Error stopping keepalive: {e}")
            
            # Stop all stimulations first
            for device_id in self.devices:
                try:
                    if self.get_device_status(device_id):
                        self.stop_stimulation(device_id)
                except Exception:
                    pass
            
            # Close D128API
            if self.d128api:
                try:
                    with self._dll_lock:
                        self.d128api.Close()
                except Exception as e:
                    print(f"[DS8R_Manager] Error closing D128API: {e}")
                self.d128api = None
                
            # Close Arduino connection
            if self.ser and self.ser.is_open:
                try:
                    self.ser.close()
                except Exception as e:
                    print(f"[DS8R_Manager] Error closing serial: {e}")
                self.ser = None
                
            self.is_initialized = False
            print("[DS8R_Manager] All connections closed")
            
        except Exception as e:
            print(f"[DS8R_Manager] Error during close: {e}")
    
    def __del__(self):
        """Destructor to ensure cleanup"""
        try:
            self.close()
        except:
            pass

class DS8RSetupPanel_Manager(QWidget):
    """
    Unified setup panel for all DS8R devices managed by Arduino_DS8R_Manager
    """
    
    def __init__(self, return_callback=None, parent=None):
        super().__init__(parent)
        self.return_callback = return_callback
        self.ds8r_manager = None
        # Resolve local icon paths for spinbox arrows (use forward slashes for Qt)
        # Resolve icons: prefer local assets next to this script, but fall back
        # to a packaged `scripts/assets/icons` path if the former is missing.
        icons_dir = os.path.join(os.path.dirname(__file__), "assets", "icons")
        up_icon = os.path.join(icons_dir, "chevron_up.svg")
        down_icon = os.path.join(icons_dir, "chevron_down.svg")

        # If icons don't exist at this path (e.g. running from repo root),
        # try to find them under the `scripts/assets/icons` folder as well.
        if not os.path.exists(up_icon) or not os.path.exists(down_icon):
            alt_icons_dir = os.path.join(os.path.dirname(__file__), "assets", "icons")
            up_icon = os.path.join(alt_icons_dir, "chevron_up.svg")
            down_icon = os.path.join(alt_icons_dir, "chevron_down.svg")

        # Normalize for Qt stylesheet usage (forward slashes)
        up_icon = up_icon.replace('\\', '/')
        down_icon = down_icon.replace('\\', '/')

        css = """
            QWidget {
                background-color: #f0f8ff;
                border: 2px solid #90caf9;
                border-radius: 18px;
            }
            QLabel {
                font-size: 20px;
                font-weight: bold;
                color: #1565c0;
                min-height: 30px;
            }
            QDoubleSpinBox {
                font-size: 28px;
                min-height: 48px;
                min-width: 180px;
                background: #e3f2fd;
                border-radius: 8px;
                padding: 2px 8px;
                padding-right: 90px;
            }
            QAbstractSpinBox::up-button {
                subcontrol-origin: border;
                subcontrol-position: center right;
                width: 40px;
                height: 40px;
                border-left: 1px solid #90caf9;
                right: 45px;
                background: transparent;
            }
            QAbstractSpinBox::down-button {
                subcontrol-origin: border;
                subcontrol-position: center right;
                width: 40px;
                height: 40px;
                border-left: 1px solid #90caf9;
                right: 5px;
                background: transparent;
            }
            /* Custom chevron icons for spinbox arrows */
            QAbstractSpinBox::up-arrow, QAbstractSpinBox::down-arrow {
                width: 22px;
                height: 22px;
            }
            QAbstractSpinBox::up-arrow { image: url('__UP__'); }
            QAbstractSpinBox::down-arrow { image: url('__DOWN__'); }
            QAbstractSpinBox::up-button:hover,
            QAbstractSpinBox::down-button:hover {
                background: rgba(33, 150, 243, 0.10);
            }
            /* Use same icons for disabled to ensure visibility on Windows; add grey variants later if desired */
            QAbstractSpinBox::up-arrow:disabled { image: url('__UP__'); }
            QAbstractSpinBox::down-arrow:disabled { image: url('__DOWN__'); }
            QComboBox {
                font-size: 20px;
                background: #e3f2fd;
                border-radius: 8px;
                padding: 8px 12px;
                min-height: 40px;
                min-width: 120px;
            }
            QComboBox::drop-down {
                width: 30px;
                border-radius: 4px;
                background: #2196f3;
            }
            QComboBox::drop-down:hover {
                background: #1976d2;
            }
            QPushButton {
                font-size: 18px;
                border-radius: 12px;
                padding: 8px 24px;
                font-weight: bold;
            }
            QPushButton#Connect {
                background-color: #43a047;
                color: white;
            }
            QPushButton#Apply {
                background-color: #2196f3;
                color: white;
            }
            QPushButton#Stim {
                background-color: #ff5722;
                color: white;
            }
            QPushButton#Return {
                background-color: #e53935;
                color: white;
            }
            QPushButton:hover {
                opacity: 0.85;
            }
        """
        css = css.replace('__UP__', up_icon).replace('__DOWN__', down_icon)
        self.setStyleSheet(css)

        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout()
        layout.setSpacing(18)
        
        # Header
        header = QLabel("DS8R Device Manager Setup")
        header.setAlignment(Qt.AlignCenter)
        header.setStyleSheet("font-size: 28px; font-weight: bold; color: #1976d2; margin-bottom: 16px;")
        layout.addWidget(header)
        
        # Connection section
        conn_layout = QHBoxLayout()
        self.connect_btn = QPushButton("Initialize Connection")
        self.connect_btn.setObjectName("Connect")
        self.connect_btn.clicked.connect(self.initialize_connection)
        conn_layout.addWidget(self.connect_btn)
        
        self.connection_status = QLabel("Not Connected")
        self.connection_status.setStyleSheet("font-size: 16px; color: #d32f2f;")
        conn_layout.addWidget(self.connection_status)
        layout.addLayout(conn_layout)
        
        # Create tab widget for each device
        self.tab_widget = QTabWidget()
        
        # Create tabs for each device
        self.device_tabs = {}
        for device_id in [0, 1, 2]:
            tab = self.create_device_tab(device_id)
            self.device_tabs[device_id] = tab
            device_name = f"Device {device_id}"
            self.tab_widget.addTab(tab, device_name)
        
        layout.addWidget(self.tab_widget)
        
        # Return button
        return_btn = QPushButton("Return to Main")
        return_btn.setObjectName("Return")
        return_btn.clicked.connect(self.handle_return_to_main)
        layout.addWidget(return_btn, alignment=Qt.AlignCenter)
        
        self.setLayout(layout)
    
    def create_device_tab(self, device_id):
        """Create setup tab for individual device"""
        tab = QWidget()
        layout = QVBoxLayout()
        
        # Device info
        info_label = QLabel(f"DS8R Device {device_id}")
        info_label.setStyleSheet("font-size: 22px; font-weight: bold; color: #1976d2;")
        layout.addWidget(info_label)
        
        # Status
        status_label = QLabel("Status: Not Connected")
        status_label.setObjectName(f"status_{device_id}")
        layout.addWidget(status_label)
        
        # Frequency
        freq_layout = QHBoxLayout()
        freq_label = QLabel("Frequency (Hz):")
        freq_spin = QDoubleSpinBox()
        freq_spin.setRange(1, 1000)
        freq_spin.setSingleStep(5)
        freq_spin.setValue(25.0)
        freq_spin.setMinimumSize(200, 60)
        freq_spin.setObjectName(f"freq_{device_id}")
        freq_layout.addWidget(freq_label)
        freq_layout.addWidget(freq_spin)
        layout.addLayout(freq_layout)
        
        # Duration
        dur_layout = QHBoxLayout()
        dur_label = QLabel("Duration (s):")
        dur_spin = QDoubleSpinBox()
        dur_spin.setRange(0.01, 10.0)
        dur_spin.setDecimals(2)
        dur_spin.setSingleStep(0.25)
        dur_spin.setValue(0.5)
        dur_spin.setMinimumSize(200, 60)
        dur_spin.setObjectName(f"dur_{device_id}")
        dur_layout.addWidget(dur_label)
        dur_layout.addWidget(dur_spin)
        layout.addLayout(dur_layout)
        
        # Pulse Width
        pw_layout = QHBoxLayout()
        pw_label = QLabel("Pulse Width (μs):")
        pw_spin = QDoubleSpinBox()
        pw_spin.setRange(50, 2000)
        pw_spin.setSingleStep(50)
        pw_spin.setValue(100)
        pw_spin.setMinimumSize(200, 60)
        pw_spin.setObjectName(f"pw_{device_id}")
        pw_layout.addWidget(pw_label)
        pw_layout.addWidget(pw_spin)
        layout.addLayout(pw_layout)
        
        # Amplitude
        amp_layout = QHBoxLayout()
        amp_label = QLabel("Amplitude (mA):")
        amp_spin = QDoubleSpinBox()
        amp_spin.setRange(0.1, 10.0)
        amp_spin.setDecimals(2)
        amp_spin.setSingleStep(0.1)
        amp_spin.setValue(0.8)
        amp_spin.setMinimumSize(200, 60)
        amp_spin.setObjectName(f"amp_{device_id}")
        amp_layout.addWidget(amp_label)
        amp_layout.addWidget(amp_spin)
        layout.addLayout(amp_layout)
        
        # Location
        location_layout = QHBoxLayout()
        location_label = QLabel("Location:")
        location_combo = QComboBox()
        location_combo.addItems(["Cymba", "Earlobe", "Arm", "Finger", "Leg", "Toe"])
        default_locations = ["Cymba", "Earlobe", "Arm"]
        location_combo.setCurrentText(default_locations[device_id])
        location_combo.setMinimumSize(200, 60)
        location_combo.setObjectName(f"location_{device_id}")
        location_layout.addWidget(location_label)
        location_layout.addWidget(location_combo)
        layout.addLayout(location_layout)
        
        # Buttons
        btn_layout = QHBoxLayout()
        
        apply_btn = QPushButton("Apply Settings")
        apply_btn.setObjectName("Apply")
        apply_btn.clicked.connect(lambda: self.apply_settings(device_id))
        btn_layout.addWidget(apply_btn)
        
        stim_btn = QPushButton("Stimulate")
        stim_btn.setObjectName("Stim")
        stim_btn.clicked.connect(lambda: self.start_stimulation(device_id))
        btn_layout.addWidget(stim_btn)
        
        layout.addLayout(btn_layout)
        
        tab.setLayout(layout)
        return tab
    
    def initialize_connection(self):
        """Initialize the DS8R manager connection"""
        if self.ds8r_manager is None:
            self.ds8r_manager = Arduino_DS8R_Manager()
        
        success = self.ds8r_manager.initialize_connection()
        
        if success:
            self.connection_status.setText("Connected")
            self.connection_status.setStyleSheet("font-size: 16px; color: #2e7d32;")
            self.connect_btn.setText("Reconnect")
            self.update_device_statuses()
        else:
            self.connection_status.setText("Connection Failed")
            self.connection_status.setStyleSheet("font-size: 16px; color: #d32f2f;")
    
    def update_device_statuses(self):
        """Update status displays for all devices"""
        if not self.ds8r_manager:
            return
            
        for device_id in [0, 1, 2]:
            status_label = self.findChild(QLabel, f"status_{device_id}")
            if status_label:
                is_connected = self.ds8r_manager.get_device_status(device_id)
                device_info = self.ds8r_manager.get_device_info(device_id)
                
                if is_connected:
                    status_text = f"Connected - {device_info['location']}"
                    if device_info['target_id']:
                        status_text += f" (ID: {device_info['target_id']})"
                    status_label.setText(f"Status: {status_text}")
                    status_label.setStyleSheet("color: #2e7d32;")
                else:
                    status_label.setText("Status: Not Available")
                    status_label.setStyleSheet("color: #d32f2f;")
    
    def apply_settings(self, device_id):
        """Apply settings for specific device"""
        if not self.ds8r_manager:
            print(f"DS8R Manager not initialized")
            return
            
        try:
            # Get values from UI
            amp_spin = self.findChild(QDoubleSpinBox, f"amp_{device_id}")
            pw_spin = self.findChild(QDoubleSpinBox, f"pw_{device_id}")
            freq_spin = self.findChild(QDoubleSpinBox, f"freq_{device_id}")
            dur_spin = self.findChild(QDoubleSpinBox, f"dur_{device_id}")
            location_combo = self.findChild(QComboBox, f"location_{device_id}")
            
            # Round values to avoid floating point precision issues
            amplitude = round(amp_spin.value(), 1)  # Round to 1 decimal place
            pulsewidth = int(round(pw_spin.value(), 0))  # Round to nearest integer
            frequency = freq_spin.value()
            duration = dur_spin.value()
            location = location_combo.currentText()
            
            # Update location in manager
            self.ds8r_manager.devices[device_id]["location"] = location
            
            # Apply amplitude and pulsewidth via D128API
            success = self.ds8r_manager.set_amplitude_and_pulsewidth(device_id, amplitude, pulsewidth)
            
            if success:
                print(f"Device {device_id} settings applied: {amplitude}mA, {pulsewidth}μs, {location}")
            else:
                print(f"Failed to apply settings for device {device_id}")
                
        except Exception as e:
            print(f"Error applying settings for device {device_id}: {e}")
    
    def start_stimulation(self, device_id):
        """Start stimulation for specific device"""
        if not self.ds8r_manager:
            print("DS8R Manager not initialized")
            return
            
        try:
            freq_spin = self.findChild(QDoubleSpinBox, f"freq_{device_id}")
            dur_spin = self.findChild(QDoubleSpinBox, f"dur_{device_id}")
            
            frequency = freq_spin.value()
            duration = dur_spin.value()
            
            success = self.ds8r_manager.send_stimulation(device_id, frequency, duration)
            if success:
                print(f"Device {device_id} stimulation started: {frequency}Hz, {duration}s")
            else:
                print(f"Failed to start stimulation for device {device_id}")
                
        except Exception as e:
            print(f"Error starting stimulation for device {device_id}: {e}")
    
    def handle_return_to_main(self):
        """Return to main screen"""
        if self.return_callback:
            self.return_callback()
    
    def get_manager(self):
        """Get the DS8R manager instance for use by tasks"""
        return self.ds8r_manager
