# Arduino DS8R Manager

A Python-based graphical interface for controlling Digitimer DS8R neurostimulation devices through DLL and Arduino communication. This project provides a unified management system for multiple DS8R devices with both hardware integration and simulation capabilities.

## Features

- **Multi-Device Support**: Manage up to 3 DS8R devices simultaneously
- **Arduino Integration**: Communicate with DS8R devices through Arduino interface
- **Graphical User Interface**: User-friendly PySide6-based GUI with tabbed interface
- **Real-time Control**: Set stimulation parameters (frequency, duration, amplitude, pulse width)
- **Device Status Monitoring**: Real-time connection and device status feedback
- **Simulation Mode**: Full functionality testing without hardware requirements
- **Rate Limiting**: Compliant with DS8R API 10Hz communication limits
- **Error Handling**: Robust error detection and recovery mechanisms

## Hardware Requirements

### DS8R Devices
- Digitimer DS8R constant current stimulators
- Compatible with device IDs: 348 (Cymba), 362 (Earlobe), 387 (Arm)
- Windows system with D128API.dll support

### Arduino Setup
- Arduino Uno or compatible board
- USB connection to host computer
- Custom firmware for DS8R communication (see `Updated_ds8r_arduino/` folder)

### System Requirements
- Windows OS (for Digitimer DLL support)
- Python 3.8+ 
- USB ports for Arduino connection

## Installation

1. **Clone the repository**:
   ```bash
   git clone https://github.com/yourusername/arduino-ds8r-manager.git
   cd arduino-ds8r-manager
   ```

2. **Install Python dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

3. **Upload Arduino firmware**:
   - Open `Updated_ds8r_arduino/V2_updated_ds8r_arduino/V2_updated_ds8r_arduino.ino` in Arduino IDE
   - Upload to your Arduino Uno

4. **Install Digitimer drivers** (for hardware mode):
   - Install official Digitimer DS8R drivers
   - Ensure `D128API.dll` is available in `C:\Windows\System32\`

## Usage

### Running with Hardware
```bash
python arduino_ds8r_manager_standalone.py
```

### Running in Simulation Mode
```bash
python arduino_ds8r_manager_standalone.py --simulate
```

### Basic Operation

1. **Launch the application**
2. **Device Setup**: 
   - The system automatically detects connected Arduino and DS8R devices
   - Each device appears in a separate tab (Cymba, Earlobe, Arm)
3. **Set Parameters**:
   - **Frequency**: 1-200 Hz stimulation frequency
   - **Duration**: 0.1-10.0 seconds stimulation duration  
   - **Amplitude**: 0.1-50.0 mA current amplitude
   - **Pulse Width**: 50-1000 μs pulse duration
4. **Apply Settings**: Click "Apply Settings" to configure device parameters
5. **Start Stimulation**: Click "Start Stimulation" to begin neurostimulation

### Device Mapping
- **Device 0**: DS8R ID 348 (Cymba location)
- **Device 1**: DS8R ID 362 (Earlobe location)  
- **Device 2**: DS8R ID 387 (Arm location)

## Architecture

### Core Components

- **`Arduino_DS8R_Manager`**: Unified manager for all DS8R devices
- **`DS8RSetupPanel_Manager`**: GUI interface with tabbed device controls
- **`Arduino_DS8R`**: Individual device communication class
- **Mock Classes**: Simulation support for testing without hardware

### Communication Flow
1. GUI → Arduino_DS8R_Manager → Arduino (Serial) → DS8R Device
2. Arduino_DS8R_Manager ← D128API ← DS8R Device (status/feedback)

### Safety Features
- **Rate Limiting**: Respects 10Hz API communication limits
- **Circuit Breaker**: Automatic failure detection and recovery
- **Connection Health Monitoring**: Continuous connection validation
- **Parameter Validation**: Input range checking and sanitization

## Configuration

### Device Parameters
```python
devices = {
    0: {
        "target_id": 348,
        "location": "Cymba",
        "arduino_id": 0
    },
    1: {
        "target_id": 362, 
        "location": "Earlobe",
        "arduino_id": 1
    },
    2: {
        "target_id": 387,
        "location": "Arm", 
        "arduino_id": 2
    }
}
```

### Communication Settings
- **Serial Baud Rate**: 9600
- **Serial Timeout**: 1 second
- **DLL Rate Limit**: 150ms minimum interval
- **Connection Retry**: 3 attempts with exponential backoff

## API Reference

### Arduino_DS8R_Manager Methods

- `initialize_connection()`: Initialize Arduino and DS8R connections
- `send_stimulation(device_id, frequency, duration)`: Trigger stimulation
- `set_stimulation_params(device_id, frequency, duration)`: Configure parameters
- `set_amplitude_and_pulsewidth(device_id, amplitude_ma, pulsewidth_us)`: Set output parameters
- `stop_stimulation(device_id)`: Stop current stimulation
- `get_device_status(device_id)`: Query device status
- `test_connection(device_id)`: Validate device connectivity

### Error Codes
- `100002`: Device communication error
- `100003`: Invalid parameter range  
- `100018`: Device not found
- `100019`: Connection timeout
- `100020`: Hardware initialization failure

## Development

### Project Structure
```
arduino_ds8r_manager/
├── arduino_ds8r_manager.py          # Main manager class
├── arduino_ds8r_manager_standalone.py # Standalone executable
├── arduino_ds8r.py                   # Individual device classes
├── Updated_ds8r_arduino/             # Arduino firmware
│   └── V2_updated_ds8r_arduino/
│       └── V2_updated_ds8r_arduino.ino
├── assets/                           # UI resources
│   └── icons/
├── requirements.txt                  # Python dependencies
└── README.md                        # This file
```

### Contributing
1. Fork the repository
2. Create a feature branch (`git checkout -b feature/new-feature`)
3. Commit changes (`git commit -am 'Add new feature'`)
4. Push to branch (`git push origin feature/new-feature`)
5. Create Pull Request

### Testing
```bash
# Run in simulation mode for testing
python arduino_ds8r_manager_standalone.py --simulate

# Test individual components
python -c "from arduino_ds8r_manager import Arduino_DS8R_Manager; mgr = Arduino_DS8R_Manager()"
```

## Troubleshooting

### Common Issues

**"No Arduino found"**
- Verify Arduino is connected and recognized by system
- Check Device Manager for COM port assignment
- Ensure Arduino firmware is uploaded correctly

**"DS8R device not found"** 
- Verify DS8R devices are powered and connected
- Check Digitimer driver installation
- Confirm D128API.dll is in System32

**"Connection timeout"**
- Try reducing communication frequency
- Check USB cable connections
- Restart application and reconnect devices

**GUI not responding**
- Install/update PySide6: `pip install --upgrade PySide6`
- Check Python version compatibility (3.8+)

### Debug Mode
Enable verbose logging by modifying the code:
```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

## License

This project is open source and available under the [MIT License](LICENSE).

## Acknowledgments

- Digitimer Ltd. for DS8R hardware and API documentation
- Arduino community for microcontroller support
- Qt/PySide6 for cross-platform GUI framework

## Disclaimer

This software is intended for research purposes only. Users are responsible for ensuring compliance with all applicable regulations and safety protocols when using neurostimulation equipment. Always follow proper medical device handling procedures and institutional guidelines.
