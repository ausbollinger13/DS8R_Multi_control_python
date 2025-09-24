import sys
import time
import threading
import argparse
import os
try:
    from PySide6.QtWidgets import QApplication
except Exception:
    raise RuntimeError("PySide6 is required to run this standalone manager. Install with `pip install PySide6`")

"""
Standalone version of `arduino_ds8r_manager.py` suitable for open-source release.

This file provides a safe-to-publish manager and PySide6 UI that can run without
the proprietary Digitimer DLL or actual Arduino hardware. When hardware is not
available it falls back to mock implementations so the UI and logic can be
exercised.

Usage:
  python arduino_ds8r_manager_standalone.py [--simulate]

"""

from arduino_ds8r_manager import DS8RSetupPanel_Manager, Arduino_DS8R_Manager


class MockSerial:
    """Minimal mock for serial.Serial used for simulation and testing."""
    def __init__(self, *args, **kwargs):
        self.is_open = True
        self._buffer = b""

    def write(self, data: bytes):
        print(f"[MockSerial] write: {data!r}")

    def flush(self):
        pass

    def readline(self):
        time.sleep(0.01)
        return b"ACK\n"

    @property
    def in_waiting(self):
        return 0

    def close(self):
        self.is_open = False


class MockD128API:
    """Mock of the D128API used by the manager to allow safe open-source release."""
    def __init__(self):
        self._devices = []

    def Initialise(self):
        return 0, 0

    def DeviceCount(self):
        return 0

    def GetState(self):
        return True

    def Close(self):
        return 0

    def set_amplitude_and_width(self, idx, amp, width):
        print(f"[MockD128API] set_amplitude_and_width idx={idx} amp={amp} width={width}")

    def Device(self, i):
        raise IndexError

    def recover_connection(self):
        pass


def install_mocks(manager: Arduino_DS8R_Manager):
    """Replace hardware interfaces on the manager with mocks for safe running."""
    manager.ser = MockSerial()
    manager.d128api = MockD128API()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--simulate", action="store_true", help="Force simulation mode (use mock serial and D128API)")
    args = parser.parse_args(argv)

    app = QApplication([])
    panel = DS8RSetupPanel_Manager()

    # Ensure that the internal manager exists
    if panel.ds8r_manager is None:
        panel.ds8r_manager = Arduino_DS8R_Manager()

    if args.simulate:
        install_mocks(panel.ds8r_manager)
    else:
        # Try to initialize; if it fails, fall back to mocks so the UI still runs
        try:
            ok = panel.ds8r_manager.initialize_connection()
            if not ok:
                print("Initialization failed — running in simulation mode")
                install_mocks(panel.ds8r_manager)
        except Exception as e:
            print(f"Initialization raised: {e} — running in simulation mode")
            install_mocks(panel.ds8r_manager)

    panel.update_device_statuses()
    panel.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
