"""
Read SDO Object Example
~~~~~~~~~~~~~~~~~~~~~~~

Read configuration objects from the drive via SDO (Service Data Object).

Usage:
    python read_sdo.py
"""

from hdrive_etc import HDriveETC
import time


def main():
    with HDriveETC(adapter_index=0) as motor:

        # Wait until motor is ready
        for _ in range(50):
            if motor.get_state_name() == "operation_enabled":
                break
            time.sleep(0.1)

        # Read error code
        error_code = motor.get_error_code()
        print(f"Error code: {error_code} ({motor.get_error_message(error_code)})")

        # Read some SDO objects
        # Adjust indices/subindices for your motor configuration
        value = motor.read_sdo(0x6060, 0x00, "b")  # Modes of operation
        print(f"Modes of operation (0x6060): {value}")

        value = motor.read_sdo(0x6061, 0x00, "b")  # Modes of operation display
        print(f"Modes of operation display (0x6061): {value}")

        # Write an SDO (example: set a parameter)
        # motor.write_sdo(0x6640, 0x01, 100)  # Set torque bandwidth
        # print("Torque bandwidth set to 100")

        motor.stop()
        print("Done.")


if __name__ == "__main__":
    main()
