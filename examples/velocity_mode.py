"""
Velocity Control Example
~~~~~~~~~~~~~~~~~~~~~~~~

Spin the motor at a constant speed, then reverse, then stop.

Usage:
    python velocity_mode.py
"""

from hdrive_etc import HDriveETC, Mode
import time


def main():
    with HDriveETC(adapter="eth0", slave_index=0) as motor:
    # with HDriveETC(slave_index=0, pdo_config_path="ethercat_config.json") as motor:

        # Wait until motor is ready
        for _ in range(50):
            if motor.get_state_name() == "operation_enabled":
                break
            time.sleep(0.1)

        motor.set_mode(Mode.VELOCITY)

        # Forward
        print("Spinning forward at velocity 300...")
        motor.set_velocity(300)
        for _ in range(30):
            print(
                f"  Velocity: {motor.get_velocity():8.1f} RPM  |  "
                f"Position: {int(motor.get_position()):8d}"
            )
            time.sleep(0.1)

        # Reverse
        print("\nReversing to -300...")
        motor.set_velocity(-300)
        for _ in range(30):
            print(
                f"  Velocity: {motor.get_velocity():8.1f} RPM  |  "
                f"Position: {int(motor.get_position()):8d}"
            )
            time.sleep(0.1)

        motor.stop()
        print("\nMotor stopped.")


if __name__ == "__main__":
    main()
