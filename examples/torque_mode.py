"""
Torque Control Example
~~~~~~~~~~~~~~~~~~~~~~

Apply a constant torque and monitor the resulting motion.

Usage:
    python torque_mode.py
"""

from hdrive_etc import HDriveETC, Mode
import time


def main():
    with HDriveETC(adapter=None) as motor:

        # Wait until motor is ready
        for _ in range(50):
            if motor.get_state_name() == "operation_enabled":
                break
            time.sleep(0.1)

        # Switch to torque mode
        motor.set_mode(Mode.TORQUE)

        # Ramp torque up
        print("Ramping torque from 0 to 500 mNm...")
        for torque in range(0, 501, 50):
            motor.set_torque(torque)
            time.sleep(0.5)
            print(
                f"  Setpoint: {torque:4d} mNm  |  "
                f"Actual torque: {motor.get_torque():5d}  |  "
                f"Velocity: {motor.get_velocity():.1f} RPM"
            )

        # Hold for 2 seconds
        print("\nHolding 500 mNm for 2 seconds...")
        time.sleep(2)

        # Ramp down
        print("Ramping torque down...")
        for torque in range(500, -1, -50):
            motor.set_torque(torque)
            time.sleep(0.3)

        motor.stop()
        print("Done.")


if __name__ == "__main__":
    main()
