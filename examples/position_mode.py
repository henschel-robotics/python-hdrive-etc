"""
Position Control Example
~~~~~~~~~~~~~~~~~~~~~~~~

Move the motor to several target positions using cyclic synchronous
position mode (CSP).

Usage:
    python position_mode.py
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

        motor.set_mode(Mode.POSITION)

        # Move to a sequence of positions
        targets = [10000, 50000, 0, -25000, 0]

        for target in targets:
            print(f"Moving to position {target}...")
            motor.set_position(target)

            # Wait until close to target (simple polling)
            for _ in range(100):
                pos = motor.get_position()
                error = abs(target - pos)
                print(f"  Target: {target:8d}  Actual: {pos:8d}  Error: {error:6d}")
                if error < 100:
                    print("  Reached target!")
                    break
                time.sleep(0.05)

            time.sleep(0.5)  # short pause between moves

        motor.stop()
        print("\nDone.")


if __name__ == "__main__":
    main()
