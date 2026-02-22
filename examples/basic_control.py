"""
Basic HDrive EtherCAT Control
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Connect to the motor, apply a small torque, print telemetry, and stop.

Usage:
    python basic_control.py

Note:
    Run ``HDriveETC.list_adapters()`` first to find the correct adapter name
    for your system.
"""

from hdrive_etc import HDriveETC, Mode
import time


def main():
    # ---- find the right adapter name ----
    HDriveETC.list_adapters()

    # ---- connect and control ----
    with HDriveETC(adapter="eth0", slave_index=0) as motor:
    # with HDriveETC(slave_index=0, pdo_config_path="ethercat_config.json") as motor:

        # Wait for the CiA 402 state machine to reach "operation_enabled"
        print("Waiting for motor to become ready...")
        for _ in range(50):
            state = motor.get_state_name()
            if state == "operation_enabled":
                break
            time.sleep(0.1)

        if motor.get_state_name() != "operation_enabled":
            print(f"Motor stuck in state: {motor.get_state_name()}")
            return

        print("Motor is ready!\n")

        # Apply a small torque
        motor.set_mode(Mode.TORQUE)
        motor.set_torque(200)  # 200 mNm

        # Print live telemetry for 5 seconds
        for _ in range(50):
            status = motor.get_status()
            print(
                f"Position: {int(status.get('position', 0)):8d}  "
                f"Velocity: {motor.get_velocity():8.1f} RPM  "
                f"Torque: {int(status.get('torque', 0)):5d}  "
                f"State: {motor.get_state_name()}"
            )
            time.sleep(0.1)

        # Communication health check
        stats = motor.get_comm_stats()
        print(f"\nComm success rate: {stats['success_rate']:.1f}%")
        print(f"Cycle time: {stats['cycle_time_actual_ms']:.2f} ms")

        # Stop the motor
        motor.stop()
        print("Motor stopped.")

    # Motor is automatically disconnected when leaving the 'with' block


if __name__ == "__main__":
    main()
