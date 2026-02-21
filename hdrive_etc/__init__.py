"""
HDrive EtherCAT SDK
~~~~~~~~~~~~~~~~~~~

Python SDK for Henschel Robotics HDrive servo motors over EtherCAT.
Requires PySOEM for EtherCAT communication.

Basic usage (standalone — one motor)::

    from hdrive_etc import HDriveETC, Mode
    import time

    with HDriveETC(adapter=r"\\Device\\NPF_{...}") as motor:
        motor.set_mode(Mode.TORQUE)
        motor.set_torque(200)
        time.sleep(2)
        motor.stop()

Multi-motor usage (shared bus)::

    from hdrive_etc import EtherCATBus, HDriveETC, Mode

    bus = EtherCATBus(adapter=r"\\Device\\NPF_{...}", cycle_time_ms=1)
    motor0 = HDriveETC(slave_index=0, bus=bus)
    motor1 = HDriveETC(slave_index=1, bus=bus)
    bus.open()
    # ... control motor0 and motor1 ...
    bus.close()

:copyright: (c) Henschel Robotics GmbH
:license: MIT
"""

from .bus import EtherCATBus
from .motor import HDriveETC, Mode, Error
from .exceptions import (
    HDriveError,
    ConnectionError,
    CommunicationError,
    StateError,
    ConfigurationError,
)

__version__ = "0.2.0"
__all__ = [
    "EtherCATBus",
    "HDriveETC",
    "Mode",
    "Error",
    "HDriveError",
    "ConnectionError",
    "CommunicationError",
    "StateError",
    "ConfigurationError",
    "__version__",
]
