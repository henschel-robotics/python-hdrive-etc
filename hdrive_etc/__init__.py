"""
HDrive EtherCAT SDK
~~~~~~~~~~~~~~~~~~~

Python SDK for Henschel Robotics HDrive servo motors over EtherCAT.
Requires PySOEM for EtherCAT communication.

Basic usage::

    from hdrive_etc import HDriveETC, Mode
    import time

    with HDriveETC(adapter_index=0) as motor:
        motor.set_mode(Mode.TORQUE)
        motor.set_torque(200)
        time.sleep(2)
        motor.stop()

:copyright: (c) Henschel Robotics GmbH
:license: MIT
"""

from .motor import HDriveETC, Mode
from .exceptions import (
    HDriveError,
    ConnectionError,
    CommunicationError,
    StateError,
    ConfigurationError,
)

__version__ = "0.1.1"
__all__ = [
    "HDriveETC",
    "Mode",
    "HDriveError",
    "ConnectionError",
    "CommunicationError",
    "StateError",
    "ConfigurationError",
    "__version__",
]
