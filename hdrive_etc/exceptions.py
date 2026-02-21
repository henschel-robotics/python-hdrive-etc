"""
HDrive EtherCAT SDK — Custom exceptions

HDrive exceptions extend the generic ``ethercat_master`` exceptions
so that code catching either base class will work.
"""

import ethercat_master.exceptions as _ec


class HDriveError(_ec.EtherCATError):
    """Base exception for all HDrive EtherCAT errors."""
    pass


class ConnectionError(HDriveError, _ec.ConnectionError):
    """Raised when the EtherCAT connection cannot be established."""
    pass


class CommunicationError(HDriveError, _ec.CommunicationError):
    """Raised when EtherCAT communication fails or times out."""
    pass


class StateError(HDriveError):
    """Raised when the motor is in an unexpected CiA 402 state."""
    pass


class ConfigurationError(HDriveError, _ec.ConfigurationError):
    """Raised when PDO mapping or slave configuration fails."""
    pass
