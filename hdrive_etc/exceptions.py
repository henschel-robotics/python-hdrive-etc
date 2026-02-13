"""
HDrive EtherCAT SDK — Custom exceptions
"""


class HDriveError(Exception):
    """Base exception for all HDrive EtherCAT errors."""
    pass


class ConnectionError(HDriveError):
    """Raised when the EtherCAT connection cannot be established."""
    pass


class CommunicationError(HDriveError):
    """Raised when EtherCAT communication fails or times out."""
    pass


class StateError(HDriveError):
    """Raised when the motor is in an unexpected CiA 402 state."""
    pass


class ConfigurationError(HDriveError):
    """Raised when PDO mapping or slave configuration fails."""
    pass
