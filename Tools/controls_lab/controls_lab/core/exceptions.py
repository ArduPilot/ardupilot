"""
Custom exceptions for the controls_lab package.

AP_FLAKE8_CLEAN
"""


class ControlsLabError(Exception):
    """Base class for all exceptions in controls_lab."""

    pass


class MAVLinkConnectionError(ControlsLabError):
    """Raised when the MAVLink connection cannot be established or is lost."""

    pass


# Deprecated alias for backward compatibility
ConnectionError = MAVLinkConnectionError
