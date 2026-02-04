"""
Controls Lab - A framework for control system experiments on ArduPilot SITL.

AP_FLAKE8_CLEAN
"""

from ._version import __version__, __version_info__
from .interface.lab import ControlsLab, StepResponseResult
from .core.metrics import StepResponseMetrics, analyze_step_response
from .core.vehicle import Vehicle, Position, Attitude, Velocity
from .core.connection import SITLConnection

__all__ = [
    "__version__",
    "__version_info__",
    "ControlsLab",
    "StepResponseResult",
    "StepResponseMetrics",
    "analyze_step_response",
    "Vehicle",
    "Position",
    "Attitude",
    "Velocity",
    "SITLConnection",
]
