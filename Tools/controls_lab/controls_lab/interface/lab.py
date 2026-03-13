"""
Main ControlsLab interface for running experiments.

AP_FLAKE8_CLEAN
"""
from __future__ import annotations
import time
import logging
from dataclasses import dataclass, field

from ..core.connection import SITLConnection
from ..core.vehicle import Vehicle
from ..core.metrics import analyze_step_response, StepResponseMetrics

logger = logging.getLogger(__name__)


@dataclass
class StepResponseResult:
    """Results from a step response test."""

    time: list[float] = field(default_factory=list)
    altitude: list[float] = field(default_factory=list)
    target: float = 0.0
    metrics: StepResponseMetrics | None = None


class ControlsLab:
    """
    Main interface for running control system experiments on ArduPilot SITL.

    Example:
        lab = ControlsLab()
        lab.connect()
        results = lab.run_step_response(target_alt=20.0)
        lab.plot(results)
    """

    def __init__(self):
        self._connection: SITLConnection | None = None
        self._vehicle: Vehicle | None = None

    @property
    def vehicle(self) -> Vehicle:
        """
        Get the Vehicle instance.

        Raises:
            RuntimeError: If not connected.
        """
        if self._vehicle is None:
            raise RuntimeError("Not connected. Call connect() first.")
        return self._vehicle

    def connect(
        self, host: str = "127.0.0.1", port: str = "14550", timeout: int = 30
    ):
        """
        Connect to SITL instance.

        Args:
            host: SITL hostname or IP.
            port: SITL UDP port.
            timeout: Connection timeout in seconds.
        """
        self._connection = SITLConnection()
        self._connection.connect(host, port, timeout)
        self._vehicle = Vehicle(self._connection)
        logger.info("ControlsLab connected.")

    def disconnect(self):
        """Disconnect from SITL."""
        self._connection = None
        self._vehicle = None
        logger.info("ControlsLab disconnected.")

    def run_step_response(
        self,
        target_alt: float,
        duration: float = 10.0,
        sample_rate: float = 20.0,
        initial_alt: float = 10.0
    ) -> StepResponseResult:
        """
        Run an altitude step response test.

        Args:
            target_alt: Target altitude for step (meters).
            duration: Test duration after step command (seconds).
            sample_rate: Data collection rate (Hz).
            initial_alt: Starting altitude before step (meters).

        Returns:
            StepResponseResult with time series and computed metrics.
        """
        result = StepResponseResult(target=target_alt)
        sample_interval = 1.0 / sample_rate

        logger.info(f"Setting up: takeoff to {initial_alt}m...")
        self.vehicle.arm_and_takeoff(initial_alt)

        logger.info("Stabilizing at initial altitude...")
        self._wait_for_altitude(initial_alt, tolerance=1.0, timeout=30)
        time.sleep(2.0)

        logger.info(f"Applying step to {target_alt}m...")
        self.vehicle.goto_ned(0, 0, -target_alt)

        logger.info(f"Collecting data for {duration}s...")
        start_time = time.time()
        while time.time() - start_time < duration:
            loop_start = time.time()

            pos = self.vehicle.get_position(timeout=0.5)
            if pos:
                result.time.append(time.time() - start_time)
                result.altitude.append(pos.alt)

            elapsed = time.time() - loop_start
            if elapsed < sample_interval:
                time.sleep(sample_interval - elapsed)

        if len(result.time) > 0:
            result.metrics = analyze_step_response(
                result.time, result.altitude, target_alt
            )
            logger.info(f"Metrics: {result.metrics}")

        return result

    def plot(self, result: StepResponseResult, show: bool = True):
        """
        Plot step response results.

        Args:
            result: StepResponseResult from run_step_response().
            show: Whether to display the plot immediately.

        Returns:
            matplotlib Figure object.
        """
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            logger.error("matplotlib not installed. Run: pip install matplotlib")
            return

        fig, ax = plt.subplots(figsize=(10, 6))

        ax.plot(result.time, result.altitude, 'b-', linewidth=2, label='Altitude')
        ax.axhline(
            y=result.target, color='r', linestyle='--',
            label=f'Target ({result.target}m)'
        )

        if result.metrics:
            if result.metrics.rise_time is not None:
                ax.axvline(
                    x=result.metrics.rise_time, color='g', linestyle=':',
                    alpha=0.7, label=f'Rise Time ({result.metrics.rise_time:.2f}s)'
                )
            if result.metrics.settling_time is not None:
                ax.axvline(
                    x=result.metrics.settling_time, color='orange', linestyle=':',
                    alpha=0.7,
                    label=f'Settling Time ({result.metrics.settling_time:.2f}s)'
                )

        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Altitude (m)')
        ax.set_title('Step Response')
        ax.legend()
        ax.grid(True, alpha=0.3)

        if result.metrics:
            rise_str = (
                f'Rise Time: {result.metrics.rise_time:.2f}s'
                if result.metrics.rise_time else 'Rise Time: N/A'
            )
            settle_str = (
                f'Settling: {result.metrics.settling_time:.2f}s'
                if result.metrics.settling_time else 'Settling: N/A'
            )
            textstr = '\n'.join([
                rise_str,
                f'Overshoot: {result.metrics.overshoot:.1f}%',
                settle_str,
                f'SS Error: {result.metrics.steady_state_error:.3f}m'
            ])
            props = dict(boxstyle='round', facecolor='wheat', alpha=0.5)
            ax.text(
                0.02, 0.98, textstr, transform=ax.transAxes, fontsize=9,
                verticalalignment='top', bbox=props
            )

        plt.tight_layout()

        if show:
            plt.show()

        return fig

    def _wait_for_altitude(
        self, target: float, tolerance: float = 0.5, timeout: float = 30
    ):
        """Wait until vehicle reaches target altitude."""
        start = time.time()
        while time.time() - start < timeout:
            pos = self.vehicle.get_position()
            if pos and abs(pos.alt - target) < tolerance:
                return
            time.sleep(0.5)
        logger.warning(f"Altitude wait timed out after {timeout}s")
