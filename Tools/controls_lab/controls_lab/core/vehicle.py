"""
High-level vehicle control interface for ArduPilot.

AP_FLAKE8_CLEAN
"""
from __future__ import annotations
import time
import logging
import math
from dataclasses import dataclass
from pymavlink import mavutil

from .connection import SITLConnection
from .exceptions import MAVLinkConnectionError

logger = logging.getLogger(__name__)


@dataclass
class Position:
    """Vehicle position in global coordinates."""

    lat: float
    lon: float
    alt: float


@dataclass
class Attitude:
    """Vehicle attitude in radians."""

    roll: float
    pitch: float
    yaw: float


@dataclass
class Velocity:
    """Vehicle velocity in m/s (NED frame)."""

    vn: float
    ve: float
    vd: float


class Vehicle:
    """High-level control interface for an ArduPilot vehicle."""

    def __init__(self, connection: SITLConnection):
        self.sitl = connection

    @property
    def _mav(self):
        return self.sitl.connection.mav

    @property
    def _target_system(self):
        return self.sitl.connection.target_system

    @property
    def _target_component(self):
        return self.sitl.connection.target_component

    def set_mode(self, mode: str, timeout: int = 5):
        """
        Set the flight mode of the vehicle.

        Available modes depend on vehicle type and are queried dynamically.
        Examples: 'GUIDED', 'LOITER', 'ALT_HOLD', 'STABILIZE'.

        Args:
            mode: Flight mode name (e.g., 'GUIDED', 'LOITER').
            timeout: Maximum seconds to wait for mode confirmation.

        Raises:
            ValueError: If mode name is not recognized.
            MAVLinkConnectionError: If mode change is not confirmed in timeout.
        """
        mode_num = self.sitl.get_mode_number(mode)
        logger.info(f"Switching to mode: {mode}")

        self._mav.command_long_send(
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_num,
            0, 0, 0, 0, 0
        )

        start_time = time.time()
        while time.time() - start_time < timeout:
            msg = self.sitl.connection.recv_match(
                type='HEARTBEAT', blocking=True, timeout=1
            )
            if msg and msg.custom_mode == mode_num:
                logger.info(f"Mode switched to {mode}")
                return

        raise MAVLinkConnectionError(
            f"Failed to switch to mode {mode} within {timeout}s"
        )

    def arm(self, wait: bool = True):
        """
        Arm the vehicle motors.

        Args:
            wait: Block until arming is confirmed.
        """
        logger.info("Arming...")
        self.sitl.connection.arducopter_arm()

        if wait:
            self.sitl.connection.motors_armed_wait()
            logger.info("Armed.")

    def disarm(self, wait: bool = True):
        """
        Disarm the vehicle motors.

        Args:
            wait: Block until disarming is confirmed.
        """
        logger.info("Disarming...")
        self.sitl.connection.arducopter_disarm()

        if wait:
            self.sitl.connection.motors_disarmed_wait()
            logger.info("Disarmed.")

    def takeoff(self, altitude: float, wait: bool = True, timeout: int = 10):
        """
        Command the vehicle to take off.

        Args:
            altitude: Target altitude in meters.
            wait: Block until command is acknowledged.
            timeout: Maximum seconds to wait for acknowledgment.

        Raises:
            MAVLinkConnectionError: If ACK not received or command rejected.
        """
        logger.info(f"Taking off to {altitude}m...")
        self._mav.command_long_send(
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )

        if wait:
            ack = self.sitl.connection.recv_match(
                type='COMMAND_ACK', blocking=True, timeout=timeout
            )
            if not ack:
                raise MAVLinkConnectionError(
                    "No ACK received for takeoff command."
                )
            if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                raise MAVLinkConnectionError(
                    f"Takeoff rejected with result: {ack.result}"
                )
            logger.info("Takeoff command accepted.")

    def land(self, wait: bool = True, timeout: int = 10):
        """
        Command the vehicle to land at current position.

        Args:
            wait: Block until command is acknowledged.
            timeout: Maximum seconds to wait for acknowledgment.

        Raises:
            MAVLinkConnectionError: If ACK not received or command rejected.
        """
        logger.info("Landing...")
        self._mav.command_long_send(
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )

        if wait:
            ack = self.sitl.connection.recv_match(
                type='COMMAND_ACK', blocking=True, timeout=timeout
            )
            if not ack:
                raise MAVLinkConnectionError("No ACK received for land command.")
            if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
                raise MAVLinkConnectionError(
                    f"Land rejected with result: {ack.result}"
                )
            logger.info("Land command accepted.")

    def goto(self, lat: float, lon: float, alt: float):
        """
        Command the vehicle to fly to a global position (GUIDED mode).

        Args:
            lat: Target latitude in degrees.
            lon: Target longitude in degrees.
            alt: Target altitude in meters (relative to home).
        """
        logger.info(f"Going to ({lat}, {lon}, {alt}m)...")
        self._mav.set_position_target_global_int_send(
            0,
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

    def goto_ned(self, north: float, east: float, down: float):
        """
        Command the vehicle to fly to a local NED position (GUIDED mode).

        Args:
            north: Meters north of home.
            east: Meters east of home.
            down: Meters down from home (negative = up).
        """
        logger.info(f"Going to NED ({north}, {east}, {down})...")
        self._mav.set_position_target_local_ned_send(
            0,
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111111000,
            north, east, down,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

    def set_velocity(self, vn: float, ve: float, vd: float):
        """
        Set velocity setpoint in NED frame (GUIDED mode).

        Args:
            vn: Velocity north in m/s.
            ve: Velocity east in m/s.
            vd: Velocity down in m/s (negative = up).
        """
        self._mav.set_position_target_local_ned_send(
            0,
            self._target_system,
            self._target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0, 0, 0,
            vn, ve, vd,
            0, 0, 0,
            0, 0
        )

    def set_attitude(self, roll: float, pitch: float, yaw: float, thrust: float):
        """
        Set attitude and thrust setpoint (GUIDED mode).

        Args:
            roll: Roll angle in radians.
            pitch: Pitch angle in radians.
            yaw: Yaw angle in radians.
            thrust: Thrust 0.0 to 1.0.
        """
        q = self._euler_to_quaternion(roll, pitch, yaw)

        self._mav.set_attitude_target_send(
            0,
            self._target_system,
            self._target_component,
            0b00000111,
            q,
            0, 0, 0,
            thrust
        )

    def get_position(self, timeout: float = 1.0) -> Position | None:
        """
        Get current vehicle position.

        Args:
            timeout: Maximum seconds to wait for position message.

        Returns:
            Position dataclass or None if no message received.
        """
        msg = self.sitl.connection.recv_match(
            type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout
        )
        if not msg:
            return None

        return Position(
            lat=msg.lat / 1e7,
            lon=msg.lon / 1e7,
            alt=msg.relative_alt / 1000.0
        )

    def get_attitude(self, timeout: float = 1.0) -> Attitude | None:
        """
        Get current vehicle attitude.

        Args:
            timeout: Maximum seconds to wait for attitude message.

        Returns:
            Attitude dataclass or None if no message received.
        """
        msg = self.sitl.connection.recv_match(
            type='ATTITUDE', blocking=True, timeout=timeout
        )
        if not msg:
            return None

        return Attitude(roll=msg.roll, pitch=msg.pitch, yaw=msg.yaw)

    def get_velocity(self, timeout: float = 1.0) -> Velocity | None:
        """
        Get current vehicle velocity.

        Args:
            timeout: Maximum seconds to wait for velocity message.

        Returns:
            Velocity dataclass or None if no message received.
        """
        msg = self.sitl.connection.recv_match(
            type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout
        )
        if not msg:
            return None

        return Velocity(
            vn=msg.vx / 100.0,
            ve=msg.vy / 100.0,
            vd=msg.vz / 100.0
        )

    def arm_and_takeoff(self, altitude: float = 10.0, mode: str = "GUIDED"):
        """
        Switch mode, arm, and take off in sequence.

        Args:
            altitude: Target altitude in meters.
            mode: Flight mode name for takeoff. Defaults to "GUIDED".
        """
        self.set_mode(mode)
        self.arm()
        self.takeoff(altitude)

    @staticmethod
    def _euler_to_quaternion(roll: float, pitch: float, yaw: float) -> list[float]:
        """Convert euler angles to quaternion [w, x, y, z]."""
        cr = math.cos(roll / 2)
        sr = math.sin(roll / 2)
        cp = math.cos(pitch / 2)
        sp = math.sin(pitch / 2)
        cy = math.cos(yaw / 2)
        sy = math.sin(yaw / 2)

        return [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy
        ]
