"""
MAVLink connection management for SITL instances.

AP_FLAKE8_CLEAN
"""
from __future__ import annotations

import logging
from pymavlink import mavutil
from pymavlink.mavutil import mavudp

from .exceptions import MAVLinkConnectionError

logger = logging.getLogger(__name__)


class SITLConnection:
    """Manages the MAVLink connection to a SITL instance."""

    def __init__(self):
        """Initialize the SITLConnection instance."""
        self._connection = None

    def connect(self, host: str, port: str, timeout: int = 10) -> SITLConnection:
        """
        Establish a UDP connection to the SITL instance.

        Args:
            host: The hostname or IP address of the SITL instance.
            port: The UDP port number.
            timeout: Maximum time in seconds to wait for a heartbeat.

        Returns:
            The SITLConnection instance itself.

        Raises:
            MAVLinkConnectionError: If heartbeat is not received within timeout.
        """
        connection_string = f"udp:{host}:{port}"
        logger.info(f"Connecting to SITL at {connection_string}...")
        self._connection = mavutil.mavlink_connection(connection_string)
        heartbeat_msg = self._connection.wait_heartbeat(timeout=timeout)
        if not heartbeat_msg:
            logger.error(
                f"Failed to receive heartbeat from {connection_string} "
                f"within {timeout}s"
            )
            raise MAVLinkConnectionError(
                "Connection timed out or could not be established."
            )

        logger.info(
            f"Connected to system {self._connection.target_system}, "
            f"component {self._connection.target_component}"
        )
        return self

    @property
    def connection(self) -> mavudp:
        """
        Return the underlying MAVLink connection object.

        Raises:
            MAVLinkConnectionError: If the connection has not been established.
        """
        if self._connection is None:
            raise MAVLinkConnectionError(
                "Connection not established. Call connect() first."
            )
        return self._connection

    @connection.setter
    def connection(self, connection: mavudp):
        self._connection = connection

    def mode_mapping(self) -> dict[str, int]:
        """
        Get available flight modes for the connected vehicle.

        Returns:
            Dictionary mapping mode names to mode numbers.

        Raises:
            MAVLinkConnectionError: If no mode mapping is available.
        """
        mapping = self.connection.mode_mapping()
        if mapping is None:
            raise MAVLinkConnectionError("No mode mapping available")
        return mapping

    def get_mode_number(self, mode: str | int) -> int:
        """
        Convert mode name to number, or validate mode number.

        Args:
            mode: Mode name (e.g., 'GUIDED') or mode number.

        Returns:
            The mode number.

        Raises:
            ValueError: If mode name is not recognized.
        """
        if isinstance(mode, int):
            return mode
        mapping = self.mode_mapping()
        if mode not in mapping:
            available = ', '.join(sorted(mapping.keys()))
            raise ValueError(f"Unknown mode '{mode}'. Available: {available}")
        return mapping[mode]

    def set_stream_rate(
        self, stream_rate: int = 50, message_ids: list[int] | None = None
    ):
        """
        Set the update interval for specific MAVLink messages.

        Args:
            stream_rate: Desired update rate in Hz.
            message_ids: List of MAVLink message IDs to configure. Defaults to
                ATTITUDE, GLOBAL_POSITION_INT, and SYS_STATUS if None.
        """
        if message_ids is None:
            message_ids = [
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS
            ]

        interval_us = int(1_000_000 / stream_rate)
        logger.debug(
            f"Setting rate to {stream_rate}Hz ({interval_us}us) "
            f"for messages: {message_ids}"
        )

        for msg_id in message_ids:
            self.connection.mav.command_long_send(
                self.connection.target_system,
                self.connection.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                msg_id,
                interval_us,
                0, 0, 0, 0, 0
            )
