from __future__ import annotations

import logging
from pymavlink import mavutil
from pymavlink.mavutil import mavudp

from .exceptions import MAVLinkConnectionError

logger = logging.getLogger(__name__)

class SITLConnection:
    """
    Manages the MAVLink connection to a SITL instance.
    """

    def __init__(self):
        """
        Initializes the SITLConnection instance.
        """
        self._connection = None
    
    def connect(self, host: str, port: str, timeout: int = 10) -> SITLConnection:
        """
        Establishes a UDP connection to the SITL instance.

        Args:
            host: The hostname or IP address of the SITL instance.
            port: The UDP port number.
            timeout: Maximum time in seconds to wait for a heartbeat.

        Returns:
            The SITLConnection instance itself.

        Raises:
            ConnectionError: If a heartbeat is not received within the timeout.
        """
        connection_string = f"udp:{host}:{port}"
        logger.info(f"Connecting to SITL at {connection_string}...")
        self._connection = mavutil.mavlink_connection(connection_string)
        heartbeat_msg = self._connection.wait_heartbeat(timeout=timeout)
        if not heartbeat_msg:
            logger.error(f"Failed to receive heartbeat from {connection_string} within {timeout}s")
            raise MAVLinkConnectionError("Connection timed out or could not be established.")
        
        logger.info(f"Connected to system {self._connection.target_system}, component {self._connection.target_component}")
        return self

    @property
    def connection(self) -> mavudp:
        """
        Returns the underlying MAVLink connection object.

        Raises:
            ConnectionError: If the connection has not been established.
        """
        if self._connection is None:
            raise MAVLinkConnectionError("Connection not established. Call connect() first.")
        return self._connection
    
    @connection.setter
    def connection(self, connection: mavudp):
        self._connection = connection

    def set_stream_rate(self, stream_rate: int = 50, message_ids: list[int] | None = None):
        """
        Sets the update interval for specific MAVLink messages.

        Args:
            stream_rate: Desired update rate in Hz.
            message_ids: List of MAVLink message IDs to configure. Defaults to ATTITUDE, 
                         GLOBAL_POSITION_INT, and SYS_STATUS if None.
        """
        if message_ids is None:
            message_ids = [
                mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS
            ]

        interval_us = int(1_000_000 / stream_rate)
        logger.debug(f"Setting rate to {stream_rate}Hz ({interval_us}us) for messages: {message_ids}")

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