# Copyright 2025 ArduPilot.org.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.

"""
Bring up ArduPilot SITL and check the airspeed commands result in changed airspeed.

colcon test --executor sequential --parallel-workers 0 \
    --packages-select ardupilot_dds_tests \
    --event-handlers=console_cohesion+ --pytest-args -k test_plane_airspeed
"""


import launch_pytest
import pytest
import rclpy
import rclpy.node
import threading

from launch import LaunchDescription

from launch_pytest.tools import process as process_tools

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy

from ardupilot_msgs.msg import Airspeed

TOPIC = "/ap/airspeed"
WAIT_FOR_START_TIMEOUT = 5.0
AIRSPEED_RECV_TIMEOUT = 20.0


class AirspeedTester(rclpy.node.Node):
    """Subscribe to Airspeed messages."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("airspeed_listener")
        self.msg_event_object = threading.Event()

    def start_subscriber(self):
        """Start the subscriber."""
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(Airspeed, TOPIC, self.airspeed_data_callback, qos_profile)

        # Add a spin thread.
        self.ros_spin_thread = threading.Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def airspeed_data_callback(self, msg):
        """Process a airspeed message from the autopilot."""
        if self.msg_event_object.set():
            return

        self.get_logger().info(f"From AP : airspeed {msg}")

        # set event last
        self.msg_event_object.set()


@launch_pytest.fixture
def launch_sitl_copter_dds_serial(sitl_copter_dds_serial):
    """Fixture to create the launch description."""
    sitl_ld, sitl_actions = sitl_copter_dds_serial

    ld = LaunchDescription(
        [
            sitl_ld,
            launch_pytest.actions.ReadyToTest(),
        ]
    )
    actions = sitl_actions
    yield ld, actions


@launch_pytest.fixture
def launch_sitl_copter_dds_udp(sitl_copter_dds_udp):
    """Fixture to create the launch description."""
    sitl_ld, sitl_actions = sitl_copter_dds_udp

    ld = LaunchDescription(
        [
            sitl_ld,
            launch_pytest.actions.ReadyToTest(),
        ]
    )
    actions = sitl_actions
    yield ld, actions


@launch_pytest.fixture
def launch_sitl_plane_dds_serial(sitl_plane_dds_serial):
    """Fixture to create the launch description."""
    sitl_ld, sitl_actions = sitl_plane_dds_serial

    ld = LaunchDescription(
        [
            sitl_ld,
            launch_pytest.actions.ReadyToTest(),
        ]
    )
    actions = sitl_actions
    yield ld, actions


@launch_pytest.fixture
def launch_sitl_plane_dds_udp(sitl_plane_dds_udp):
    """Fixture to create the launch description."""
    sitl_ld, sitl_actions = sitl_plane_dds_udp

    ld = LaunchDescription(
        [
            sitl_ld,
            launch_pytest.actions.ReadyToTest(),
        ]
    )
    actions = sitl_actions
    yield ld, actions


@pytest.mark.launch(fixture=launch_sitl_copter_dds_serial)
def test_dds_serial_airspeed_msg_recv_copter(launch_context, launch_sitl_copter_dds_serial):
    """Test airspeed messages are published by AP_DDS."""
    _, actions = launch_sitl_copter_dds_serial
    virtual_ports = actions["virtual_ports"].action
    micro_ros_agent = actions["micro_ros_agent"].action
    mavproxy = actions["mavproxy"].action
    sitl = actions["sitl"].action

    # Wait for process to start.
    process_tools.wait_for_start_sync(launch_context, virtual_ports, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, micro_ros_agent, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, mavproxy, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, sitl, timeout=WAIT_FOR_START_TIMEOUT)

    rclpy.init()
    try:
        node = AirspeedTester()
        node.start_subscriber()
        msgs_received_flag = node.msg_event_object.wait(timeout=AIRSPEED_RECV_TIMEOUT)
        assert msgs_received_flag, f"Did not receive '{TOPIC}' msgs."
    finally:
        rclpy.shutdown()
    yield


@pytest.mark.launch(fixture=launch_sitl_copter_dds_udp)
def test_dds_udp_airspeed_msg_recv_copter(launch_context, launch_sitl_copter_dds_udp):
    """Test airspeed messages are published by AP_DDS."""
    _, actions = launch_sitl_copter_dds_udp
    micro_ros_agent = actions["micro_ros_agent"].action
    mavproxy = actions["mavproxy"].action
    sitl = actions["sitl"].action

    # Wait for process to start.
    process_tools.wait_for_start_sync(launch_context, micro_ros_agent, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, mavproxy, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, sitl, timeout=WAIT_FOR_START_TIMEOUT)

    rclpy.init()
    try:
        node = AirspeedTester()
        node.start_subscriber()
        msgs_received_flag = node.msg_event_object.wait(timeout=AIRSPEED_RECV_TIMEOUT)
        assert msgs_received_flag, f"Did not receive '{TOPIC}' msgs."

    finally:
        rclpy.shutdown()
    yield


@pytest.mark.launch(fixture=launch_sitl_plane_dds_serial)
def test_dds_serial_airspeed_msg_recv_plane(launch_context, launch_sitl_plane_dds_serial):
    """Test airspeed messages are published by AP_DDS."""
    _, actions = launch_sitl_plane_dds_serial
    virtual_ports = actions["virtual_ports"].action
    micro_ros_agent = actions["micro_ros_agent"].action
    mavproxy = actions["mavproxy"].action
    sitl = actions["sitl"].action

    # Wait for process to start.
    process_tools.wait_for_start_sync(launch_context, virtual_ports, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, micro_ros_agent, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, mavproxy, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, sitl, timeout=WAIT_FOR_START_TIMEOUT)

    rclpy.init()
    try:
        node = AirspeedTester()
        node.start_subscriber()
        msgs_received_flag = node.msg_event_object.wait(timeout=AIRSPEED_RECV_TIMEOUT)
        assert msgs_received_flag, f"Did not receive '{TOPIC}' msgs."
    finally:
        rclpy.shutdown()
    yield


@pytest.mark.launch(fixture=launch_sitl_plane_dds_udp)
def test_dds_udp_airspeed_msg_recv_plane(launch_context, launch_sitl_plane_dds_udp):
    """Test airspeed messages are published by AP_DDS."""
    _, actions = launch_sitl_plane_dds_udp
    micro_ros_agent = actions["micro_ros_agent"].action
    mavproxy = actions["mavproxy"].action
    sitl = actions["sitl"].action

    # Wait for process to start.
    process_tools.wait_for_start_sync(launch_context, micro_ros_agent, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, mavproxy, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, sitl, timeout=WAIT_FOR_START_TIMEOUT)

    rclpy.init()
    try:
        node = AirspeedTester()
        node.start_subscriber()
        msgs_received_flag = node.msg_event_object.wait(timeout=AIRSPEED_RECV_TIMEOUT)
        assert msgs_received_flag, f"Did not receive '{TOPIC}' msgs."

    finally:
        rclpy.shutdown()
    yield
