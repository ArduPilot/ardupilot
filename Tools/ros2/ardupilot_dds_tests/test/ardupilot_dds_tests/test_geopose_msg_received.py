# Copyright 2023 ArduPilot.org.
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

"""Bring up ArduPilot SITL and check the GeoPose message is being published."""

import launch_pytest
import math
import pytest
import rclpy
import rclpy.node
from scipy.spatial.transform import Rotation as R
import threading

from launch import LaunchDescription

from launch_pytest.tools import process as process_tools

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy

from geographic_msgs.msg import GeoPoseStamped

TOPIC = "ap/geopose/filtered"
# Copied from locations.txt
CMAC_LAT = -35.363261
CMAC_LON = 149.165230
CMAC_ABS_ALT = 584
CMAC_HEADING = 353


def ros_quat_to_heading(quat):
    # By default, scipy follows scalar-last order – (x, y, z, w)
    rot = R.from_quat([quat.x, quat.y, quat.z, quat.w])
    r, p, y = rot.as_euler(seq="xyz", degrees=True)
    return y


def validate_position_cmac(position):
    """Returns true if the vehicle is at CMAC"""
    LAT_LON_TOL = 1e-5
    return (
        math.isclose(position.latitude, CMAC_LAT, abs_tol=LAT_LON_TOL)
        and math.isclose(position.longitude, CMAC_LON, abs_tol=LAT_LON_TOL)
        and math.isclose(position.altitude, CMAC_ABS_ALT, abs_tol=1.0)
    )


def validate_heading_cmac(orientation):
    """Returns true if the vehicle is facing the right way for CMAC"""
    return math.isclose(ros_quat_to_heading(orientation), CMAC_HEADING)


class GeoPoseListener(rclpy.node.Node):
    """Subscribe to GeoPoseStamped messages"""

    def __init__(self):
        """Initialise the node."""
        super().__init__("geopose_listener")
        self.msg_event_object = threading.Event()
        self.position_correct_event_object = threading.Event()

        # Declare and acquire `topic` parameter
        self.declare_parameter("topic", TOPIC)
        self.topic = self.get_parameter("topic").get_parameter_value().string_value

    def start_subscriber(self):
        """Start the subscriber."""
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(GeoPoseStamped, self.topic, self.subscriber_callback, qos_profile)

        # Add a spin thread.
        self.ros_spin_thread = threading.Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def subscriber_callback(self, msg):
        """Process a GeoPoseStamped message."""
        self.msg_event_object.set()

        position = msg.pose.position

        self.get_logger().info("From AP : Position [lat:{}, lon: {}]".format(position.latitude, position.longitude))

        if validate_position_cmac(msg.pose.position):
            self.position_correct_event_object.set()


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


@pytest.mark.launch(fixture=launch_sitl_copter_dds_serial)
def test_dds_serial_geopose_msg_recv(launch_context, launch_sitl_copter_dds_serial):
    """Test position messages are published by AP_DDS."""
    _, actions = launch_sitl_copter_dds_serial
    virtual_ports = actions["virtual_ports"].action
    micro_ros_agent = actions["micro_ros_agent"].action
    mavproxy = actions["mavproxy"].action
    sitl = actions["sitl"].action

    # Wait for process to start.
    process_tools.wait_for_start_sync(launch_context, virtual_ports, timeout=2)
    process_tools.wait_for_start_sync(launch_context, micro_ros_agent, timeout=2)
    process_tools.wait_for_start_sync(launch_context, mavproxy, timeout=2)
    process_tools.wait_for_start_sync(launch_context, sitl, timeout=2)

    rclpy.init()
    try:
        node = GeoPoseListener()
        node.start_subscriber()
        msgs_received_flag = node.msg_event_object.wait(timeout=10.0)
        assert msgs_received_flag, f"Did not receive '{TOPIC}' msgs."
        pose_correct_flag = node.position_correct_event_object.wait(timeout=10.0)
        assert pose_correct_flag, f"Did not receive correct position."
    finally:
        rclpy.shutdown()
    yield


@pytest.mark.launch(fixture=launch_sitl_copter_dds_udp)
def test_dds_udp_geopose_msg_recv(launch_context, launch_sitl_copter_dds_udp):
    """Test position messages are published by AP_DDS."""
    _, actions = launch_sitl_copter_dds_udp
    micro_ros_agent = actions["micro_ros_agent"].action
    mavproxy = actions["mavproxy"].action
    sitl = actions["sitl"].action

    # Wait for process to start.
    process_tools.wait_for_start_sync(launch_context, micro_ros_agent, timeout=2)
    process_tools.wait_for_start_sync(launch_context, mavproxy, timeout=2)
    process_tools.wait_for_start_sync(launch_context, sitl, timeout=2)

    rclpy.init()
    try:
        node = GeoPoseListener()
        node.start_subscriber()
        msgs_received_flag = node.msg_event_object.wait(timeout=10.0)
        assert msgs_received_flag, f"Did not receive '{TOPIC}' msgs."
        pose_correct_flag = node.position_correct_event_object.wait(timeout=10.0)
        assert pose_correct_flag, f"Did not receive correct position."
    finally:
        rclpy.shutdown()
    yield
