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

# flake8: noqa

"""
Bring up ArduPilot SITL and check the GeoPoseStamped message is being published.

colcon test --packages-select ardupilot_dds_tests \
--event-handlers=console_cohesion+ --pytest-args -k test_geopose_msg_received

"""

import math
import pytest
import rclpy
import rclpy.node
from scipy.spatial.transform import Rotation as R
import threading

from launch_pytest.tools import process as process_tools

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy

from geographic_msgs.msg import GeoPoseStamped

from launch_fixtures import (
    launch_sitl_copter_dds_serial,
    launch_sitl_copter_dds_udp,
)

TOPIC = "ap/geopose/filtered"
WAIT_FOR_START_TIMEOUT = 5.0
# Copied from locations.txt
CMAC_LAT = -35.363261
CMAC_LON = 149.165230
CMAC_ABS_ALT = 584
CMAC_HEADING = 353


def ros_quat_to_heading_deg(quat):
    # By default, scipy follows scalar-last order – (x, y, z, w)
    rot = R.from_quat([quat.x, quat.y, quat.z, quat.w])
    r, p, y = rot.as_euler(seq="xyz", degrees=True)
    return y


def validate_position_cmac(position):
    """Return true if the vehicle is at CMAC."""
    LAT_LON_TOL = 1e-5
    return (
        math.isclose(position.latitude, CMAC_LAT, abs_tol=LAT_LON_TOL)
        and math.isclose(position.longitude, CMAC_LON, abs_tol=LAT_LON_TOL)
        and math.isclose(position.altitude, CMAC_ABS_ALT, abs_tol=1.0)
    )


def wrap_360(angle):
    if angle > 360:
        angle -= 360.0
        return wrap_360(angle)
    if angle < 0:
        angle += 360.0
        return wrap_360(angle)
    return angle


def validate_heading_cmac(orientation):
    """
    Return true if the vehicle is facing the right way for CMAC.

    Per ROS REP-103, the quaternion should report 0 when the vehicle faces east,
    and 90 degrees when facing north.
    Because CMAC is NNW, we expect ~97 degees.
    See this PR for more info:
    * https://github.com/ros-infrastructure/rep/pull/407
    """
    # The following converts from compass bearing (locations.txt) to REP-103 heading.
    CMAC_YAW_ENU_REP103 = wrap_360(-1 * CMAC_HEADING) + 90
    return math.isclose(ros_quat_to_heading_deg(orientation), CMAC_YAW_ENU_REP103, abs_tol=5)


class GeoPoseListener(rclpy.node.Node):
    """Subscribe to GeoPoseStamped messages."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("geopose_listener")
        self.msg_event_object = threading.Event()
        self.position_correct_event_object = threading.Event()
        self.orientation_event_object = threading.Event()

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
        if self.msg_event_object.set():
            return

        position = msg.pose.position

        self.get_logger().info("From AP : Position [lat:{}, lon: {}]".format(position.latitude, position.longitude))

        if validate_position_cmac(msg.pose.position):
            self.position_correct_event_object.set()

        if validate_heading_cmac(msg.pose.orientation):
            self.orientation_event_object.set()

        # set event last
        self.msg_event_object.set()


@pytest.mark.launch(fixture=launch_sitl_copter_dds_serial)
def test_dds_serial_geopose_msg_recv(launch_context, launch_sitl_copter_dds_serial):
    """Test position messages are published by AP_DDS."""
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
        node = GeoPoseListener()
        node.start_subscriber()
        msgs_received_flag = node.msg_event_object.wait(timeout=10.0)
        assert msgs_received_flag, f"Did not receive '{TOPIC}' msgs."
        pose_correct_flag = node.position_correct_event_object.wait(timeout=10.0)
        assert pose_correct_flag, f"Did not receive correct position."
        orientation_correct_flag = node.orientation_event_object.wait(timeout=10.0)
        assert orientation_correct_flag, f"Did not receive correct orientation."
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
    process_tools.wait_for_start_sync(launch_context, micro_ros_agent, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, mavproxy, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, sitl, timeout=WAIT_FOR_START_TIMEOUT)

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
