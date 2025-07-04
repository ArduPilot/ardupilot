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
Bring up ArduPilot SITL and check the Joy message is being published.

Arms the copter and commands a throttle up. Check that the vehicle climbs.

colcon test --packages-select ardupilot_dds_tests \
--event-handlers=console_cohesion+ --pytest-args -k test_joy_msg_received

"""

import rclpy
import time
from rclpy.node import Node
from builtin_interfaces.msg import Time
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import TwistStamped
from geopy import point
from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import ModeSwitch
from sensor_msgs.msg import Joy
from scipy.spatial.transform import Rotation as R
import pytest
from launch_pytest.tools import process as process_tools
import threading

from launch_fixtures import launch_sitl_copter_dds_udp

WAIT_FOR_START_TIMEOUT = 5.0

GUIDED = 4
LOITER = 5

FRAME_GLOBAL_INT = 5

# Hard code some known locations
# Note - Altitude in geopy is in km!
GRAYHOUND_TRACK = point.Point(latitude=-35.345996, longitude=149.159017, altitude=0.575)
CMAC = point.Point(latitude=-35.3627010, longitude=149.1651513, altitude=0.585)


class PlaneFbwbJoyControl(Node):
    """Plane follow waypoints using guided control."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("copter_joy_listener")

        self.arming_event_object = threading.Event()
        self.mode_event_object = threading.Event()
        self.climbing_event_object = threading.Event()

        self._client_arm = self.create_client(ArmMotors, "/ap/arm_motors")

        self._client_mode_switch = self.create_client(ModeSwitch, "/ap/mode_switch")

        self._joy_pub = self.create_publisher(Joy, "/ap/joy", 1)

        # Create subscriptions after services are up
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, durability=rclpy.qos.DurabilityPolicy.VOLATILE, depth=1
        )
        self._subscription_geopose = self.create_subscription(
            GeoPoseStamped, "/ap/geopose/filtered", self.geopose_cb, qos
        )
        self._cur_geopose = GeoPoseStamped()

        self._subscription_twist = self.create_subscription(TwistStamped, "/ap/twist/filtered", self.twist_cb, qos)
        self._climb_rate = 0.0

        # Add a spin thread.
        self.ros_spin_thread = threading.Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def geopose_cb(self, msg: GeoPoseStamped):
        """Process a GeoPose message."""
        stamp = msg.header.stamp
        if stamp.sec:
            # Store current state
            self._cur_geopose = msg

    def twist_cb(self, msg: TwistStamped):
        """Process a Twist message."""
        linear = msg.twist.linear
        self._climb_rate = linear.z

    def arm(self):
        req = ArmMotors.Request()
        req.arm = True
        future = self._client_arm.call_async(req)
        time.sleep(0.2)
        try:
            return future.result().result
        except:
            return False

    def arm_with_timeout(self, timeout: rclpy.duration.Duration):
        """Try to arm. Returns true on success, or false if arming fails or times out."""
        armed = False
        start = self.get_clock().now()
        while not armed and self.get_clock().now() - start < timeout:
            armed = self.arm()
            time.sleep(1)
        return armed

    def switch_mode(self, mode):
        req = ModeSwitch.Request()
        req.mode = mode
        future = self._client_mode_switch.call_async(req)
        time.sleep(1)
        try:
            return future.result().status and future.result().curr_mode == mode
        except:
            return False

    def switch_mode_with_timeout(self, desired_mode: int, timeout: rclpy.duration.Duration):
        """Try to switch mode. Returns true on success, or false if mode switch fails or times out."""
        is_in_desired_mode = False
        start = self.get_clock().now()
        while not is_in_desired_mode:
            is_in_desired_mode = self.switch_mode(desired_mode)
            time.sleep(1)

        return is_in_desired_mode

    def get_cur_geopose(self):
        """Return latest geopose."""
        return self._cur_geopose

    def send_joy_value(self, joy_val):
        self._joy_pub.publish(joy_val)

    # -------------- PROCESSES -----------------
    def process_arm(self):
        if self.arm_with_timeout(rclpy.duration.Duration(seconds=30)):
            self.arming_event_object.set()

    def start_arm(self):
        self.arm_thread = threading.Thread(target=self.process_arm)
        self.arm_thread.start()

    def process_climb(self):
        joy_msg = Joy()
        joy_msg.axes.append(0.0)
        joy_msg.axes.append(0.0)
        joy_msg.axes.append(0.8)  # - straight up
        joy_msg.axes.append(0.0)

        while True:
            self.send_joy_value(joy_msg)
            self.arm()  # - Keep the system armed (sometimes it disarms and the test fails)
            time.sleep(0.1)
            self.get_logger().info("From AP : Climb rate {}".format(self._climb_rate))
            if self._climb_rate > 0.5:
                self.climbing_event_object.set()
                return

    def climb(self):
        self.climb_thread = threading.Thread(target=self.process_climb)
        self.climb_thread.start()

    def arm_and_takeoff_process(self):
        self.process_arm()
        self.climb()

    def arm_and_takeoff(self):
        self.climb_thread = threading.Thread(target=self.arm_and_takeoff_process)
        self.climb_thread.start()


@pytest.mark.launch(fixture=launch_sitl_copter_dds_udp)
def test_dds_udp_joy_msg_recv(launch_context, launch_sitl_copter_dds_udp):
    """Test joy messages are published by AP_DDS."""
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
        node = PlaneFbwbJoyControl()
        node.arm_and_takeoff()
        climb_flag = node.climbing_event_object.wait(10)
        assert climb_flag, "Could not climb"
    finally:
        rclpy.shutdown()
    yield
