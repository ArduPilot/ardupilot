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

"""AP_DDS with DDS_USE_NS."""

import threading
import time

import pytest
import rclpy
import rclpy.node

from builtin_interfaces.msg import Time
from launch_fixtures import launch_sitl_copter_dds_udp_use_ns
from launch_pytest.tools import process as process_tools
from std_srvs.srv import Trigger

TOPIC = "ap/v1/time"
SERVICE = "/ap/v1/prearm_check"
WAIT_FOR_START_TIMEOUT = 5.0


class TimeListener(rclpy.node.Node):
    """Subscribe to Time messages."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("time_listener")
        self.msg_event_object = threading.Event()

        # Declare and acquire `topic` parameter.
        self.declare_parameter("topic", TOPIC)
        self.topic = self.get_parameter("topic").get_parameter_value().string_value

    def start_subscriber(self):
        """Start the subscriber."""
        self.subscription = self.create_subscription(Time, self.topic, self.subscriber_callback, 1)

        # Add a spin thread.
        self.ros_spin_thread = threading.Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def subscriber_callback(self, msg):
        """Process a Time message."""
        if self.msg_event_object.set():
            return

        if msg.sec:
            self.get_logger().info("From AP : True [sec:{}, nsec: {}]".format(msg.sec, msg.nanosec))
        else:
            self.get_logger().info("From AP : False")

        # set event last
        self.msg_event_object.set()


@pytest.mark.launch(fixture=launch_sitl_copter_dds_udp_use_ns)
def test_dds_udp_time_msg_recv(launch_context, launch_sitl_copter_dds_udp_use_ns):
    """Test Time is published by AP_DDS."""
    _, actions = launch_sitl_copter_dds_udp_use_ns
    micro_ros_agent = actions["micro_ros_agent"].action
    mavproxy = actions["mavproxy"].action
    sitl = actions["sitl"].action

    # Wait for process to start.
    process_tools.wait_for_start_sync(launch_context, micro_ros_agent, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, mavproxy, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, sitl, timeout=WAIT_FOR_START_TIMEOUT)

    rclpy.init()
    try:
        node = TimeListener()
        node.start_subscriber()
        msgs_received_flag = node.msg_event_object.wait(timeout=10.0)
        assert msgs_received_flag, f"Did not receive '{TOPIC}' msgs."
    finally:
        rclpy.shutdown()
    yield


class PreamService(rclpy.node.Node):
    def __init__(self):
        """Initialise the node."""
        super().__init__("prearm_client")
        self.service_available_object = threading.Event()
        self.is_armable_object = threading.Event()
        self._client_prearm = self.create_client(Trigger, SERVICE)

    def start_node(self):
        # Add a spin thread.
        self.ros_spin_thread = threading.Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def prearm_check(self):
        req = Trigger.Request()
        future = self._client_prearm.call_async(req)
        time.sleep(0.2)
        try:
            return future.result().success
        except Exception as e:
            print(e)
            return False

    def prearm_with_timeout(self, timeout: rclpy.duration.Duration):
        result = False
        start = self.get_clock().now()
        while not result and self.get_clock().now() - start < timeout:
            result = self.prearm_check()
            time.sleep(1)
        return result

    def process_prearm(self):
        if self.prearm_with_timeout(rclpy.duration.Duration(seconds=30)):
            self.is_armable_object.set()

    def start_prearm(self):
        try:
            self.prearm_thread.stop()
        except:
            print("start_prearm not started yet")
        self.prearm_thread = threading.Thread(target=self.process_prearm)
        self.prearm_thread.start()


@pytest.mark.launch(fixture=launch_sitl_copter_dds_udp_use_ns)
def test_dds_udp_prearm_service_call(launch_context, launch_sitl_copter_dds_udp_use_ns):
    """Test prearm service AP_DDS."""
    _, actions = launch_sitl_copter_dds_udp_use_ns
    micro_ros_agent = actions["micro_ros_agent"].action
    mavproxy = actions["mavproxy"].action
    sitl = actions["sitl"].action

    # Wait for process to start.
    process_tools.wait_for_start_sync(launch_context, micro_ros_agent, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, mavproxy, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, sitl, timeout=WAIT_FOR_START_TIMEOUT)

    rclpy.init()
    try:
        node = PreamService()
        node.start_node()
        node.start_prearm()
        is_armable_flag = node.is_armable_object.wait(timeout=25.0)
        assert is_armable_flag, f"Vehicle not armable."
    finally:
        rclpy.shutdown()
    yield
