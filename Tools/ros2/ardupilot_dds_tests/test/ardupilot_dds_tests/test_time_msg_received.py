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

"""Bring up ArduPilot SITL check the Time message is being published."""
import launch_pytest
import pytest
import rclpy
import rclpy.node
import threading

from launch import LaunchDescription

from launch_pytest.tools import process as process_tools

from builtin_interfaces.msg import Time


class TimeListener(rclpy.node.Node):
    """Subscribe to Time messages on /ap/time."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("time_listener")
        self.msg_event_object = threading.Event()

        # Declare and acquire `topic` parameter.
        self.declare_parameter("topic", "ap/time")
        self.topic = self.get_parameter("topic").get_parameter_value().string_value

    def start_subscriber(self):
        """Start the subscriber."""
        self.subscription = self.create_subscription(
            Time, self.topic, self.subscriber_callback, 1
        )

        # Add a spin thread.
        self.ros_spin_thread = threading.Thread(
            target=lambda node: rclpy.spin(node), args=(self,)
        )
        self.ros_spin_thread.start()

    def subscriber_callback(self, msg):
        """Process a Time message."""
        self.msg_event_object.set()

        if msg.sec:
            self.get_logger().info(
                "From AP : True [sec:{}, nsec: {}]".format(msg.sec, msg.nanosec)
            )
        else:
            self.get_logger().info("From AP : False")


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
def test_dds_serial_time_msg_recv(launch_context, launch_sitl_copter_dds_serial):
    """Test /ap/time is published by AP_DDS."""
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
        node = TimeListener()
        node.start_subscriber()
        msgs_received_flag = node.msg_event_object.wait(timeout=10.0)
        assert msgs_received_flag, "Did not receive 'ROS_Time' msgs."
    finally:
        rclpy.shutdown()
    yield


@pytest.mark.launch(fixture=launch_sitl_copter_dds_udp)
def test_dds_udp_time_msg_recv(launch_context, launch_sitl_copter_dds_udp):
    """Test /ap/time is published by AP_DDS."""
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
        node = TimeListener()
        node.start_subscriber()
        msgs_received_flag = node.msg_event_object.wait(timeout=10.0)
        assert msgs_received_flag, "Did not receive 'ROS_Time' msgs."
    finally:
        rclpy.shutdown()
    yield
