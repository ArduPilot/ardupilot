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

"""Bring up ArduPilot SITL and check the NavSat message is being published."""
import launch_pytest
import pytest
import rclpy
import rclpy.node
import threading

from launch import LaunchDescription

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from sensor_msgs.msg import NavSatFix


@launch_pytest.fixture
def launch_description(sitl_dds):
    """Fixture to create the launch description."""
    return LaunchDescription(
        [
            sitl_dds,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


class NavSatFixListener(rclpy.node.Node):
    """Subscribe to NavSatFix messages on /ROS2_NavSatFix0."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("navsatfix_listener")
        self.msg_event_object = threading.Event()

        # Declare and acquire `topic` parameter
        self.declare_parameter("topic", "ROS2_NavSatFix0")
        self.topic = self.get_parameter("topic").get_parameter_value().string_value

    def start_subscriber(self):
        """Start the subscriber."""
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.create_subscription(
            NavSatFix, self.topic, self.subscriber_callback, qos_profile
        )

        # Add a spin thread.
        self.ros_spin_thread = threading.Thread(
            target=lambda node: rclpy.spin(node), args=(self,)
        )
        self.ros_spin_thread.start()

    def subscriber_callback(self, msg):
        """Process a NavSatFix message."""
        self.msg_event_object.set()

        if msg.latitude:
            self.get_logger().info(
                "From AP : True [lat:{}, lon: {}]".format(msg.latitude, msg.longitude)
            )
        else:
            self.get_logger().info("From AP : False")


@pytest.mark.launch(fixture=launch_description)
def test_navsat_msgs_received(sitl_dds, launch_context):
    """Test NavSatFix messages are published by AP_DDS."""
    rclpy.init()
    try:
        node = NavSatFixListener()
        node.start_subscriber()
        msgs_received_flag = node.msg_event_object.wait(timeout=10.0)
        assert msgs_received_flag, "Did not receive 'ROS2_NavSatFix0' msgs."
    finally:
        rclpy.shutdown()

    yield

    # Anything below this line is executed after launch service shutdown.
    pass
