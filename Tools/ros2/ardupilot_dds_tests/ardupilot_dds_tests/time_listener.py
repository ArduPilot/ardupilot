#!/usr/bin/env python3
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

"""Subscribe to Time messages on topic /ap/time."""
import rclpy
import time

from rclpy.node import Node
from builtin_interfaces.msg import Time


class TimeListener(Node):
    """Subscribe to Time messages on topic /ap/time."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("time_listener")

        # Declare and acquire `topic` parameter.
        self.declare_parameter("topic", "ap/time")
        self.topic = self.get_parameter("topic").get_parameter_value().string_value

        # Subscriptions.
        self.subscription = self.create_subscription(Time, self.topic, self.cb, 1)
        self.subscription

    def cb(self, msg):
        """Process a Time message."""
        if msg.sec:
            self.get_logger().info("From AP : True [sec:{}, nsec: {}]".format(msg.sec, msg.nanosec))
        else:
            self.get_logger().info("From AP : False")


def main(args=None):
    """Node entry point."""
    rclpy.init(args=args)
    node = TimeListener()
    try:
        count = 0
        while count < 100:
            rclpy.spin_once(node)
            count += 1
            time.sleep(1.0)

    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
