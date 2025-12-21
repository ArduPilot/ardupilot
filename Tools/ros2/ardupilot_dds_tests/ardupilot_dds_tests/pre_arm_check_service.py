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

"""
Run pre_arm check test on Copter.

Warning - This is NOT production code; it's a simple demo of capability.
"""

import rclpy
import time

from rclpy.node import Node
from std_srvs.srv import Trigger


class CopterPreArm(Node):
    """Check Prearm Service."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("copter_prearm")

        self.declare_parameter("pre_arm_service", "/ap/prearm_check")
        self._prearm_service = self.get_parameter("pre_arm_service").get_parameter_value().string_value
        self._client_prearm = self.create_client(Trigger, self._prearm_service)
        while not self._client_prearm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('prearm service not available, waiting again...')

    def prearm(self):
        req = Trigger.Request()
        future = self._client_prearm.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def prearm_with_timeout(self, timeout: rclpy.duration.Duration):
        """Check if armable. Returns true on success, or false if arming will fail or times out."""
        armable = False
        start = self.get_clock().now()
        while not armable and self.get_clock().now() - start < timeout:
            armable = self.prearm().success
            time.sleep(1)
        return armable


def main(args=None):
    """Node entry point."""
    rclpy.init(args=args)
    node = CopterPreArm()
    try:
        # Block till armed, which will wait for EKF3 to initialize
        if not node.prearm_with_timeout(rclpy.duration.Duration(seconds=30)):
            raise RuntimeError("Vehicle not armable")
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
