# Copyright 2026 ArduPilot.org.
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
Bring up ArduPilot SITL Sub and check it moves in response to a cmd_vel command.

colcon test --executor sequential --parallel-workers 0 \
    --packages-select ardupilot_dds_tests \
    --event-handlers=console_cohesion+ --pytest-args -k test_sub_velocity
"""

import threading
import time

import pytest
import rclpy
import rclpy.node

from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import ModeSwitch
from geometry_msgs.msg import TwistStamped
from launch_fixtures import launch_sitl_sub_dds_udp
from launch_pytest.tools import process as process_tools

WAIT_FOR_START_TIMEOUT = 5.0
ARM_TIMEOUT = 30.0
MODE_SWITCH_TIMEOUT = 20.0
VELOCITY_RECV_TIMEOUT = 30.0

SUB_MODE_GUIDED = 4

# /ap/cmd_vel and /ap/twist/filtered use the "map" frame_id ENU convention:
# X - East, Y - North, Z - Up.
NORTH_SPEED_MS = 1.0
NORTH_SPEED_TOLERANCE_MS = 0.2


class SubVelocityTester(rclpy.node.Node):
    """Arm Sub, switch to guided, command a velocity and watch it take effect."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("sub_velocity_tester")
        self.velocity_achieved_event = threading.Event()

        self._client_arm = self.create_client(ArmMotors, "/ap/arm_motors")
        self._client_mode_switch = self.create_client(ModeSwitch, "/ap/mode_switch")
        self._cmd_vel_pub = self.create_publisher(TwistStamped, "/ap/cmd_vel", 1)

        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self._subscription_twist = self.create_subscription(TwistStamped, "/ap/twist/filtered", self.twist_cb, qos)

    def twist_cb(self, msg: TwistStamped):
        """Process a Twist message, looking for North speed close to the commanded value."""
        north_speed = msg.twist.linear.y
        self.get_logger().info(f"From AP : twist/filtered North speed {north_speed}")
        if abs(north_speed - NORTH_SPEED_MS) < NORTH_SPEED_TOLERANCE_MS:
            self.velocity_achieved_event.set()

    def _wait_for_future(self, future, timeout_sec):
        """Wait for a future without spinning -- a background thread already spins this node."""
        deadline = time.time() + timeout_sec
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        return future.result() if future.done() else None

    def arm(self):
        """Call the arm service once."""
        req = ArmMotors.Request()
        req.arm = True
        future = self._client_arm.call_async(req)
        result = self._wait_for_future(future, timeout_sec=5)
        return result is not None and result.result

    def arm_with_timeout(self, timeout_sec):
        """Try to arm. Returns true on success, or false if arming fails or times out."""
        armed = False
        start = time.time()
        while not armed and time.time() - start < timeout_sec:
            armed = self.arm()
            time.sleep(1)
        return armed

    def switch_mode(self, mode):
        """Call the mode switch service once."""
        req = ModeSwitch.Request()
        req.mode = mode
        future = self._client_mode_switch.call_async(req)
        result = self._wait_for_future(future, timeout_sec=5)
        return result is not None and (result.status or result.curr_mode == mode)

    def switch_mode_with_timeout(self, desired_mode, timeout_sec):
        """Try to switch mode. Returns true on success, or false if it fails or times out."""
        is_in_desired_mode = False
        start = time.time()
        while not is_in_desired_mode and time.time() - start < timeout_sec:
            is_in_desired_mode = self.switch_mode(desired_mode)
            time.sleep(1)
        return is_in_desired_mode

    def send_cmd_vel_north(self, speed_ms):
        """Publish an earth-frame (map) North velocity command."""
        msg = TwistStamped()
        msg.header.frame_id = "map"
        msg.twist.linear.y = speed_ms
        self._cmd_vel_pub.publish(msg)


@pytest.mark.launch(fixture=launch_sitl_sub_dds_udp)
def test_dds_udp_sub_velocity(launch_context, launch_sitl_sub_dds_udp):
    """Test that AP_ExternalControl_Sub moves the sub in response to /ap/cmd_vel."""
    _, actions = launch_sitl_sub_dds_udp
    micro_ros_agent = actions["micro_ros_agent"].action
    mavproxy = actions["mavproxy"].action
    sitl = actions["sitl"].action

    # Wait for process to start.
    process_tools.wait_for_start_sync(launch_context, micro_ros_agent, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, mavproxy, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, sitl, timeout=WAIT_FOR_START_TIMEOUT)

    rclpy.init()
    try:
        node = SubVelocityTester()

        # Add a spin thread so the twist subscription is serviced while we arm/switch/send.
        spin_thread = threading.Thread(target=lambda n: rclpy.spin(n), args=(node,))
        spin_thread.start()

        armed = node.arm_with_timeout(ARM_TIMEOUT)
        assert armed, "Unable to arm"

        in_guided = node.switch_mode_with_timeout(SUB_MODE_GUIDED, MODE_SWITCH_TIMEOUT)
        assert in_guided, "Unable to switch to guided mode"

        # Resend well within the 3 simulated-seconds guided velocity timeout
        # (see ArduSub/mode_guided.cpp GUIDED_POSVEL_TIMEOUT_MS). The SITL clock
        # runs at the fixture's speedup factor relative to this wall-clock sleep,
        # so a short sleep here still leaves comfortable margin.
        start_time = time.time()
        while not node.velocity_achieved_event.is_set() and time.time() - start_time < VELOCITY_RECV_TIMEOUT:
            node.send_cmd_vel_north(NORTH_SPEED_MS)
            time.sleep(0.05)

        assert node.velocity_achieved_event.is_set(), "Sub did not reach commanded velocity from /ap/cmd_vel"
    finally:
        rclpy.shutdown()
    yield
