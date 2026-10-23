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

# flake8: noqa

"""
Run a single-waypoint mission on Plane.

Warning - This is NOT production code; it's a simple demo of capability.
"""

import math
import rclpy
import time

from rclpy.node import Node
from ardupilot_msgs.msg import GlobalPosition
from geographic_msgs.msg import GeoPoseStamped
from geopy import distance
from geopy import point
from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import ModeSwitch
from geographic_msgs.msg import GeoPointStamped


PLANE_MODE_TAKEOFF = 13
PLANE_MODE_GUIDED = 15

FRAME_GLOBAL_INT = 5

# Hard code some known locations
# Note - Altitude in geopy is in km!
GRAYHOUND_TRACK = point.Point(latitude=-35.345996, longitude=149.159017, altitude=0.575)
CMAC = point.Point(latitude=-35.3627010, longitude=149.1651513, altitude=0.585)


class PlaneWaypointFollower(Node):
    """Plane follow waypoints using guided control."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("plane_waypoint_follower")

        self.declare_parameter("arm_topic", "/ap/arm_motors")
        self._arm_topic = self.get_parameter("arm_topic").get_parameter_value().string_value
        self._client_arm = self.create_client(ArmMotors, self._arm_topic)
        while not self._client_arm.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('arm service not available, waiting again...')

        self.declare_parameter("mode_topic", "/ap/mode_switch")
        self._mode_topic = self.get_parameter("mode_topic").get_parameter_value().string_value
        self._client_mode_switch = self.create_client(ModeSwitch, self._mode_topic)
        while not self._client_mode_switch.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('mode switch service not available, waiting again...')

        self.declare_parameter("global_position_topic", "/ap/cmd_gps_pose")
        self._global_pos_topic = self.get_parameter("global_position_topic").get_parameter_value().string_value
        self._global_pos_pub = self.create_publisher(GlobalPosition, self._global_pos_topic, 1)

        # Create subscriptions after services are up
        self.declare_parameter("geopose_topic", "/ap/geopose/filtered")
        self._geopose_topic = self.get_parameter("geopose_topic").get_parameter_value().string_value
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, durability=rclpy.qos.DurabilityPolicy.VOLATILE, depth=1
        )

        self._subscription_geopose = self.create_subscription(GeoPoseStamped, self._geopose_topic, self.geopose_cb, qos)
        self._cur_geopose = GeoPoseStamped()

        self.declare_parameter("goal_topic", "/ap/goal_lla")
        self._goal_topic = self.get_parameter("goal_topic").get_parameter_value().string_value
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        self._subscription_goal = self.create_subscription(GeoPointStamped, self._goal_topic, self.goal_cb, qos)
        self._cur_goal = GeoPointStamped()

    def geopose_cb(self, msg: GeoPoseStamped):
        """Process a GeoPose message."""
        stamp = msg.header.stamp
        if stamp.sec:
            self.get_logger().info("From AP : Geopose [sec:{}, nsec: {}]".format(stamp.sec, stamp.nanosec))

            # Store current state
            self._cur_geopose = msg

    def goal_cb(self, msg: GeoPointStamped):
        """Process a Goal message."""
        stamp = msg.header.stamp
        self.get_logger().info(
            "From AP : Goal [sec:{}, nsec: {}, lat:{} lon:{}]".format(
                stamp.sec, stamp.nanosec, msg.position.latitude, msg.position.longitude
            )
        )

        # Store current state
        self._cur_goal = msg

    def arm(self):
        req = ArmMotors.Request()
        req.arm = True
        future = self._client_arm.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def arm_with_timeout(self, timeout: rclpy.duration.Duration):
        """Try to arm. Returns true on success, or false if arming fails or times out."""
        armed = False
        start = self.get_clock().now()
        while not armed and self.get_clock().now() - start < timeout:
            armed = self.arm().result
            time.sleep(1)
        return armed

    def switch_mode(self, mode):
        req = ModeSwitch.Request()
        assert mode in [PLANE_MODE_TAKEOFF, PLANE_MODE_GUIDED]
        req.mode = mode
        future = self._client_mode_switch.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def switch_mode_with_timeout(self, desired_mode: int, timeout: rclpy.duration.Duration):
        """Try to switch mode. Returns true on success, or false if mode switch fails or times out."""
        is_in_desired_mode = False
        start = self.get_clock().now()
        while not is_in_desired_mode:
            result = self.switch_mode(desired_mode)
            # Handle successful switch or the case that the vehicle is already in expected mode
            is_in_desired_mode = result.status or result.curr_mode == desired_mode
            time.sleep(1)

        return is_in_desired_mode

    def get_cur_geopose(self):
        """Return latest geopose."""
        return self._cur_geopose

    def get_cur_goal(self):
        """Return latest goal."""
        return self._cur_goal

    def send_goal_position(self, goal_global_pos):
        """Send goal position. Must be in guided for this to work."""
        self._global_pos_pub.publish(goal_global_pos)


def achieved_goal(goal_global_pos, cur_geopose):
    """Return true if the current position (LLH) is close enough to the goal (within the orbit radius)."""
    # Use 3D geopy distance calculation
    # https://geopy.readthedocs.io/en/stable/#module-geopy.distance
    goal_lat = goal_global_pos

    p1 = (goal_global_pos.latitude, goal_global_pos.longitude, goal_global_pos.altitude)
    cur_pos = cur_geopose.pose.position
    p2 = (cur_pos.latitude, cur_pos.longitude, cur_pos.altitude)

    flat_distance = distance.distance(p1[:2], p2[:2]).m
    euclidian_distance = math.sqrt(flat_distance**2 + (p2[2] - p1[2]) ** 2)
    print(f"Goal is {euclidian_distance} meters away")
    return euclidian_distance < 150


def going_to_goal(goal_global_pos, cur_goal):
    p1 = (goal_global_pos.latitude, goal_global_pos.longitude, goal_global_pos.altitude)
    cur_pos_lla = cur_goal.position
    p2 = (cur_pos_lla.latitude, cur_pos_lla.longitude, cur_pos_lla.altitude)

    flat_distance = distance.distance(p1[:2], p2[:2]).m
    euclidian_distance = math.sqrt(flat_distance**2 + (p2[2] - p1[2]) ** 2)
    print(f"Commanded and received goal are {euclidian_distance} meters away")
    return euclidian_distance < 1


def main(args=None):
    """Node entry point."""
    rclpy.init(args=args)
    node = PlaneWaypointFollower()
    try:
        # Block till armed, which will wait for EKF3 to initialize
        if not node.arm_with_timeout(rclpy.duration.Duration(seconds=30)):
            raise RuntimeError("Unable to arm")

        # Block till in takeoff
        if not node.switch_mode_with_timeout(PLANE_MODE_TAKEOFF, rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to switch to takeoff mode")

        is_ascending_to_takeoff_alt = True
        while is_ascending_to_takeoff_alt:
            rclpy.spin_once(node)
            time.sleep(1.0)

            # Hard code waiting in takeoff to reach operating altitude of 630m
            # This is just a hack because geopose is reported with absolute rather than relative altitude,
            # and this node doesn't have access to the terrain data
            is_ascending_to_takeoff_alt = node.get_cur_geopose().pose.position.altitude < CMAC.altitude * 1000 + 45

        if is_ascending_to_takeoff_alt:
            raise RuntimeError("Failed to reach takeoff altitude")

        if not node.switch_mode_with_timeout(PLANE_MODE_GUIDED, rclpy.duration.Duration(seconds=20)):
            raise RuntimeError("Unable to switch to guided mode")

        # Send a guided WP with location, frame ID, alt frame
        goal_pos = GlobalPosition()
        goal_pos.latitude = GRAYHOUND_TRACK.latitude
        goal_pos.longitude = GRAYHOUND_TRACK.longitude
        DESIRED_AGL = 60
        goal_pos.altitude = GRAYHOUND_TRACK.altitude * 1000 + DESIRED_AGL
        goal_pos.coordinate_frame = FRAME_GLOBAL_INT
        goal_pos.header.frame_id = "map"

        node.send_goal_position(goal_pos)

        start = node.get_clock().now()
        has_achieved_goal = False
        is_going_to_goal = False
        while not has_achieved_goal and node.get_clock().now() - start < rclpy.duration.Duration(seconds=120):
            rclpy.spin_once(node)
            is_going_to_goal = going_to_goal(goal_pos, node.get_cur_goal())
            has_achieved_goal = achieved_goal(goal_pos, node.get_cur_geopose())
            time.sleep(1.0)

        if not is_going_to_goal:
            raise RuntimeError("Unable to go to goal location")
        if not has_achieved_goal:
            raise RuntimeError("Unable to achieve goal location")

        print("Goal achieved")

    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
