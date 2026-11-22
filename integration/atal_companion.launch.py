"""Proposal-only ROS 2 gate companion for ArduSub.

Requires auv_control_ros2 and auv_interfaces_ros2 from the ROS 2 checkout.
No sensor driver, controller, mixer, motor bridge, or MAVLink command is started.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="auv_control_ros2", executable="gate_link",
             name="gate_link", output="screen", parameters=[{
                 "timeout_sec": 0.3,
                 "max_stamp_age_sec": 0.3,
                 "max_future_sec": 0.1,
                 "min_confidence": 0.5,
             }]),
        Node(package="auv_control_ros2", executable="gate_guidance",
             name="gate_guidance", output="screen", parameters=[{
                 "min_confidence": 0.6,
                 "detection_timeout_sec": 0.3,
                 "center_tolerance": 0.08,
                 "align_tolerance": 0.10,
                 "max_forward": 0.15,
                 "max_lateral": 0.10,
                 "max_vertical": 0.10,
                 "max_yaw": 0.20,
                 "approach_min_height": 0.45,
             }]),
    ])
