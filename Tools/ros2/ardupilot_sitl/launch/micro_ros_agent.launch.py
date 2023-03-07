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
Launch the microROS DDS agent.

Run with default arguments:

ros2 launch ardupilot_sitl micro_ros_agent.launch.py
"""
from typing import List

from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_micro_ros_agent(
    context: LaunchContext, *args, **kwargs
) -> List[LaunchDescriptionEntity]:
    """Launch the micro_ros_agent node."""
    # ROS arguments.
    micro_ros_agent_ns = LaunchConfiguration("micro_ros_agent_ns").perform(context)

    # Common arguments.
    transport = LaunchConfiguration("transport").perform(context)
    middleware = LaunchConfiguration("middleware").perform(context)
    refs = LaunchConfiguration("refs").perform(context)
    verbose = LaunchConfiguration("verbose").perform(context)
    # discovery = LaunchConfiguration("discovery").perform(context)

    # Serial arguments.
    device = LaunchConfiguration("device").perform(context)
    baudrate = LaunchConfiguration("baudrate").perform(context)

    # Display launch arguments.
    print(f"namespace:        {micro_ros_agent_ns}")
    print(f"transport:        {transport}")
    print(f"middleware:       {middleware}")
    print(f"refs:             {refs}")
    print(f"verbose:          {verbose}")
    # print(f"discovery:        {discovery}")

    print(f"baudrate:         {baudrate}")
    print(f"device:           {device}")

    # Create action.
    micro_ros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        namespace=f"{micro_ros_agent_ns}",
        output="both",
        arguments=[
            {transport},
            "--middleware",
            {middleware},
            "--refs",
            {refs},
            "--dev",
            {device},
            "--baudrate",
            {baudrate},
        ],
    )
    return [micro_ros_agent_node]


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description for the micro_ros_agent."""
    launch_arguments = generate_launch_arguments()

    return LaunchDescription(
        launch_arguments
        + [
            OpaqueFunction(function=launch_micro_ros_agent),
        ]
    )


def generate_launch_arguments() -> List[DeclareLaunchArgument]:
    """Generate a list of launch arguments."""
    return [
        DeclareLaunchArgument(
            "micro_ros_agent_ns",
            default_value="",
            description="Set the micro_ros_agent namespace.",
        ),
        DeclareLaunchArgument(
            "transport",
            default_value="serial",
            description="Set the transport.",
            choices=["serial"],
        ),
        DeclareLaunchArgument(
            "middleware",
            default_value="dds",
            description="Set the middleware.",
            choices=["ced", "dds", "rtps"],
        ),
        DeclareLaunchArgument(
            "refs",
            default_value="",
            description="Set the refs file.",
        ),
        DeclareLaunchArgument(
            "verbose",
            default_value="",
            description="Set the verbosity level.",
        ),
        DeclareLaunchArgument(
            "baudrate",
            default_value="",
            description="Set the baudrate.",
        ),
        DeclareLaunchArgument(
            "device",
            default_value="",
            description="Set the device.",
        ),
    ]
