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
    verbose = LaunchConfiguration("verbose").perform(context)
    discovery = LaunchConfiguration("discovery").perform(context)

    # IPvX arguments.
    port = LaunchConfiguration("port").perform(context)

    # Serial arguments.
    device = LaunchConfiguration("device").perform(context)
    baudrate = LaunchConfiguration("baudrate").perform(context)

    # Display launch arguments.
    print(f"namespace:        {micro_ros_agent_ns}")
    print(f"transport:        {transport}")
    print(f"middleware:       {middleware}")
    print(f"verbose:          {verbose}")
    print(f"discovery:        {discovery}")

    # Required arguments
    args = [
        transport,
        "--middleware",
        middleware,
    ]

    if transport in ["udp4", "udp6", "tcp4", "tcp6"]:
        # IPvX arguments
        port = LaunchConfiguration("port").perform(context)
        args.append("--port")
        args.append(port)
        print(f"port:             {port}")
    elif transport in ["serial", "multiserial", "pseudoterminal"]:
        # Serial arguments
        baudrate = LaunchConfiguration("baudrate").perform(context)
        args.append("--baudrate")
        args.append(baudrate)
        print(f"baudrate:         {baudrate}")

        device = LaunchConfiguration("device").perform(context)
        args.append("--device")
        args.append(device)
        print(f"device:           {device}")
    else:
        # transport must be canfd
        pass

    # Optional arguments.
    refs = LaunchConfiguration("refs").perform(context)
    if refs:
        args.append("--refs")
        args.append(refs)
        print(f"refs:             {refs}")

    # Create action.
    micro_ros_agent_node = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        namespace=f"{micro_ros_agent_ns}",
        output="both",
        arguments=args,
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
            default_value="udp4",
            description="Set the transport.",
            choices=[
                "udp4",
                "udp6",
                "tcp4",
                "tcp6",
                "canfd",
                "serial",
                "multiserial",
                "pseudoterminal",
            ],
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
            default_value="4",
            description="Set the verbosity level.",
            choices=["0", "1", "2", "3", "4", "5", "6"],
        ),
        DeclareLaunchArgument(
            "discovery",
            default_value="7400",
            description="Set the dsicovery port.",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="2019",
            description="Set the port number.",
        ),
        DeclareLaunchArgument(
            "baudrate",
            default_value="115200",
            description="Set the baudrate.",
        ),
        DeclareLaunchArgument(
            "device",
            default_value="",
            description="Set the device.",
        ),
    ]
