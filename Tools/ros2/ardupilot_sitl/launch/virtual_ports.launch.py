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
Launch a process to create virtual ports.

Run with default arguments:

ros2 launch ardupilot_sitl virtual_ports.launch.py
"""
from typing import List

from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_socat(
    context: LaunchContext, *args, **kwargs
) -> List[LaunchDescriptionEntity]:
    """Launch a process to create virtual ports using `socat`."""
    # Declare commands.
    command = "socat"

    # Retrieve launch arguments.
    tty0 = LaunchConfiguration("tty0").perform(context)
    tty1 = LaunchConfiguration("tty1").perform(context)

    # Display launch arguments.
    print(f"command:          {command}")
    print(f"tty0:             {tty0}")
    print(f"tty1:             {tty1}")

    # Create action.
    socat_process = ExecuteProcess(
        cmd=[
            [
                "socat ",
                "-d -d ",
                f"pty,raw,echo=0,link={tty0} ",
                f"pty,raw,echo=0,link={tty1} ",
            ]
        ],
        shell=True,
        output="both",
        respawn=False,
    )
    return [socat_process]


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description for creating virtual ports using `socat`."""
    launch_arguments = generate_launch_arguments()

    return LaunchDescription(
        launch_arguments
        + [
            OpaqueFunction(function=launch_socat),
        ]
    )


def generate_launch_arguments() -> List[DeclareLaunchArgument]:
    """Generate a list of launch arguments."""
    return [
        DeclareLaunchArgument(
            "tty0",
            default_value="tty0",
            description="Symbolic link name for tty0.",
        ),
        DeclareLaunchArgument(
            "tty1",
            default_value="tty1",
            description="Symbolic link name for tty1.",
        ),
    ]
