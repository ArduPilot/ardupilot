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
Launch a non-interactive instance of MAVProxy.

Run with default arguments:

ros2 launch ardupilot_sitl mavproxy.launch.py

Show launch arguments:

ros2 launch ardupilot_sitl mavproxy.launch.py --show-args
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_mavproxy(context, *args, **kwargs):
    """Return a non-interactive MAVProxy process."""
    # Declare the command.
    command = "mavproxy.py"

    # Retrieve launch arguments.
    master = LaunchConfiguration("master").perform(context)
    # out = LaunchConfiguration("out").perform(context)
    sitl = LaunchConfiguration("sitl").perform(context)

    # Display launch arguments.
    print(f"command:          {command}")
    print(f"master:           {master}")
    print(f"sitl:             {sitl}")

    # Create action.
    mavproxy_process = ExecuteProcess(
        cmd=[
            [
                f"{command} ",
                "--out ",
                "127.0.0.1:14550 ",
                "--out ",
                "127.0.0.1:14551 ",
                f"--master {master} ",
                f"--sitl {sitl} ",
                "--non-interactive ",
                # "--daemon "
            ]
        ],
        shell=True,
        output="both",
        respawn=False,
    )

    launch_processes = [mavproxy_process]
    return launch_processes


def generate_launch_description():
    """Generate a launch description for MAVProxy."""
    # Create launch description.
    return LaunchDescription(
        [
            # Launch arguments.
            DeclareLaunchArgument(
                "master",
                default_value="tcp:127.0.0.1:5760",
                description="MAVLink master port and optional baud rate.",
            ),
            DeclareLaunchArgument(
                "out",
                default_value="127.0.0.1:14550",
                description="MAVLink output port and optional baud rate.",
            ),
            DeclareLaunchArgument(
                "sitl",
                default_value="127.0.0.1:5501",
                description="SITL output port.",
            ),
            # Launch function.
            OpaqueFunction(function=launch_mavproxy),
        ]
    )
