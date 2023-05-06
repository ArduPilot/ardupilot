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

"""Common fixtures."""
import os
import pytest

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

from pathlib import Path


@pytest.fixture(scope="module")
def device_dir(tmp_path_factory):
    """Fixture to create a temporary directory for devices."""
    path = tmp_path_factory.mktemp("devices")
    return path


@pytest.fixture
def sitl_dds(device_dir):
    """Fixture to bring up ArduPilot SITL DDS."""
    # Create directory for virtual ports.
    print(f"\ntmpdirname: {device_dir}\n")
    if not ("dev" in os.listdir(device_dir)):
        os.mkdir(Path(device_dir, "dev"))

    # Full path to virtual ports.
    tty0 = Path(device_dir, "dev", "tty0").resolve()
    tty1 = Path(device_dir, "dev", "tty1").resolve()

    virtual_ports = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_sitl"),
                        "launch",
                        "virtual_ports.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "tty0": str(tty0),
            "tty1": str(tty1),
        }.items(),
    )

    micro_ros_agent = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_sitl"),
                        "launch",
                        "micro_ros_agent.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "refs": PathJoinSubstitution(
                [
                    FindPackageShare("ardupilot_sitl"),
                    "config",
                    "dds_xrce_profile.xml",
                ]
            ),
            "baudrate": "115200",
            "device": str(tty0),
        }.items(),
    )

    sitl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_sitl"),
                        "launch",
                        "sitl.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "command": "arducopter",
            "synthetic_clock": "True",
            "wipe": "True",
            "model": "quad",
            "speedup": "10",
            "slave": "0",
            "instance": "0",
            "uartC": f"uart:{str(tty1)}",
            "defaults": str(
                Path(
                    get_package_share_directory("ardupilot_sitl"),
                    "config",
                    "default_params",
                    "copter.parm",
                )
            )
            + ","
            + str(
                Path(
                    get_package_share_directory("ardupilot_sitl"),
                    "config",
                    "default_params",
                    "dds.parm",
                )
            ),
        }.items(),
    )

    mavproxy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ardupilot_sitl"),
                        "launch",
                        "mavproxy.launch.py",
                    ]
                ),
            ]
        ),
        launch_arguments={
            "master": "tcp:127.0.0.1:5760",
            "sitl": "127.0.0.1:5501",
        }.items(),
    )

    return LaunchDescription(
        [
            virtual_ports,
            micro_ros_agent,
            sitl,
            mavproxy,
        ]
    )
