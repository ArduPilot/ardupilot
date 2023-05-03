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

    # Create directory for virtual ports.
    print(f"\ntmpdirname: {device_dir}\n")
    os.mkdir(Path(path, "dev"))

    yield path


@pytest.fixture
def virtual_ports(device_dir):
    """Fixture to create virtual ports."""
    tty0 = Path(device_dir, "dev", "tty0").resolve()
    tty1 = Path(device_dir, "dev", "tty1").resolve()

    yield IncludeLaunchDescription(
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


@pytest.fixture
def micro_ros_agent_serial(device_dir):
    """Fixture to create a micro_ros_agent node."""
    tty0 = Path(device_dir, "dev", "tty0").resolve()

    yield IncludeLaunchDescription(
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
            "transport": "serial",
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


@pytest.fixture
def micro_ros_agent_udp():
    """Fixture to create a micro_ros_agent node."""
    yield IncludeLaunchDescription(
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
            "transport": "udp4",
            "refs": PathJoinSubstitution(
                [
                    FindPackageShare("ardupilot_sitl"),
                    "config",
                    "dds_xrce_profile.xml",
                ]
            ),
        }.items(),
    )


@pytest.fixture
def mavproxy():
    """Fixture to bring up ArduPilot SITL DDS."""
    yield IncludeLaunchDescription(
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


@pytest.fixture
def sitl_copter_dds_serial(device_dir, virtual_ports, micro_ros_agent_serial, mavproxy):
    """Fixture to bring up ArduPilot SITL DDS."""
    tty1 = Path(device_dir, "dev", "tty1").resolve()

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
            # "wipe": "True",
            "wipe": "False",
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
                    "dds_serial.parm",
                )
            ),
        }.items(),
    )

    yield LaunchDescription(
        [
            virtual_ports,
            micro_ros_agent_serial,
            sitl,
            mavproxy,
        ]
    )


@pytest.fixture
def sitl_copter_dds_udp(device_dir, micro_ros_agent_udp, mavproxy):
    """Fixture to bring up ArduPilot SITL DDS."""
    tty1 = Path(device_dir, "dev", "tty1").resolve()

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
            # "wipe": "True",
            "wipe": "False",
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
                    "dds_udp.parm",
                )
            ),
        }.items(),
    )

    yield LaunchDescription(
        [
            micro_ros_agent_udp,
            sitl,
            mavproxy,
        ]
    )
