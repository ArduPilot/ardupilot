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
from launch import LaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare

from pathlib import Path

from ardupilot_sitl.launch import VirtualPortsLaunch
from ardupilot_sitl.launch import MicroRosAgentLaunch
from ardupilot_sitl.launch import MAVProxyLaunch
from ardupilot_sitl.launch import SITLLaunch


@pytest.fixture(scope="module")
def device_dir(tmp_path_factory):
    """Fixture to create a temporary directory for devices."""
    path = tmp_path_factory.mktemp("devices")

    # Create dev directory for virtual ports.
    os.mkdir(Path(path, "dev"))

    yield path


@pytest.fixture
def virtual_ports(device_dir):
    """Fixture to create virtual ports."""
    tty0 = Path(device_dir, "dev", "tty0").resolve()
    tty1 = Path(device_dir, "dev", "tty1").resolve()

    vp_ld, vp_actions = VirtualPortsLaunch.generate_launch_description_with_actions()

    ld = IncludeLaunchDescription(
        LaunchDescriptionSource(vp_ld),
        launch_arguments={
            "tty0": str(tty0),
            "tty1": str(tty1),
        }.items(),
    )
    yield ld, vp_actions


@pytest.fixture
def micro_ros_agent_serial(device_dir):
    """Fixture to create a micro_ros_agent node."""
    tty0 = Path(device_dir, "dev", "tty0").resolve()

    mra_ld, mra_actions = MicroRosAgentLaunch.generate_launch_description_with_actions()

    ld = IncludeLaunchDescription(
        LaunchDescriptionSource(mra_ld),
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
    yield ld, mra_actions


@pytest.fixture
def micro_ros_agent_udp():
    """Fixture to create a micro_ros_agent node."""
    mra_ld, mra_actions = MicroRosAgentLaunch.generate_launch_description_with_actions()

    ld = IncludeLaunchDescription(
        LaunchDescriptionSource(mra_ld),
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
    yield ld, mra_actions


@pytest.fixture
def mavproxy():
    """Fixture to bring up MAVProxy."""
    mp_ld, mp_actions = MAVProxyLaunch.generate_launch_description_with_actions()

    ld = IncludeLaunchDescription(
        LaunchDescriptionSource(mp_ld),
        launch_arguments={
            "master": "tcp:127.0.0.1:5760",
            "sitl": "127.0.0.1:5501",
        }.items(),
    )
    yield ld, mp_actions


@pytest.fixture
def sitl_copter_dds_serial(device_dir, virtual_ports, micro_ros_agent_serial, mavproxy):
    """Fixture to bring up ArduPilot SITL DDS."""
    tty1 = Path(device_dir, "dev", "tty1").resolve()

    vp_ld, vp_actions = virtual_ports
    mra_ld, mra_actions = micro_ros_agent_serial
    mp_ld, mp_actions = mavproxy
    sitl_ld, sitl_actions = SITLLaunch.generate_launch_description_with_actions()

    sitl_ld_args = IncludeLaunchDescription(
        LaunchDescriptionSource(sitl_ld),
        launch_arguments={
            "command": "arducopter",
            "synthetic_clock": "True",
            # "wipe": "True",
            "wipe": "False",
            "model": "quad",
            "speedup": "10",
            "slave": "0",
            "instance": "0",
            "serial1": f"uart:{str(tty1)}",
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

    ld = LaunchDescription(
        [
            vp_ld,
            mra_ld,
            mp_ld,
            sitl_ld_args,
        ]
    )
    actions = {}
    actions.update(vp_actions)
    actions.update(mra_actions)
    actions.update(mp_actions)
    actions.update(sitl_actions)
    yield ld, actions


@pytest.fixture
def sitl_copter_dds_udp(micro_ros_agent_udp, mavproxy):
    """Fixture to bring up ArduPilot SITL DDS."""
    mra_ld, mra_actions = micro_ros_agent_udp
    mp_ld, mp_actions = mavproxy
    sitl_ld, sitl_actions = SITLLaunch.generate_launch_description_with_actions()

    sitl_ld_args = IncludeLaunchDescription(
        LaunchDescriptionSource(sitl_ld),
        launch_arguments={
            "command": "arducopter",
            "synthetic_clock": "True",
            # "wipe": "True",
            "wipe": "False",
            "model": "quad",
            "speedup": "10",
            "slave": "0",
            "instance": "0",
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

    ld = LaunchDescription(
        [
            mra_ld,
            mp_ld,
            sitl_ld_args,
        ]
    )
    actions = {}
    actions.update(mra_actions)
    actions.update(mp_actions)
    actions.update(sitl_actions)
    yield ld, actions
