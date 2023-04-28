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

"""Use `socat` to create virtual ports ttyROS0 and ttyRO1 and check they exist."""
import os
import launch_pytest
import pytest

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.events.process import ShutdownProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

from launch_pytest.tools import process as process_tools

from launch_ros.substitutions import FindPackageShare

from pathlib import Path


@pytest.fixture()
def dummy_proc():
    """Return a dummy process action to manage test lifetime."""
    return ExecuteProcess(
        cmd=["echo", "ardupilot_dds_tests"],
        shell=True,
        cached_output=True,
    )


@pytest.fixture()
def virtual_ports(device_dir):
    """Fixture that includes and configures the virtual port launch."""
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
    return virtual_ports


@launch_pytest.fixture()
def launch_description(dummy_proc, virtual_ports):
    """Fixture to create the launch description."""
    on_start = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=dummy_proc,
            on_start=[
                LogInfo(msg="Test started."),
            ],
        )
    )

    on_process_exit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=dummy_proc,
            on_exit=[
                LogInfo(msg="Test stopping."),
                EmitEvent(
                    event=ShutdownProcess(process_matcher=matches_action(virtual_ports))
                ),
            ],
        )
    )

    return LaunchDescription(
        [
            dummy_proc,
            virtual_ports,
            on_start,
            on_process_exit,
            launch_pytest.actions.ReadyToTest(),
        ]
    )


@pytest.mark.launch(fixture=launch_description)
def test_virtual_ports(dummy_proc, virtual_ports, launch_context):
    """Test the virtual ports are present."""
    # TODO: check virtial port process for regex:
    #   "starting data transfer loop"

    def validate_output(output):
        assert output.splitlines() == [
            "ardupilot_dds_tests"
        ], "Test process had no output."

    process_tools.assert_output_sync(
        launch_context, dummy_proc, validate_output, timeout=5
    )

    yield

    # Anything below this line is executed after launch service shutdown.
    assert dummy_proc.return_code == 0
