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
import launch_pytest
import pytest

from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import LogInfo
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.event_handlers import OnProcessExit
from launch.events import matches_action
from launch.events.process import ShutdownProcess

from launch_pytest.tools import process as process_tools


@pytest.fixture()
def dummy_proc():
    """Return a dummy process action to manage test lifetime."""
    yield ExecuteProcess(
        cmd=["echo", "ardupilot_dds_tests"],
        shell=True,
        cached_output=True,
    )


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

    yield LaunchDescription(
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
    # TODO: check virtual port process for regex:
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
