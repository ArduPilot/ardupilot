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

from launch_pytest.tools import process as process_tools


@launch_pytest.fixture
def launch_description(virtual_ports):
    """Launch description fixture."""
    vp_ld, vp_actions = virtual_ports

    ld = LaunchDescription(
        [
            vp_ld,
            launch_pytest.actions.ReadyToTest(),
        ]
    )
    actions = vp_actions
    yield ld, actions


@pytest.mark.launch(fixture=launch_description)
def test_virtual_ports(launch_context, launch_description):
    """Test the virtual ports are present."""
    _, actions = launch_description
    virtual_ports = actions["virtual_ports"].action

    # Wait for process to start.
    process_tools.wait_for_start_sync(launch_context, virtual_ports, timeout=2)

    # Assert contents of output to stderr.
    def validate_output(output):
        assert "N starting data transfer loop" in output, "Test process had no output."

    process_tools.assert_stderr_sync(
        launch_context, virtual_ports, validate_output, timeout=5
    )
    yield
