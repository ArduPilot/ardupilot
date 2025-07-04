# Copyright 2025 ArduPilot.org.
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

import launch_pytest
from launch import LaunchDescription


@launch_pytest.fixture(scope="function")
def launch_sitl_copter_dds_serial(sitl_copter_dds_serial):
    """Launch SITL Copter with DDS over serial."""
    sitl_ld, sitl_actions = sitl_copter_dds_serial
    yield LaunchDescription([sitl_ld, launch_pytest.actions.ReadyToTest()]), sitl_actions


@launch_pytest.fixture(scope="function")
def launch_sitl_copter_dds_udp(sitl_copter_dds_udp):
    """Launch SITL Copter with DDS over UDP."""
    sitl_ld, sitl_actions = sitl_copter_dds_udp
    yield LaunchDescription([sitl_ld, launch_pytest.actions.ReadyToTest()]), sitl_actions


@launch_pytest.fixture(scope="function")
def launch_sitl_plane_dds_serial(sitl_plane_dds_serial):
    """Launch SITL Plane with DDS over serial."""
    sitl_ld, sitl_actions = sitl_plane_dds_serial
    yield LaunchDescription([sitl_ld, launch_pytest.actions.ReadyToTest()]), sitl_actions


@launch_pytest.fixture(scope="function")
def launch_sitl_plane_dds_udp(sitl_plane_dds_udp):
    """Launch SITL Plane with DDS over UDP."""
    sitl_ld, sitl_actions = sitl_plane_dds_udp
    yield LaunchDescription([sitl_ld, launch_pytest.actions.ReadyToTest()]), sitl_actions
