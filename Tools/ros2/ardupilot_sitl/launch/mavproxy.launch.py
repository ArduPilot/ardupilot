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
from ardupilot_sitl.launch import MAVProxyLaunch


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description for MAVProxy."""
    return MAVProxyLaunch.generate_launch_description()
