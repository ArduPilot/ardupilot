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
Launch the microROS DDS agent.

Run with default arguments:

ros2 launch ardupilot_sitl micro_ros_agent.launch.py
"""
from launch import LaunchDescription
from ardupilot_sitl.launch import MicroRosAgentLaunch


def generate_launch_description() -> LaunchDescription:
    """Generate a launch description for the micro_ros_agent."""
    return MicroRosAgentLaunch.generate_launch_description()
