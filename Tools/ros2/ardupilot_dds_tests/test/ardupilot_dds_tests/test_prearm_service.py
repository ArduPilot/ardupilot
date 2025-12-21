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

# flake8: noqa

"""
Bring up ArduPilot SITL and check whether the vehicle can be armed.

colcon test --executor sequential --parallel-workers 0 --base-paths src/ardupilot \
    --event-handlers=console_cohesion+ --packages-select ardupilot_dds_tests \
    --pytest-args -k test_prearm_service

"""

import time
import pytest
import rclpy
import rclpy.node
import threading

from launch_pytest.tools import process as process_tools
from std_srvs.srv import Trigger

from launch_fixtures import (
    launch_sitl_copter_dds_serial,
    launch_sitl_copter_dds_udp,
)

SERVICE = "/ap/prearm_check"
WAIT_FOR_START_TIMEOUT = 5.0


class PreamService(rclpy.node.Node):
    def __init__(self):
        """Initialise the node."""
        super().__init__("prearm_client")
        self.service_available_object = threading.Event()
        self.is_armable_object = threading.Event()
        self._client_prearm = self.create_client(Trigger, "/ap/prearm_check")

    def start_node(self):
        # Add a spin thread.
        self.ros_spin_thread = threading.Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def prearm_check(self):
        req = Trigger.Request()
        future = self._client_prearm.call_async(req)
        time.sleep(0.2)
        try:
            return future.result().success
        except Exception as e:
            print(e)
            return False

    def prearm_with_timeout(self, timeout: rclpy.duration.Duration):
        result = False
        start = self.get_clock().now()
        while not result and self.get_clock().now() - start < timeout:
            result = self.prearm_check()
            time.sleep(1)
        return result

    def process_prearm(self):
        if self.prearm_with_timeout(rclpy.duration.Duration(seconds=30)):
            self.is_armable_object.set()

    def start_prearm(self):
        try:
            self.prearm_thread.stop()
        except:
            print("start_prearm not started yet")
        self.prearm_thread = threading.Thread(target=self.process_prearm)
        self.prearm_thread.start()


@pytest.mark.launch(fixture=launch_sitl_copter_dds_serial)
def test_dds_serial_prearm_service_call(launch_context, launch_sitl_copter_dds_serial):
    """Test prearm service AP_DDS."""
    _, actions = launch_sitl_copter_dds_serial
    virtual_ports = actions["virtual_ports"].action
    micro_ros_agent = actions["micro_ros_agent"].action
    mavproxy = actions["mavproxy"].action
    sitl = actions["sitl"].action

    # Wait for process to start.
    process_tools.wait_for_start_sync(launch_context, virtual_ports, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, micro_ros_agent, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, mavproxy, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, sitl, timeout=WAIT_FOR_START_TIMEOUT)

    rclpy.init()
    try:
        node = PreamService()
        node.start_node()
        node.start_prearm()
        is_armable_flag = node.is_armable_object.wait(timeout=25.0)
        assert is_armable_flag, f"Vehicle not armable."
    finally:
        rclpy.shutdown()
    yield


@pytest.mark.launch(fixture=launch_sitl_copter_dds_udp)
def test_dds_udp_prearm_service_call(launch_context, launch_sitl_copter_dds_udp):
    """Test prearm service AP_DDS."""
    _, actions = launch_sitl_copter_dds_udp
    micro_ros_agent = actions["micro_ros_agent"].action
    mavproxy = actions["mavproxy"].action
    sitl = actions["sitl"].action

    # Wait for process to start.
    process_tools.wait_for_start_sync(launch_context, micro_ros_agent, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, mavproxy, timeout=WAIT_FOR_START_TIMEOUT)
    process_tools.wait_for_start_sync(launch_context, sitl, timeout=WAIT_FOR_START_TIMEOUT)

    rclpy.init()
    try:
        node = PreamService()
        node.start_node()
        node.start_prearm()
        is_armable_flag = node.is_armable_object.wait(timeout=25.0)
        assert is_armable_flag, f"Vehicle not armable."
    finally:
        rclpy.shutdown()
    yield
