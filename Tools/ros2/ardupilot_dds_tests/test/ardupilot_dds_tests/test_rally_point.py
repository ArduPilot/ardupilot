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
Tests the Rally Point interface.

Clear the list, adds points and verifies that they are correctly updated.

colcon test --packages-select ardupilot_dds_tests \
--event-handlers=console_cohesion+ --pytest-args -k test_rally_point

"""

import rclpy
import time
import launch_pytest
from rclpy.node import Node
from builtin_interfaces.msg import Time
from ardupilot_msgs.msg import Rally
from ardupilot_msgs.srv import RallyGet
from ardupilot_msgs.srv import RallySet
import pytest
from launch_pytest.tools import process as process_tools
from launch import LaunchDescription
import threading

RALLY_0 = Rally()
RALLY_0.point.latitude =-35.1234567
RALLY_0.point.longitude = 149.1234567
RALLY_0.point.altitude = 400.0
RALLY_0.altitude_frame = 2


class RallyControl(Node):
    """Push/Pull Rally points."""

    def __init__(self):
        """Initialise the node."""
        super().__init__("rally_service")
        self.clear_rally_event = threading.Event()
        self.add_rally_event = threading.Event()
        self.get_rally_event = threading.Event()

        self._client_rally_get = self.create_client(RallyGet, "/ap/rally_get")
        self._client_rally_set = self.create_client(RallySet, "/ap/rally_set")
        # Add a spin thread.
        self.ros_spin_thread = threading.Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()
        
    def _clear_list(self):
        req = RallySet.Request()
        req.clear = True
        self.future = self._client_rally_set.call_async(req)
        time.sleep(1.0)
        if self.future.result() is None:
            return False
        else:
            print(self.future)
            print(self.future.result())
            return self.future.result().size == 0
        
    def _send_rally(self, rally):
        req = RallySet.Request()
        req.clear = False
        req.rally = rally
        self.future = self._client_rally_set.call_async(req)
        time.sleep(1.0)
        return self.future.result()
    
    def _get_rally(self):
        req = RallyGet.Request()
        req.index = 0
        self.future = self._client_rally_get.call_async(req)
        time.sleep(1.0)
        return self.future.result()        

    #-------------- PROCESSES -----------------
    def process_clear(self):
        print("---> Process start")
        while True:
            if self._clear_list():
                print("---> List Cleared")
                self.clear_rally_event.set()
                return
            time.sleep(1)
        
    def clear_list(self):
        try:
            self.clear_thread.stop()
        except:
            print("---> Starting thread")
            self.clear_thread = threading.Thread(target=self.process_clear)
            self.clear_thread.start()

    def process_send(self, rally):
        while True:
            response = self._send_rally(rally)
            if response is not None:
                if response.success:
                    self.add_rally_event.set()
                    return
            time.sleep(1)
        
    def send_rally(self, rally):
        try:
            self.send_thread.stop()
        except:
            self.send_thread = threading.Thread(target=self.process_send, args=[rally])
            self.send_thread.start()

    def process_compare(self, rally):
        while True:
            response = self._get_rally()
            if response is not None:
                if response.success:
                    if (abs(response.rally.point.latitude - rally.point.latitude) < 1e-7
                        and abs(response.rally.point.longitude - rally.point.longitude) < 1e-7
                        and response.rally.altitude_frame == rally.altitude_frame):
                        self.get_rally_event.set()
                    else:
                        print (f" Latitude: {response.rally.point.latitude} - diff: {abs(response.rally.point.latitude - rally.point.latitude)}")
                        print (f" Longitude: {response.rally.point.longitude} - diff: {abs(response.rally.point.longitude - rally.point.longitude)}")
                        print (response.rally.altitude_frame == rally.altitude_frame)
                    return
            time.sleep(1)
        
    def get_and_compare_rally(self, rally):
        try:
            self.get_thread.stop()
        except:
            self.get_thread = threading.Thread(target=self.process_compare, args=[rally])
            self.get_thread.start()    

@launch_pytest.fixture
def launch_sitl_copter_dds_serial(sitl_copter_dds_serial):
    """Fixture to create the launch description."""
    sitl_ld, sitl_actions = sitl_copter_dds_serial

    ld = LaunchDescription(
        [
            sitl_ld,
            launch_pytest.actions.ReadyToTest(),
        ]
    )
    actions = sitl_actions
    yield ld, actions

@launch_pytest.fixture
def launch_sitl_copter_dds_udp(sitl_copter_dds_udp):
    """Fixture to create the launch description."""
    sitl_ld, sitl_actions = sitl_copter_dds_udp

    ld = LaunchDescription(
        [
            sitl_ld,
            launch_pytest.actions.ReadyToTest(),
        ]
    )
    actions = sitl_actions
    yield ld, actions
    
@pytest.mark.launch(fixture=launch_sitl_copter_dds_udp)
def test_dds_udp_rally(launch_context, launch_sitl_copter_dds_udp):
    """Test Rally Points with AP_DDS."""
    _, actions = launch_sitl_copter_dds_udp
    micro_ros_agent = actions["micro_ros_agent"].action
    mavproxy = actions["mavproxy"].action
    sitl = actions["sitl"].action

    # Wait for process to start.
    process_tools.wait_for_start_sync(launch_context, micro_ros_agent, timeout=2)
    process_tools.wait_for_start_sync(launch_context, mavproxy, timeout=2)
    process_tools.wait_for_start_sync(launch_context, sitl, timeout=2)

    rclpy.init()
    time.sleep(5)
    try:
        node = RallyControl()
        node.clear_list()
        clear_flag = node.clear_rally_event.wait(5)
        assert clear_flag, "Could not clear"
        node.send_rally(RALLY_0)

        send_flag = node.add_rally_event.wait(5)
        assert send_flag, "Could not send Rally"
        
        node.get_and_compare_rally(RALLY_0)
        send_flag = node.get_rally_event.wait(5)
        assert send_flag, "Wrong Rally point back"        
    finally:
        rclpy.shutdown()
    yield
