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
Bring up ArduPilot SITL and check that the get/set_parameters services are up and running.

Checks whether a parameter is changed using service call.

colcon test --packages-select ardupilot_dds_tests \
--event-handlers=console_cohesion+ --pytest-args -k test_parameter_service

"""

import launch_pytest
import pytest
import rclpy
import rclpy.node
import threading
import time

from launch import LaunchDescription

from launch_pytest.tools import process as process_tools

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy

from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter

# Enums for parameter type
PARAMETER_NOT_SET = 0
PARAMETER_INTEGER = 2
PARAMETER_DOUBLE = 3


class ParameterClient(rclpy.node.Node):
    """Send GetParameters and SetParameters Requests."""

    def __init__(self):
        """Initialize the node."""
        super().__init__('parameter_client')
        self.get_param_event_object = threading.Event()
        self.set_param_event_object = threading.Event()
        self.set_correct_object = threading.Event()

    def start_client(self):
        """Start the parameter client."""
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Define clients for getting and setting parameter
        self.get_cli = self.create_client(GetParameters, 'ap/get_parameters')
        while not self.get_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('GetParameters service not available, waiting again...')

        self.set_cli = self.create_client(SetParameters, 'ap/set_parameters')
        while not self.set_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('SetParameters service not availabel, waiting again...')

        # Add a spin thread
        self.ros_spin_thread = threading.Thread(target=lambda node: rclpy.spin(node), args=(self,))
        self.ros_spin_thread.start()

    def send_get_param_req(self, param_name):
        self.get_req = GetParameters.Request()
        self.get_req.names.append(param_name)

        self.get_future = self.get_cli.call_async(self.get_req)
        while not self.get_future.done():
            self.get_logger().info("Waiting for GetParameters service response...")
            time.sleep(0.1)

        resp = self.get_future.result()
        value = resp.values[0]

        if value.type != PARAMETER_NOT_SET:
            self.get_param_event_object.set()

    def send_set_param_req(self, param_name, param_value, param_type):
        self.set_req = SetParameters.Request()
        param_update = Parameter()
        param_update.name = param_name
        param_update.value.type = param_type
        if param_type == PARAMETER_INTEGER:
            param_update.value.integer_value = int(param_value)
        else:
            param_update.value.double_value = float(param_value)

        self.set_req.parameters.append(param_update)

        self.desired_value = param_value
        get_response = self.get_future.result()
        initial_value = get_response.values[0]

        if initial_value.type == PARAMETER_INTEGER:
            self.param_value = initial_value.integer_value
        elif initial_value.type == PARAMETER_DOUBLE:
            self.param_value = initial_value.double_value
        else:
            self.param_value = 'nan'

        self.set_future = self.set_cli.call_async(self.set_req)

        while not self.set_future.done():
            self.get_logger().info("Waiting for SetParameters service response...")
            time.sleep(0.1)

        if self.set_future.done():
            response = self.set_future.result()
            if response.results[0].successful:
                self.set_param_event_object.set()

    def check_param_change(self):
        param_name = self.set_req.parameters[0].name

        self.send_get_param_req(param_name)

        get_response = self.get_future.result()
        new_value = get_response.values[0]

        if new_value.type == PARAMETER_INTEGER:
            updated_value = new_value.integer_value
        elif new_value.type == PARAMETER_DOUBLE:
            updated_value = new_value.double_value
        else:
            updated_value = 'nan'

        if updated_value == self.desired_value:
            self.set_correct_object.set()


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
def test_dds_udp_parameter_services(launch_context, launch_sitl_copter_dds_udp):
    """Test Get/Set parameter services broadcast by AP_DDS."""
    _, actions = launch_sitl_copter_dds_udp
    micro_ros_agent = actions["micro_ros_agent"].action
    mavproxy = actions["mavproxy"].action
    sitl = actions["sitl"].action

    # Wait for process to start.
    process_tools.wait_for_start_sync(launch_context, micro_ros_agent, timeout=2)
    process_tools.wait_for_start_sync(launch_context, mavproxy, timeout=2)
    process_tools.wait_for_start_sync(launch_context, sitl, timeout=2)

    rclpy.init()
    try:
        node = ParameterClient()
        node.start_client()
        parameter_name = "LOIT_SPEED"
        param_change_value = 1250
        param_type = PARAMETER_DOUBLE
        node.send_get_param_req(parameter_name)
        get_param_received_flag = node.get_param_event_object.wait(timeout=10.0)
        assert get_param_received_flag, f"Did not get '{parameter_name}' param."
        node.send_set_param_req(parameter_name, param_change_value, param_type)
        set_param_received_flag = node.set_param_event_object.wait(timeout=10.0)
        assert set_param_received_flag, f"Could not set '{parameter_name}' to '{param_change_value}'"
        node.check_param_change()
        set_param_changed_flag = node.set_correct_object.wait(timeout=10.0)
        assert set_param_changed_flag, f"Did not confirm '{parameter_name}' value change"

    finally:
        rclpy.shutdown()
    yield
