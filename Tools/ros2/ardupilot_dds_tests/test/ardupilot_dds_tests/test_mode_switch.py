import rclpy
import time
from ardupilot_msgs.srv import ModeSwitch
from enum import Enum


class FlightModes(Enum):
    STABILIZE = 0
    ACRO = 1
    ALT_HOLD = 2
    AUTO = 3
    GUIDED = 4
    LOITER = 5
    RTL = 6
    CIRCLE = 7
    LAND = 9
    DRIFT = 11
    SPORT = 13
    FLIP = 14
    AUTOTUNE = 15
    POSHOLD = 16
    BRAKE = 17
    THROW = 18
    AVOID_ADSB = 19
    GUIDED_NOGPS = 20
    SMART_RTL = 21
    FLOWHOLD = 22
    FOLLOW = 23
    ZIGZAG = 24
    SYSTEMID = 25
    AUTOROTATE = 26
    AUTO_RTL = 27
    TURTLE = 28


def test_mode_switch():
    rclpy.init()

    # Create a ROS node
    node = rclpy.create_node("mode_switch_client")

    # Create a service client for the ModeSwitch service
    client = node.create_client(ModeSwitch, "/ap/mode_switch")

    # Wait for the service to be available
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().info("Service not available.")
        return False

    for mode_value in FlightModes:
        # Create a request with the specified mode value
        request = ModeSwitch.Request()
        request.mode = mode_value.value

        # Call the service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        # Check if the service call was successful
        if future.result() is not None:
            response = future.result()
            assert response.status, f"""Mode switch to {mode_value}
                                        unsuccessful."""
            node.get_logger().info(
                f"""Mode switch to {mode_value}
                                    successful. Current mode:
                                    {response.curr_mode}"""
            )
        else:
            node.get_logger().info("Service call failed.")

        time.sleep(2)
