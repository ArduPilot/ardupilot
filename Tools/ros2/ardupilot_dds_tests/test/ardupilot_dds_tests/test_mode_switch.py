import rclpy
import time
from ardupilot_msgs.srv import ModeSwitch


def test_mode_switch():
    rclpy.init()

    # Create a ROS node
    node = rclpy.create_node('mode_switch_client')

    # Create a service client for the ModeSwitch service
    client = node.create_client(ModeSwitch, '/ap/mode_switch')

    # Wait for the service to be available
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().info('Service not available.')
        return False

    for mode_value in range(10):
        # Create a request with the specified mode value
        request = ModeSwitch.Request()
        request.mode = mode_value

        # Call the service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)

        # Check if the service call was successful
        if future.result() is not None:
            response = future.result()
            assert response.status, f'''Mode switch to {mode_value}
                                        unsuccessful.'''
            node.get_logger().info(f'''Mode switch to {mode_value}
                                    successful. Current mode:
                                    {response.curr_mode}''')
        else:
            node.get_logger().info('Service call failed.')

        time.sleep(2)
