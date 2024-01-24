import rclpy
from ardupilot_msgs.srv import ArmMotors


def test_arm_motors_service():
    rclpy.init()

    # Create a ROS node
    node = rclpy.create_node('arm_motors_client')

    # Create a service client for the ArmMotors service
    client = node.create_client(ArmMotors, '/ap/arm_motors')

    # Wait for the service to be available
    if not client.wait_for_service(timeout_sec=10.0):
        node.get_logger().info('Service not available.')
        return

    # Create a request with 'arm' set to True
    request = ArmMotors.Request()
    request.arm = True

    # Call the service
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    # Check if the service call was successful
    if future.result() is not None:
        response = future.result()
        assert response.result, "AP: Arming request unsuccessful"
        node.get_logger().info("AP: Arming request successful")
    else:
        node.get_logger().info("AP: Arming service call failed.")

    # Shutdown the node
    node.destroy_node()
    rclpy.shutdown()
