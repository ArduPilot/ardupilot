import pytest
import time
import rclpy
import rclpy.node
import threading

from ardupilot_msgs.srv import ArmMotors
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped


class JoyMessage(rclpy.node.Node):
    def __init__(self):
        super().__init__("joy_message")
        self.msg_event_object = threading.Event()
        self.pose_data_before = 0
        self.pose_data_after = 0

        self.client = self.create_client(ArmMotors, "/ap/arm_motors")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.publisher = self.create_publisher(Joy, "/ap/joy", 10)

    def call_service(self):
        request = ArmMotors.Request()
        request.arm = True

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error('Service call failed')


def publish_joy_message(topic, axes_values):
    node = rclpy.create_node('joy_publisher')
    publisher = node.create_publisher(Joy, topic, 10)

    joy_msg = Joy()
    joy_msg.axes = axes_values

    start_time = time.time()
    while time.time() - start_time < 5.0:
        node.get_logger().info(f"Publishing Joy message to {topic}: {joy_msg}")
        publisher.publish(joy_msg)
        time.sleep(0.1)
    node.destroy_node()


@pytest.mark.launch(fixture=None)
def test_main(args=None):
    rclpy.init(args=args)

    joy_node = JoyMessage()
    joy_node.call_service()

    def pose_callback_before(msg):
        joy_node.pose_data_before = msg.pose.position.x

    joy_node.create_subscription(PoseStamped, '/ap/pose/filtered', pose_callback_before, 10)

    try:
        rclpy.spin_once(joy_node, timeout_sec=1.0)
        arr = [1.0, 0.0, 1.0, 0.0]
        topic = "/ap/joy"
        publish_joy_message(topic, arr)

        def pose_callback(msg):
            joy_node.pose_data_after = msg.pose.position.x

        joy_pose_subscriber = joy_node.create_subscription(PoseStamped, '/ap/pose/filtered', pose_callback, 10)

        rclpy.spin_until_future_complete(joy_node, joy_pose_subscriber)

        assert joy_node.pose_data_after - joy_node.pose_data_before > 0

    finally:
        joy_node.destroy_node()
        rclpy.shutdown()
