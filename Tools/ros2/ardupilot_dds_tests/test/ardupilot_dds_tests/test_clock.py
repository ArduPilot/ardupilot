import rclpy
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time


class ClockSubscriber:
    def __init__(self):
        self.node = rclpy.create_node("clock_subscriber")
        self.subscription = self.node.create_subscription(Clock, "/ap/clock", self.clock_callback, 10)

    def clock_callback(self, msg):
        try:
            # Assert the information here
            assert isinstance(msg.clock, Time)
            assert isinstance(msg.clock.sec, int)
            assert isinstance(msg.clock.nanosec, int)

            print("Assertion passed. Received valid Clock message:")
            print(f"Seconds: {msg.clock.sec}")
            print(f"Nanoseconds: {msg.clock.nanosec}")

        except AssertionError as e:
            print(f"Assertion failed. Invalid Clock message: {e}")
        finally:
            # Stop the ROS node and exit the program
            rclpy.shutdown()


def test_clock(args=None):
    rclpy.init(args=args)
    subscriber = ClockSubscriber()
    rclpy.spin(subscriber.node)
