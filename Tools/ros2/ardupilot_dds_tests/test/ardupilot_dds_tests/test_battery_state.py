import rclpy
from std_msgs.msg import Header
from sensor_msgs.msg import BatteryState
from builtin_interfaces.msg import Time


class BatterySubscriber:
    def __init__(self):
        self.node = rclpy.create_node("battery_subscriber")
        self.subscription = self.node.create_subscription(
            BatteryState,
            "/ap/battery/battery0",
            self.battery_callback,
            10,
        )

    def battery_callback(self, msg):
        try:
            # Assert the information here
            assert isinstance(msg.header, Header)
            assert isinstance(msg.header.stamp, Time)
            assert isinstance(msg.voltage, float)
            assert isinstance(msg.temperature, float)
            assert isinstance(msg.current, float)
            assert isinstance(msg.charge, float)
            assert isinstance(msg.capacity, float)
            assert isinstance(msg.design_capacity, float)
            assert isinstance(msg.percentage, float)
            assert isinstance(msg.power_supply_status, int)
            assert isinstance(msg.power_supply_health, int)
            assert isinstance(msg.power_supply_technology, int)
            assert isinstance(msg.present, bool)
            assert isinstance(msg.cell_voltage, list)
            assert isinstance(msg.cell_temperature, list)
            assert isinstance(msg.location, str)
            assert isinstance(msg.serial_number, str)

            print("Assertion passed. Received valid Battery State Message")

        except AssertionError as e:
            print(f"Assertion failed. Invalid Battery State message: {e}")
        finally:
            # Stop the ROS node and exit the program
            rclpy.shutdown()


def test_battery_state(args=None):
    rclpy.init(args=args)
    subscriber = BatterySubscriber()
    rclpy.spin(subscriber.node)
    subscriber.node.destroy_node()
    rclpy.shutdown()
