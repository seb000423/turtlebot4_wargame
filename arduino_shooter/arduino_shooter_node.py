import time
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class ArduinoShooterNode(Node):
    def __init__(self):
        super().__init__('arduino_shooter_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 9600)

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value

        self.serial = None

        try:
            self.get_logger().info(f'Opening Arduino serial: {self.port}')
            self.serial = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2.0)
            self.get_logger().info('Arduino connected')
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')

        self.fire_sub = self.create_subscription(
            Empty,
            '/blue/shooter/fire',
            self.fire_callback,
            10
        )

        self.get_logger().info('Subscribed to /blue/shooter/fire')

    def fire_callback(self, msg):
        if self.serial is None:
            self.get_logger().error('Arduino serial is not connected')
            return

        try:
            self.serial.write(b'F')
            self.get_logger().info('Fire command sent: F')
        except Exception as e:
            self.get_logger().error(f'Failed to send command: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoShooterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial is not None:
            node.serial.close()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
