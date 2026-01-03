import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Listener(Node):
    def __init__(self):
        super().__init__('raspi_listener')
        self.subscription = self.create_subscription(
            String,
            'wsl_to_raspi',
            self.callback,
            10)

    def callback(self, msg):
        self.get_logger().info(f'Recibido: {msg.data}')


def main():
    rclpy.init()
    node = Listener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Listener detenido por Ctrl+C')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
