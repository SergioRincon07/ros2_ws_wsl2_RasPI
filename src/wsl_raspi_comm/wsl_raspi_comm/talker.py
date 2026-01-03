import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self):
        super().__init__('wsl_talker')
        self.publisher_ = self.create_publisher(String, 'wsl_to_raspi', 10)
        self.timer = self.create_timer(1.0, self.publish_message)
        self.counter = 0

    def publish_message(self):
        msg = String()
        msg.data = f'Hola desde WSL #{self.counter}'
        self.publisher_.publish(msg)
        self.get_logger().info(msg.data)
        self.counter += 1


def main():
    rclpy.init()
    node = Talker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Talker detenido por Ctrl+C')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
