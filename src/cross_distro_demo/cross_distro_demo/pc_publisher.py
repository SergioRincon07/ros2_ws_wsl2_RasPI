import rclpy
from rclpy.node import Node
from custom_interfaces.msg import CrossDistroMessage

class PCPublisher(Node):
    def __init__(self):
        super().__init__('pc_publisher')
        self.publisher_ = self.create_publisher(CrossDistroMessage, 'cross_distro_topic', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        msg = CrossDistroMessage()
        msg.data = f'Hello from PC! Count: {self.count}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    pc_publisher = PCPublisher()
    rclpy.spin(pc_publisher)
    pc_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()