#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_interfaces.msg import CrossDistroMessage

class PiSubscriber(Node):
    def __init__(self):
        super().__init__('pi_subscriber')
        self.subscription = self.create_subscription(
            CrossDistroMessage,
            'cross_distro_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    pi_subscriber = PiSubscriber()
    rclpy.spin(pi_subscriber)
    pi_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()