"""
Nodo Pong - Responde a pings (comunicación bidireccional simple)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PongNode(Node):
    def __init__(self):
        super().__init__('pong_node')
        
        # Subscriber para ping
        self.ping_sub = self.create_subscription(
            String,
            'ping',
            self.ping_callback,
            10
        )
        
        # Publisher para pong
        self.pong_pub = self.create_publisher(String, 'pong', 10)
        
        self.pong_count = 0
        
        self.get_logger().info('Pong Node iniciado, esperando pings...')

    def ping_callback(self, msg):
        self.pong_count += 1
        
        self.get_logger().info(f'🏓 PING recibido: {msg.data}')
        
        # Responder con pong
        pong_msg = String()
        pong_msg.data = msg.data.replace('ping', 'pong')
        
        self.pong_pub.publish(pong_msg)
        self.get_logger().info(f'✓ PONG enviado: {pong_msg.data} (total: {self.pong_count})')


def main(args=None):
    rclpy.init(args=args)
    node = PongNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
