"""
Nodo Ping - Envía ping y espera pong (comunicación bidireccional simple)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class PingNode(Node):
    def __init__(self):
        super().__init__('ping_node')
        
        # Publisher para ping
        self.ping_pub = self.create_publisher(String, 'ping', 10)
        
        # Subscriber para pong
        self.pong_sub = self.create_subscription(
            String,
            'pong',
            self.pong_callback,
            10
        )
        
        # Timer para enviar pings
        self.timer = self.create_timer(2.0, self.send_ping)
        self.ping_count = 0
        self.ping_times = {}
        
        self.get_logger().info('Ping Node iniciado')

    def send_ping(self):
        msg = String()
        msg.data = f'ping_{self.ping_count}'
        
        self.ping_times[msg.data] = time.time()
        self.ping_pub.publish(msg)
        
        self.get_logger().info(f'🏓 PING enviado: {msg.data}')
        self.ping_count += 1

    def pong_callback(self, msg):
        if msg.data.replace('pong', 'ping') in self.ping_times:
            ping_id = msg.data.replace('pong', 'ping')
            rtt = (time.time() - self.ping_times[ping_id]) * 1000
            self.get_logger().info(f'✓ PONG recibido: {msg.data}, RTT={rtt:.2f}ms')
            del self.ping_times[ping_id]
        else:
            self.get_logger().warn(f'⚠️  PONG inesperado: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = PingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
