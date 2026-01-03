"""
Nodo Service Client - Envía peticiones al servidor
"""
import rclpy
from rclpy.node import Node
from wsl_raspi_comm_msgs.srv import TestService
import time
import socket


class ServiceClient(Node):
    def __init__(self):
        super().__init__('service_client')
        
        # Parámetros
        self.declare_parameter('service_name', 'test_service')
        self.declare_parameter('request_rate', 0.5)
        
        service_name = self.get_parameter('service_name').value
        rate = self.get_parameter('request_rate').value
        
        self.device_name = socket.gethostname()
        self.request_id = 0
        
        # Cliente del servicio
        self.cli = self.create_client(TestService, service_name)
        
        # Timer para enviar peticiones
        self.timer = self.create_timer(1.0 / rate, self.send_request)
        
        self.get_logger().info(f'Service Client iniciado en {self.device_name}')
        self.get_logger().info(f'Enviando peticiones a: {service_name} cada {1.0/rate:.1f}s')
        
        # Esperar a que el servicio esté disponible
        self.get_logger().info('Esperando al servicio...')

    def send_request(self):
        if not self.cli.service_is_ready():
            self.get_logger().warn('⚠️  Servicio no disponible aún...')
            return
        
        # Crear petición
        request = TestService.Request()
        request.request_data = f'Petición desde {self.device_name}'
        request.request_id = self.request_id
        
        self.get_logger().info(f'📤 Enviando petición #{self.request_id}')
        
        # Enviar petición de forma asíncrona
        future = self.cli.call_async(request)
        future.add_done_callback(self.handle_response)
        
        self.request_id += 1

    def handle_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f'✓ Respuesta recibida #{response.processed_id}: '
                    f'"{response.response_data}", '
                    f'tiempo_servidor={response.processing_time*1000:.2f}ms'
                )
            else:
                self.get_logger().error(f'❌ Petición falló #{response.processed_id}')
        except Exception as e:
            self.get_logger().error(f'❌ Error en servicio: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ServiceClient()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
