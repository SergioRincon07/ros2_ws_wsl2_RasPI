"""
Nodo Service Server - Responde a peticiones de prueba
"""
import rclpy
from rclpy.node import Node
from wsl_raspi_comm_msgs.srv import TestService
import time
import socket


class ServiceServer(Node):
    def __init__(self):
        super().__init__('service_server')
        
        # Parámetros
        self.declare_parameter('service_name', 'test_service')
        service_name = self.get_parameter('service_name').value
        
        self.device_name = socket.gethostname()
        self.request_count = 0
        
        # Crear servicio
        self.srv = self.create_service(
            TestService,
            service_name,
            self.handle_request
        )
        
        self.get_logger().info(f'Service Server iniciado en {self.device_name}')
        self.get_logger().info(f'Servicio disponible: {service_name}')

    def handle_request(self, request, response):
        start_time = time.time()
        self.request_count += 1
        
        self.get_logger().info(
            f'📨 Petición recibida #{request.request_id}: "{request.request_data}"'
        )
        
        # Procesar la petición
        response.success = True
        response.response_data = f'Respuesta desde {self.device_name}: procesado "{request.request_data}"'
        response.processed_id = request.request_id
        response.processing_time = time.time() - start_time
        
        self.get_logger().info(
            f'📤 Respuesta enviada #{response.processed_id}, '
            f'tiempo={response.processing_time*1000:.2f}ms, '
            f'total_procesados={self.request_count}'
        )
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ServiceServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
