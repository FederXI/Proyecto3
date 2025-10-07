#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import SetBool
import threading
import time

class Sprint1Node(Node):
    def __init__(self):
        super().__init__('sprint1')
        
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.srv = self.create_service(SetBool, 'control_dibujo', self.control_callback)
        
        # Cliente para teletransportar la tortuga
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando servicio /turtle1/teleport_absolute...')
        
        self.running = False
        self.stop_flag = False
        
        self.thread = threading.Thread(target=self.draw_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def control_callback(self, request, response):
        if request.data:
            self.running = True
            self.stop_flag = False
            response.success = True
            response.message = "Dibujo iniciado/reanudado"
        else:
            self.stop_flag = True
            self.running = False
            response.success = True
            response.message = "Dibujo detenido"
        return response
    
    def move_line(self, x_start, y_start, x_end, y_end, speed=1.0):
        """
        Dibuja una línea de (x_start,y_start) a (x_end,y_end) usando velocidad.
        """
        # Teletransportar al inicio
        req = TeleportAbsolute.Request()
        req.x = x_start
        req.y = y_start
        req.theta = 0.0
        self.teleport_client.call_async(req)
        time.sleep(0.2)
        
        # Calcular ángulo y distancia
        dx = x_end - x_start
        dy = y_end - y_start
        distance = (dx**2 + dy**2)**0.5
        angle = 0.0
        if distance != 0:
            angle = math.atan2(dy, dx)
        
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = 0.0
        duration = distance / speed
        start = self.get_clock().now().seconds_nanoseconds()[0]
        
        while self.get_clock().now().seconds_nanoseconds()[0] - start < duration:
            if self.stop_flag:
                break
            self.cmd_pub.publish(msg)
            time.sleep(0.05)
        
        # Detener
        msg.linear.x = 0.0
        self.cmd_pub.publish(msg)
    
    def draw_loop(self):
        while True:
            if self.running:
                # Dibujar número 1 usando coordenadas absolutas
                # Línea vertical
                self.move_line(5.5, 2.0, 5.5, 8.0)
                # Base diagonal (opcional)
                self.move_line(5.0, 2.0, 5.5, 2.0)
                self.running = False
            time.sleep(0.1)

def main(args=None):
    import math
    rclpy.init(args=args)
    node = Sprint1Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

