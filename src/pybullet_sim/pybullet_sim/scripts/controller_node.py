#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys
import termios
import tty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.get_logger().info('Nodo de teleoperación iniciado')

        # Crear publicador para enviar comandos de velocidad
        self.publisher_ = self.create_publisher(Float32MultiArray, 'wheel_cmd', 10)

        # Mapeo de teclas a velocidades (linear_velocity, angular_velocity)
        self.linear_speed = 1.0  # Velocidad lineal base
        self.angular_speed = 0.5  # Velocidad angular base

        self.key_bindings = {
            'w': [self.linear_speed, self.linear_speed],  # Adelante
            's': [-self.linear_speed, -self.linear_speed],  # Atrás
            'a': [-self.angular_speed, self.angular_speed],  # Girar a la izquierda
            'd': [self.angular_speed, -self.angular_speed],  # Girar a la derecha
            'q': [0.0, 0.0]  # Detener
        }

    def get_key(self):
        """Capturar entrada del teclado en modo no bloqueante."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """Escuchar las teclas y publicar comandos de velocidad."""
        try:
            while rclpy.ok():
                key = self.get_key()
                if key in self.key_bindings:
                    speeds = self.key_bindings[key]
                    self.publish_speeds(speeds)
                elif key == '\x03':  # Ctrl+C para salir
                    break
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def publish_speeds(self, speeds):
        """Publicar las velocidades de las ruedas."""
        msg = Float32MultiArray()
        msg.data = speeds
        self.publisher_.publish(msg)
        self.get_logger().info(f"Velocidades enviadas: {speeds}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
