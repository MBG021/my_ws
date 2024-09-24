#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pybullet as p
import pybullet_data
from std_msgs.msg import Float32MultiArray
from ament_index_python.packages import get_package_share_directory
import os

class PyBulletSim(Node):
    def __init__(self):
        super().__init__('pybullet_sim_node')
        self.get_logger().info('Iniciando simulación en PyBullet')

        # Suscribirse al tópico 'wheel_cmd' para recibir velocidades de las ruedas
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'wheel_cmd',
            self.listener_callback,
            10
        )

        # Conectar a PyBullet en modo GUI
        self.physicsClient = p.connect(p.GUI)
        
        p.setGravity(0, 0, -9.81)  # Gravedad en la dirección Z

        # Establecer el directorio de búsqueda adicional para PyBullet (ej: plano)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Cargar el plano
        self.planeId = p.loadURDF("plane.urdf")

        # Cargar el modelo URDF del vehículo desde el paquete
        package_share_directory = get_package_share_directory('pybullet_sim')
        urdf_path = os.path.join(package_share_directory, 'urdf', 'rosbot.urdf')

        self.get_logger().info(f'Cargando URDF desde: {urdf_path}')

        # Cargar el modelo del robot
        self.robotId = p.loadURDF(urdf_path, [0, 0, 0])

        # Configurar simulación en tiempo real
        p.setRealTimeSimulation(1)

        # Obtener los joints de las llantas
        self.left_wheel_joint = p.getJointInfo(self.robotId, 0)[0]  # Índice de la llanta izquierda
        self.right_wheel_joint = p.getJointInfo(self.robotId, 1)[0]  # Índice de la llanta derecha

    def listener_callback(self, msg):
        """Callback que recibe las velocidades de las ruedas desde el nodo de teleoperación"""
        if len(msg.data) == 2:
            left_wheel_speed = msg.data[0]
            right_wheel_speed = msg.data[1]

            # Aplicar las velocidades a los joints de las llantas
            p.setJointMotorControl2(self.robotId, self.left_wheel_joint, p.VELOCITY_CONTROL, targetVelocity=left_wheel_speed)
            p.setJointMotorControl2(self.robotId, self.right_wheel_joint, p.VELOCITY_CONTROL, targetVelocity=right_wheel_speed)

            self.get_logger().info(f'Llanta izquierda: {left_wheel_speed}, Llanta derecha: {right_wheel_speed}')

    def __del__(self):
        p.disconnect()

def main(args=None):
    rclpy.init(args=args)
    node = PyBulletSim()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
