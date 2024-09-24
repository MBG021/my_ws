#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pybullet as p
import pybullet_data
from std_msgs.msg import Float32MultiArray  # Para enviar múltiples velocidades
from ament_index_python.packages import get_package_share_directory
import os

class PyBulletSim(Node):
    def __init__(self):
        super().__init__('pybullet_sim_node')
        self.get_logger().info('Iniciando simulación en PyBullet')

        # Conectar a PyBullet en modo GUI
        self.physicsClient = p.connect(p.GUI)
        
        p.setGravity(0, 0, -9.81)  # Gravedad en la dirección Z

        # Establecer el directorio de búsqueda adicional para PyBullet (ej: plano)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        # Cargar el plano
        self.planeId = p.loadURDF("plane.urdf")

        # Cargar el modelo rosbot.urdf desde la carpeta del paquete
        package_share_directory = get_package_share_directory('pybullet_sim')
        urdf_path = os.path.join(package_share_directory, 'urdf', 'rosbot.urdf')

        self.get_logger().info(f'Cargando URDF desde: {urdf_path}')
        self.robotId = p.loadURDF(urdf_path, [0, 0, 0])

        # Configurar simulación en tiempo real
        p.setRealTimeSimulation(1)

        # Lista de nombres de los joints que son llantas
        self.wheel_joints = ['left_wheel_joint', 'right_wheel_joint']  # Asegúrate de que estos nombres coincidan con los del URDF

        # Obtener los IDs de los joints de las llantas
        self.joint_indices = {}
        self.get_wheel_joint_indices()

        # Crear un suscriptor para recibir comandos de velocidad de las ruedas
        self.subscription = self.create_subscription(
            Float32MultiArray,  # Tipo de mensaje: lista de velocidades
            'wheel_cmd',  # Nombre del tópico
            self.wheel_cmd_callback,  # Función callback
            10)

        # Inicializar velocidades de las ruedas
        self.wheel_speeds = [0.0, 0.0]

    def get_wheel_joint_indices(self):
        """Obtener los índices de los joints de las ruedas desde el URDF."""
        self.num_joints = p.getNumJoints(self.robotId)
        for joint_index in range(self.num_joints):
            joint_info = p.getJointInfo(self.robotId, joint_index)
            joint_name = joint_info[1].decode("utf-8")  # Nombre del joint
            if joint_name in self.wheel_joints:
                self.joint_indices[joint_name] = joint_index
                self.get_logger().info(f"Joint encontrado: {joint_name}, Índice: {joint_index}")

    def wheel_cmd_callback(self, msg):
        """Callback para recibir las velocidades de las ruedas."""
        if len(msg.data) == 2:
            self.wheel_speeds[0] = msg.data[0]  # Velocidad para left_wheel_joint
            self.wheel_speeds[1] = msg.data[1]  # Velocidad para right_wheel_joint
            self.get_logger().info(f"Recibidas velocidades: {self.wheel_speeds}")
        else:
            self.get_logger().error("El mensaje de velocidad debe contener exactamente 2 valores.")

    def update_wheel_joints(self):
        """Actualizar las velocidades de las ruedas."""
        for i, joint_name in enumerate(self.wheel_joints):
            joint_index = self.joint_indices[joint_name]
            p.setJointMotorControl2(
                self.robotId,
                joint_index,
                p.VELOCITY_CONTROL,
                targetVelocity=self.wheel_speeds[i])

    def publish_robot_state(self):
        """Publicar el estado del robot y actualizar las ruedas."""
        self.update_wheel_joints()

    def __del__(self):
        p.disconnect()

def main(args=None):
    rclpy.init(args=args)
    node = PyBulletSim()

    try:
        # Ejecutar el nodo
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
