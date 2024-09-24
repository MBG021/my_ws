import rclpy
from rclpy.node import Node
from controller_manager import ControllerManager
from std_msgs.msg import String

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.controller_manager = ControllerManager(self)

        # Cargar el controlador desde el archivo de configuración
        self.controller_manager.load_controller('joint_trajectory_controller')

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
