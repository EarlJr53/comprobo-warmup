import rclpy
from rclpy.node import Node

class DriveSquareNode(Node):
    def __init__(self):
        super().__init__('drive_square_node')

def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()