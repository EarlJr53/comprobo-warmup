import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from time import sleep


class DriveSquareNode(Node):
    """
    Node to drive in a square once
    """

    def __init__(self):
        super().__init__("drive_square_node")
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.run_loop()

    def run_loop(self):
        """
        Drive forward and turn left 4 times
        """
        for _ in range(4):
            self.drive_forward(0.5)
            self.turn_left()

    def turn_left(self):
        """
        Turn left 90 degrees
        """
        angular_vel = 0.3  # Preset angular velocity
        self.drive(linear=0.0, angular=angular_vel)  # Set angular velocity
        sleep(math.pi / angular_vel / 2)  # Wait roughly 90 degrees
        self.drive(linear=0.0, angular=0.0)  # Stop

    def drive_forward(self, distance):
        """
        Drive Neato forward a given distance

        Args:
            distance (float): Distance to drive forward (meters)
        """
        forward_vel = 0.3  # Preset forward velocity
        self.drive(linear=forward_vel, angular=0.0)  # Drive forward
        sleep(distance / forward_vel)  # Wait roughly same as distance
        self.drive(linear=0.0, angular=0.0)  # Stop

    def drive(self, linear, angular):
        """
        Drive Neato with given linear and angular velocities

        Args:
            linear (float): linear velocity
            angular (float): angular velocity
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)


def main(args=None):
    """
    Create drive square node
    """
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
