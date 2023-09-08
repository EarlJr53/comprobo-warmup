import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from time import sleep

class DriveSquareNode(Node):
    def __init__(self):
        super().__init__('drive_square_node')
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        # self.odom_sub = self.create_subscription(Odometry, "odom", 10)
        self.run_loop()


    def run_loop(self):
        for _ in range(4):
            self.drive_forward(0.5)
            self.turn_left()

    def turn_left(self):
        angular_vel = 0.3
        self.drive(linear=0.0, angular=angular_vel)
        sleep(math.pi / angular_vel / 2)
        self.drive(linear=0.0, angular=0.0)

    def drive_forward(self, distance):
        forward_vel = 0.3
        self.drive(linear=forward_vel, angular=0.0)
        sleep(distance / forward_vel)
        self.drive(linear=0.0, angular=0.0)


    def drive(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = DriveSquareNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()