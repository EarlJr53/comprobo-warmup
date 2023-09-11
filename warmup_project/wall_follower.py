import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
from time import sleep


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__("wall_follower_node")
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "stable_scan", self.handle_scan, 10
        )
        self.shortest_angle = 0.0
        self.target_distance = 1.0
        self.distance_deadband = 0.05
        # self.angle_deadband = 1.0
        self.shortest_distance = 5.0
        self.run_loop()

    def handle_scan(self, msg):
        if msg.ranges[0] != 0.0:
            self.shortest_distance = 5.0
            for i, dist in enumerate(msg.ranges):
                if dist < self.shortest_distance:
                    self.shortest_distance = dist
                    self.shortest_angle = i
                    print(f"{dist}, {i}")

    def run_loop(self):

        self.drive(0.0, 0.0)
        sleep(1)

        while True:
            ang_dif = self.shortest_angle - 270.0

            if self.shortest_distance > (self.target_distance + self.distance_deadband):
                # too far from wall
                if ang_dif < 14.0:
                    self.turn_right(15 - ang_dif)
                elif ang_dif > 16.0:
                    self.turn_left(15 - ang_dif)
                else:
                    self.drive_forward()
            elif self.shortest_distance < (self.target_distance - self.distance_deadband):
                # too close to wall
                if ang_dif > -14.0:
                    self.turn_left(-15 - ang_dif)
                elif ang_dif < -16.0:
                    self.turn_right(-15 - ang_dif)
                else:
                    self.drive_forward()
            else:
                # at target distance from wall
                if ang_dif > 0.5:
                    self.turn_left(ang_dif)
                elif ang_dif < -0.5:
                    self.turn_right(ang_dif)
                else:
                    self.drive_forward()

    def turn_left(self, angle):
        angular_vel = 0.2
        self.drive(linear=0.0, angular=angular_vel)
        sleep(math.pi * abs(angle) / angular_vel / 180)
        self.drive(linear=0.0, angular=0.0)

    def turn_right(self, angle):
        angular_vel = 0.2
        self.drive(linear=0.0, angular=-angular_vel)
        sleep(math.pi * abs(angle) / angular_vel / 180)
        self.drive(linear=0.0, angular=0.0)

    def drive_forward(self):
        forward_vel = 0.3
        self.drive(linear=forward_vel, angular=0.0)

    def drive(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
