import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
from time import sleep


class WallFollowerNode(Node):
    def __init__(self):
        super().__init__("wall_follower_node")
        self.create_timer(0.1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            "scan",
            self.handle_scan,
            qos_profile=qos_profile_sensor_data,
        )
        self.shortest_angle = 0.0
        self.target_distance = 1.0
        self.distance_deadband = 0.15
        self.shortest_distance = 5.0
        self.Kp = 0.05

    def handle_scan(self, scan):
        if scan.ranges[270] != 0.0:
            self.shortest_distance = 5.0
            for i, dist in enumerate(scan.ranges):
                if dist <= self.shortest_distance:
                    self.shortest_distance = dist
                    self.shortest_angle = i
                    print(f"{dist}, {i}")

    def run_loop(self):
        msg = Twist()

        ang_dif = self.shortest_angle - 270.0
        msg.linear.x = 0.2
        if self.shortest_distance > (self.target_distance + self.distance_deadband):
            print("too far")
            # too far from wall
            if 10.0 <= ang_dif <= 20.0:
                msg.angular.z = 0.0
            elif ang_dif < 10.0:
                msg.angular.z = -0.2
            elif ang_dif > 20.0:
                msg.angular.z = 0.2
        elif self.shortest_distance < (self.target_distance - self.distance_deadband):
            print("too close")
            # too close to wall
            if -10.0 >= ang_dif >= -20.0:
                msg.angular.z = 0.0
            elif ang_dif > -10.0:
                msg.angular.z = 0.2
            elif ang_dif < -20.0:
                msg.angular.z = -0.2
        else:
            print("right distance")
            # at target distance from wall
            if -1.0 <= ang_dif <= 1.0:
                msg.angular.z = 0.0
            elif ang_dif > 1.0:
                msg.angular.z = 0.2
            elif ang_dif < -1.0:
                msg.angular.z = -0.2

        self.vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
