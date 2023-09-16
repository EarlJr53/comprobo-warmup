import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from time import sleep


class WallFollowerNode(Node):
    """
    Node to follow walls at a given distance
    """

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
        self.target_distance = 0.5
        self.distance_deadband = 0.15
        self.shortest_distance = 5.0
        self.Kp = 0.05

    def handle_scan(self, scan):
        """
        Callback function to handle laser scan when received

        Args:
            scan: Set of laser scan data
        """
        if scan.ranges[270] != 0.0:  # Check for valid data
            self.shortest_distance = 5.0  # Reset min dist to 5.0
            for i, dist in enumerate(scan.ranges):  # Iterate through all angles of scan
                if (dist <= self.shortest_distance) and (dist != 0.0):
                    # If shorter than existing shortest angle
                    # Record new shortest distance and corresponding angle
                    self.shortest_distance = dist
                    self.shortest_angle = i
                    print(f"{dist}, {i}")

    def run_loop(self):
        """
        Drive Neato based on angle relative to wall and distance from wall
        """
        msg = Twist()

        # Difference in angle between shortest distance and Neato heading
        ang_dif = self.shortest_angle - 270.0

        msg.linear.x = 0.1  # Always driving forward

        # If the shortest distance is greater than the target, we're too far from the wall
        if self.shortest_distance > (self.target_distance + self.distance_deadband):
            print("too far")
            # too far from wall, turn to (10:20) degrees to drive closer to wall
            if 10.0 <= ang_dif <= 20.0:
                msg.angular.z = 0.0
            elif ang_dif < 10.0:
                msg.angular.z = -0.2
            elif ang_dif > 20.0:
                msg.angular.z = 0.2

        # If the shortest distance is less than the target, we're too close to the wall
        elif self.shortest_distance < (self.target_distance - self.distance_deadband):
            print("too close")
            # too close to wall, turn to (-10:-20) degrees to drive further from wall
            if -10.0 >= ang_dif >= -20.0:
                msg.angular.z = 0.0
            elif ang_dif > -10.0:
                msg.angular.z = 0.2
            elif ang_dif < -20.0:
                msg.angular.z = -0.2

        # We're the right distance from the wall!
        else:
            print("right distance")
            # at target distance from wall, align to be parallel to wall
            if -1.0 <= ang_dif <= 1.0:
                msg.angular.z = 0.0
            elif ang_dif > 1.0:
                msg.angular.z = 0.2
            elif ang_dif < -1.0:
                msg.angular.z = -0.2

        self.vel_pub.publish(msg)


def main(args=None):
    """
    Create wall follower node
    """
    rclpy.init(args=args)
    node = WallFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
