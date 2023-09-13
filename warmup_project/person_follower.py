import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__("person_follower_node")
        self.create_timer(0.1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            "scan",
            self.handle_scan,
            qos_profile=qos_profile_sensor_data,
        )
        self.feet_ang = []
        self.feet_dist = []
        self.centroid_ang = 0.0
        self.centroid_dist = 0.0
        self.target_distance = 0.5
        self.distance_deadband = 0.15

    def handle_scan(self, scan):
        if scan.ranges[0] != 0.0:
            temp_angles = []
            temp_dist = []
            self.feet_dist = []
            self.feet_ang = []
            modscan = scan.ranges[180:] + scan.ranges[:180]
            for i, dist in enumerate(modscan):
                if dist != 0.0 and dist <= 2.0:
                    temp_angles.append(i)
                    temp_dist.append(dist)
                elif dist > 2.0 and temp_angles:
                    self.calc_CoM(temp_angles, temp_dist)
                    temp_angles = []
                    temp_dist = []
            if self.feet_ang:
                self.calc_centroid()

    def run_loop(self):
        msg = Twist()

        ang_dif = self.centroid_ang - 180

        if self.centroid_dist > (self.target_distance + self.distance_deadband):
            print("too far")
            # too far from person
            msg.linear.x = 0.2
        elif self.centroid_dist < (self.target_distance - self.distance_deadband):
            print("too close")
            # too close to person
            msg.linear.x = 0.0

        print(self.centroid_ang)
        if ang_dif > 7:
            print("turn left")
            # too far right, turn left
            msg.angular.z = 0.2
        elif ang_dif < -7:
            print("turn right")
            # too far left, turn right
            msg.angular.z = -0.2
        else:
            msg.angular.z = 0.0

        self.vel_pub.publish(msg)

    def calc_CoM(self, angles, dists):
        self.feet_dist.append(sum(dists) / len(dists))
        self.feet_ang.append(sum(angles) / len(angles))

    def calc_centroid(self):
        self.centroid_ang = sum(self.feet_ang) / len(self.feet_ang)
        self.centroid_dist = sum(self.feet_dist) / len(self.feet_dist)


# class FootObject():
#     def __init__(self, angles, dists):
#         self.dist = sum(dists) / len(dists)
#         self.ang = sum(angles) / len(angles)

#     def getAng(self):
#         return self.ang

#     def getDist(self):
#         return self.dist


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
