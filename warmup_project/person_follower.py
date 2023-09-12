import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__('person_follower_node')
        self.create_timer(0.1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.scan_sub = self.create_subscription(
            LaserScan,
            "scan",
            self.handle_scan,
            qos_profile=qos_profile_sensor_data,
        )
        self.feet = []
        self.centroid_ang = 0.0
        self.centroid_dist = 0.0


    def handle_scan(self, scan):
        temp_angles = []
        temp_dist = []
        if scan.ranges[0] != 0.0:
            for i, dist in enumerate(scan.ranges):
                if dist != 0.0 and dist <= 0.5:
                    temp_angles.append(i)
                    temp_dist.append(dist)
                elif dist > 0.5:
                    self.feet.append(FootObject(temp_angles, temp_dist))
                    temp_angles = []
                    temp_dist = []

    def calc_centroid(self):
        self.feet[0].ang


class FootObject():
    def __init__(self, angles, dists):
        self.dist = sum(dists) / len(dists)
        self.ang = sum(angles) / len(angles)

    def getAng(self):
        return self.ang
    
    def getDist(self):
        return self.dist
        


def main(args=None):
    rclpy.init(args=args)
    node = PersonFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()