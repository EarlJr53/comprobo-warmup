#Brooke and Swasti's person follower node
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import math

class PersonFollowerNode(Node):
    def __init__(self):
        super().__init__("person_follower_node")
        self.create_timer(0.1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10) #for publishing marker at person location
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
        self.target_distance = 1.0
        self.distance_deadband = 0.15

    def handle_scan(self, scan):
        if scan.ranges[0] != 0.0: #if there's something straight ahead
            #initialize temporary lists
            temp_angles = []
            temp_dist = []
            self.feet_dist = []
            self.feet_ang = []

            #take a scan 
            modscan = scan.ranges[180:] + scan.ranges[:60] #what range is this scanning?
            for i, dist in enumerate(modscan, 120):#enumarate so you can iterate through the list?
                if dist != 0.0 and dist <= 2.0: #if something is detected and within 2 meters, append it to temp_angles and temp_dist
                    temp_angles.append(i)
                    temp_dist.append(dist)
                elif dist > 2.0 and temp_angles: #if temp_angles ISNT empty, it is seen as True(?)
                    self.calc_CoM(temp_angles, temp_dist) #function for calculating center of mass
                    temp_angles = []
                    temp_dist = []
            if self.feet_ang:
                self.calc_centroid()
            else:
                self.centroid_ang = 0.0
                self.centroid_dist = 0.0
    
    def publish_marker(self):
        #double check these x,y calculations
        x = 0.0
        y = 0.0
        x = x + math.sin(self.centroid_ang)*self.centroid_dist
        y = y + math.cos(self.centroid_ang)*self.centroid_dist

        #create a marker at that x,y position
        marker = Marker()
        marker.header.frame_id = "odom";
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "my_namespace";
        marker.id = 0

        marker.type = Marker.SPHERE;
        marker.action = Marker.ADD;
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 1.0
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0; # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish( marker );
    
    def run_loop(self):
        msg = Twist()
        
        #publish marker at location of person in rviz2 (untested)
        self.publish_marker()

        #calculate angle difference between where you are and where you want to get to?
        ang_dif = self.centroid_ang - 180

        print(f"{self.centroid_dist}, {ang_dif}")

        #figure out linear x velocity need
        if self.centroid_dist > (self.target_distance + self.distance_deadband):
            print("too far")
            # too far from person
            msg.linear.x = 0.2
        elif self.centroid_dist < (self.target_distance - self.distance_deadband):
            print("too close")
            # too close to person
            msg.linear.x = 0.0

        #figure out angular velocity need
        print(self.centroid_ang)
        if ang_dif > 7:
            print("turn left")
            # too far right, turn left
            msg.angular.z = 0.3
        elif ang_dif < -7:
            print("turn right")
            # too far left, turn right
            msg.angular.z = -0.3
        else:
            msg.angular.z = 0.0

        #send velocity message to neato
        self.vel_pub.publish(msg)

    #method for calculating center of mass
    def calc_CoM(self, angles, dists):
        self.feet_dist.append(sum(dists) / len(dists))
        self.feet_ang.append(sum(angles) / len(angles))

    #method for calculating centroid? what is centroid?
    def calc_centroid(self):
        # if 90 < (sum(self.feet_ang) / len(self.feet_ang)) < 270:
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