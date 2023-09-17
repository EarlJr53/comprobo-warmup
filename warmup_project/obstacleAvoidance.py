#Brooke and Swasti's person follower node
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import math
import time
from numpy import inf
import pandas as pd
import numpy as np


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__("ObstacleAvoidance")
        self.create_timer(0.1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10) #for publishing marker at person location
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.process_scan, qos_profile=qos_profile_sensor_data)

        self.flag = False
        #note: can add bump to detect something youve run tino thats too close for the lidar
    def process_scan(self, scan):
        #scan in front of the Neato 
        #print("updating scan")
        neatoPathScanLeft = list(scan.ranges[0:10])
        neatoPathScanRight= list(scan.ranges[350:360])
        generalLeftScan = list(scan.ranges[10:80])
        generalRightScan = list(scan.ranges[250:350])

        #for physical neatos, remove zeroes to prevent issues with min() later on
        if max(neatoPathScanLeft) == 0.0:
            neatoPathScanLeft = [100]
        else:
            neatoPathScanLeft = [i for i in neatoPathScanLeft if i != 0.0]

        if max(neatoPathScanRight) == 0.0:
            neatoPathScanRight = [100]
        else: 
            neatoPathScanRight = [i for i in neatoPathScanRight if i != 0.0]
        
        if max(generalLeftScan) == 0.0:
            generalLeftScan = [100]
        else: 
            generalLeftScan = [i for i in generalLeftScan if i != 0.0]

        if max(generalRightScan) == 0.0:
            generalRightScan = [100]
        else: 
            generalRightScan = [i for i in generalRightScan if i != 0.0]
  
        neatoFullScan = generalLeftScan + neatoPathScanLeft + neatoPathScanRight +generalRightScan
        straightAhead = scan.ranges[0]
        #print("neatoPathLeft is: ", X)
        #print("StraightAhead is: ", straightAhead)

        if (straightAhead <= 0.5 and straightAhead != 0.0) or (self.flag == True):
            self.turnUntilClear(straightAhead, generalLeftScan,generalRightScan)
            self.flag = False
        else: # straightAhead > 0.5 or straightAhead == 0.0:
            self.choosePath(neatoPathScanLeft, neatoPathScanRight, generalLeftScan,generalRightScan)

    def choosePath(self, neatoPathScanLeft, neatoPathScanRight, generalLeftScan, generalRightScan):
        msg = Twist()
        
        #choose z angular velocity
        if ((min(generalLeftScan) < 0.7 and min(generalLeftScan) != 0.0) or ((min(generalRightScan) < 0.7) and min(generalRightScan) != 0.0)):
            if min(generalLeftScan) < min(generalRightScan):
                print("turning right")
                z_value = -(min(generalLeftScan)/0.7)*0.3 +0.05
            else:
                print("turning left")
                z_value = (min(generalRightScan)/0.7)*0.3 + 0.05
        else:
            z_value = 0.0        
        
        #choose x linear velocity
        # print("minimum on left front is: ", min(neatoPathScanLeft))
        # print("minimum on right front is: ", min(neatoPathScanRight))
        # print("minimum on left side is: ", min(generalLeftScan))
        # print("minimum on right side is: ", min(generalRightScan))
        if ((min(neatoPathScanLeft)>0.5 or min(neatoPathScanLeft) == 0.0)) and ((min(neatoPathScanRight)>0.5) or min(neatoPathScanRight) == 0.0):
            x_value = 0.2
            sleep_time = 0.1
        elif min(neatoPathScanLeft)>0.3 and min(neatoPathScanRight)>0.3:
            x_value = ((min(min(neatoPathScanLeft), min(neatoPathScanRight))-0.3)/0.2)*0.15 + 0.01
            sleep_time = 0.1
        else:
            print("whoa, backing up")
            sleep_time = 0.2
            z_value = 0.0 #override prev. z_value if something is in the way
            x_value = -0.05
            self.flag = True

        #also check if there is something very close in periphery: if so, override prev values and back up
        if ((min(generalLeftScan) < 0.25 and min(generalLeftScan) != 0.0) or ((min(generalRightScan) < 0.25) and min(generalRightScan) != 0.0)):
            print("whoa, backing up")
            sleep_time = 0.2
            z_value = 0.0 #override prev. z_value if something is in the way
            x_value = -0.05
            self.flag = True

        msg.angular.z = z_value
        msg.linear.x = x_value
        self.vel_pub.publish(msg)
        time.sleep(sleep_time) 
    
    def turnUntilClear(self, straightAhead, generalLeftScan, generalRightScan):
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = -0.2
        self.vel_pub.publish(msg)
        print("moving backward")
        time.sleep(1.5)
        
        if min(generalLeftScan) < min(generalRightScan):
            print("turning right until clear")
            msg.angular.z = -0.3
        else:
            print("turning left until clear")
            msg.angular.z = 0.3
        msg.linear.x = 0.0
        self.vel_pub.publish(msg)
        time.sleep(3) #consider making this pause very short/ so continuously updating
    
    def run_loop(self):
        msg = Twist()

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
