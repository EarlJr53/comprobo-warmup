#Brooke and Swasti's person follower node
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
import math
import time

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__("ObstacleAvoidance")
        self.create_timer(0.1, self.run_loop)
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10) #for publishing marker at person location
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.process_scan, qos_profile=qos_profile_sensor_data)
        
    def process_scan(self, scan):
        #print(list(scan.ranges[0:10]))
        #scan in front of the Neato 
        neatoPathScanLeft = list(scan.ranges[0:10])
        neatoPathScanRight= list(scan.ranges[350:360])
        generalLeftScan = list(scan.ranges[10:90])
        generalRightScan = list(scan.ranges[270:350])
        neatoFullScan = generalLeftScan + neatoPathScanLeft + neatoPathScanRight +generalRightScan
        straightAhead = scan.ranges[0]
        #print(neatoPathScanRight)
        print("StraightAhead is: ", straightAhead)
        if straightAhead <= 0.5 and straightAhead != 0.0:
            self.turnUntilClear(straightAhead)

        else: # straightAhead > 0.5 or straightAhead == 0.0:
            self.choosePath(neatoPathScanLeft, neatoPathScanRight, generalLeftScan,generalRightScan)

    def choosePath(self, neatoPathScanLeft, neatoPathScanRight, generalLeftScan, generalRightScan):
        # print("leftObstacleCoefficient is: ", leftObstacleCoefficient)
        # print("rightObstacleCoefficient is: ", rightObstacleCoefficient)
        msg = Twist()
        
        # #choose an angular z velocity and linear velocity
        # if leftObstacleCoefficient >= 70 or rightObstacleCoefficient >= 70:
        #     z_value = 0.3
        #     x_value = -0.3
        #     sleep_time = 0.5
        # elif (abs(leftObstacleCoefficient - rightObstacleCoefficient) <=15) and max(leftObstacleCoefficient, rightObstacleCoefficient) >60:
        #     z_value = 0.3
        #     x_value = 0.0
        #     sleep_time = 0.5
        # else:
        #     z_value = ((leftObstacleCoefficient - rightObstacleCoefficient)/100)*0.3
        #     x_value = (((leftObstacleCoefficient + rightObstacleCoefficient)/2)/100)*0.3 + 0.01
        #     sleep_time = 0.1

        #choose an angular z velocity and linear velocity
        print("minimum of left: ", min(neatoPathScanLeft))
        print("minimum of right: ", min(neatoPathScanRight))
        
        
        if ((min(neatoPathScanLeft)>0.5 and min(neatoPathScanRight)>0.5) or (min(neatoPathScanLeft) == 0.0 or min(neatoPathScanRight) == 0.0)):
            x_value = 0.2
            z_value = 0.0
            sleep_time = 0.5
        elif min(neatoPathScanLeft)>0.3 and min(neatoPathScanRight)>0.3:
            x_value = ((min(min(neatoPathScanLeft), min(neatoPathScanRight))-0.3)/0.2)*0.2 + 0.01
            print("x_value check", x_value)
            sleep_time = 0.2
            if min(neatoPathScanLeft) < min(neatoPathScanRight):
                z_value = 0.2
            else:
                z_value = -0.2
        else:
            sleep_time = 0.2
            z_value = 0.0
            x_value = -0.05
        
        print("z_value is:", z_value)
        print("x_value is:", x_value)
        msg.angular.z = z_value
        msg.linear.x = x_value
        self.vel_pub.publish(msg)
        time.sleep(sleep_time) #this may cause issues?
    
    def turnUntilClear(self, straightAhead):
        msg = Twist()
        msg.angular.z = 0.0
        msg.linear.x = -0.2
        self.vel_pub.publish(msg)
        print("moving backward")
        time.sleep(2)
        
        msg.angular.z = 0.3   #can decide which way to turn based on periphery later
        msg.linear.x = 0.0
        self.vel_pub.publish(msg)
        time.sleep(3)

        #function that turns right or left until path is clear
    
    def run_loop(self):
        msg = Twist()
        #publish marker at location of person in rviz2 (untested)
        #self.publish_marker()

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidance()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()



        #coefficients representing severity (closeness and density) of obstacles on right and left
        # leftObstacleCoefficient = 0 #range 0~100 unless obstacle is within 0.2m, in which case it goes above 100
        # rightObstacleCoefficient = 0 #range 0~100

        # for point in neatoPathScanLeft:
        #     if point < 2.0:
        #         if point < 0.2:
        #             leftObstacleCoefficient += 30
        #         elif point < 0.5:
        #             leftObstacleCoefficient += 10
        #         else:
        #             leftObstacleCoefficient += 3
        
        # for point in neatoPathScanRight:
        #     #print(point)
        #     if point < 2.0:
        #         if point < 0.2:
        #             rightObstacleCoefficient += 30
        #         elif point < 0.5:
        #             rightObstacleCoefficient += 10
        #         else:
        #             rightObstacleCoefficient += 3
        
        # #print("leftObstacleCoefficient:", leftObstacleCoefficient)
        # #print("rightObstacleCoefficient:", rightObstacleCoefficient)
        
        # self.choosePath(leftObstacleCoefficient, rightObstacleCoefficient)