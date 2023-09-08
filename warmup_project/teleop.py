import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import sleep
import tty
import select
import sys
import termios


class TeleopNode(Node):
    def __init__(self):
        super().__init__("teleop_node")
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.run_loop()

    def drive(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

    def run_loop(self):
        self.drive(0.0, 0.0)
        sleep(1)

        key = None

        while key != "\x03":
            # key = None
            key = getKey()
            if key == "w":
                self.drive(linear=0.3, angular=0.0)
                # sleep(1)
                # self.drive(linear=0.0, angular=0.0)
            elif key == "s":
                self.drive(linear=-0.3, angular=0.0)
                # sleep(1)
                # self.drive(linear=0.0, angular=0.0)
            elif key == "a":
                self.drive(linear=0.0, angular=0.5)
                # sleep(1)
                # self.drive(linear=0.0, angular=0.0)
            elif key == "d":
                self.drive(linear=0.0, angular=-0.5)
                # sleep(1)
                # self.drive(linear=0.0, angular=0.0)
            else:
                self.drive(linear=0.0, angular=0.0)
            print(key)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


settings = termios.tcgetattr(sys.stdin)


if __name__ == "__main__":
    main()
