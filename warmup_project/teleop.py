import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from time import sleep
import tty
import select
import sys
import termios


class TeleopNode(Node):
    """
    Teleop Node, drive Neato with keystrokes

    Args:
        Node (rclpy.node.Node)
    """

    def __init__(self):
        super().__init__("teleop_node")
        self.vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.run_loop()

    def drive(self, linear, angular):
        """
        Drive Neato with given linear and angular velocities

        Args:
            linear (float): linear velocity
            angular (float): angular velocity
        """
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.vel_pub.publish(msg)

    def run_loop(self):
        """
        Command drive directions based on keystrokes
        """
        self.drive(0.0, 0.0)  # Neato misses first command
        sleep(1)

        key = None  # Start by clearing key var

        while key != "\x03":  # while key isn't Ctrl + C (exit stroke)
            key = getKey()  # get new keystroke
            if key == "w":
                # Drive forward
                self.drive(linear=0.3, angular=0.0)
            elif key == "s":
                # Drive backward
                self.drive(linear=-0.3, angular=0.0)
            elif key == "a":
                # Turn left
                self.drive(linear=0.0, angular=0.5)
            elif key == "d":
                # Turn right
                self.drive(linear=0.0, angular=-0.5)
            else:
                # Stop when any other key is pressed
                self.drive(linear=0.0, angular=0.0)
            print(key)


def main(args=None):
    """
    Create teleop node
    """
    rclpy.init(args=args)
    node = TeleopNode()
    rclpy.spin(node)
    rclpy.shutdown()


def getKey():
    """
    Wait for next keypress and return to run_loop
    """
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


settings = termios.tcgetattr(sys.stdin)


if __name__ == "__main__":
    main()
