#!/usr/bin/env python3

import traceback

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class JoyController(Node):
    """for demo"""

    def __init__(self) -> None:
        super().__init__("joy_controller")

        # pub
        self.left_vel_pub = self.create_publisher(Float32, "left/vel", 10)
        self.right_vel_pub = self.create_publisher(Float32, "right/vel", 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # sub
        self.create_subscription(Joy, "/joy", self.joy_callback, 10)

    def joy_callback(self, msg: Joy) -> None:
        left_vel_val = -msg.axes[1]
        right_vel_val = -msg.axes[4]
        left_vel = Float32(data=(left_vel_val))
        right_vel = Float32(data=(right_vel_val))
        front_vel_val = (left_vel_val + right_vel_val) / 2
        angular_vel_val = (right_vel_val - left_vel_val) / 0.3
        cmd_vel = Twist()
        cmd_vel.linear.x = front_vel_val
        cmd_vel.angular.z = angular_vel_val

        # publish calculated velocity
        self.left_vel_pub.publish(left_vel)
        self.right_vel_pub.publish(right_vel)
        self.cmd_vel_pub.publish(cmd_vel)



def main() -> None:
    rclpy.init()
    joy_controller = JoyController()

    try:
        rclpy.spin(joy_controller)
    except:
        joy_controller.get_logger().error(traceback.format_exc())
    finally:
        joy_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
