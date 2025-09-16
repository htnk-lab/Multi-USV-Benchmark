#!/usr/bin/env python3
import traceback
from typing import Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Twist
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion


class AngFBController(Node):
    """Allocate velocity based on an error to a given target angle and a measured angular velocity"""

    def __init__(self):
        super().__init__("angle_fbcontroller")

        # declare parameter
        self.declare_parameter(
            "K", 5.0, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )
        self.declare_parameter(
            "init_forward_velocity", 0.26, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )
        
        self.K = float(self.get_parameter("K").value)
        self.forward_vel = float(self.get_parameter("init_forward_velocity").value)

        # initialization
        # curr states
        self.ref_phi = 0.0

        # commands
        self.ref_phi_is_ready = False

        # pub
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # sub
        self.create_subscription(Pose, "curr_pose", self.pose_callback, 10)
        self.create_subscription(Float32, "target_angle", self.ref_phi_callback, 10)

    def pose_callback(self, msg: Pose) -> None:
        curr_pose = msg

        orientation = curr_pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        # allocate velocity
        if self.ref_phi_is_ready:
            err_phi = self.ref_phi - yaw
            if err_phi > np.pi:
                err_phi = err_phi - 2 * np.pi
            elif err_phi < -np.pi:
                err_phi = err_phi + 2 * np.pi

            ref_angvel = self.K * err_phi  # proportional controller
            # publish calculated velocity
            cmd_vel = Twist()
            cmd_vel.angular.z = ref_angvel
            cmd_vel.linear.x = self.forward_vel
            self.cmd_vel_pub.publish(cmd_vel)

    def ref_phi_callback(self, msg: Float32):
        self.ref_phi = msg.data  # ref_phi is a target angle (chi_d) in LOS algorithm (in the book)
        self.ref_phi_is_ready = True

def main() -> None:
    rclpy.init()
    angle_fbcontroller = AngFBController()

    try:
        rclpy.spin(angle_fbcontroller)
    except:
        angle_fbcontroller.get_logger().error(traceback.format_exc())
    finally:
        angle_fbcontroller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
