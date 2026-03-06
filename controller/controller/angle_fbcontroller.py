#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Twist
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import Float32
from tf_transformations import euler_from_quaternion


class AngFBController(Node):
    """Allocate velocity based on an error to a given target angle and a measured angular velocity"""

    def __init__(self) -> None:
        super().__init__("angle_fbcontroller")

        self.declare_parameter(
            "K", 5.0, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )
        self.declare_parameter(
            "init_forward_velocity", 0.26, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )

        self.K = float(self.get_parameter("K").value)
        self.forward_vel = float(self.get_parameter("init_forward_velocity").value)

        # curr states
        self.ref_phi = 0.0
        self.ref_phi_is_ready = False

        # pub
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # sub
        self.create_subscription(Pose, "curr_pose", self.pose_callback, 10)
        self.create_subscription(Float32, "target_angle", self.ref_phi_callback, 10)

    def pose_callback(self, msg: Pose) -> None:
        if not self.ref_phi_is_ready:
            return

        orientation = msg.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])

        err_phi = self.ref_phi - yaw
        if err_phi > np.pi:
            err_phi -= 2 * np.pi
        elif err_phi < -np.pi:
            err_phi += 2 * np.pi

        # proportional controller
        ref_angvel = self.K * err_phi
        cmd_vel = Twist()
        cmd_vel.angular.z = ref_angvel
        cmd_vel.linear.x = self.forward_vel
        self.cmd_vel_pub.publish(cmd_vel)

    def ref_phi_callback(self, msg: Float32) -> None:
        # target angle (chi_d) in LOS algorithm
        self.ref_phi = msg.data
        self.ref_phi_is_ready = True


def main() -> None:
    rclpy.init()
    node = AngFBController()

    try:
        rclpy.spin(node)
    except Exception:
        node.get_logger().error("Unexpected error", exc_info=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
