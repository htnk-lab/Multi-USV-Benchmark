#!/usr/bin/env python

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import Header
from tf_transformations import euler_from_quaternion, quaternion_from_euler


# Offset to display robot model above water surface
WATER_SURFACE_Z_OFFSET = 0.08


class IdealAgent(Node):
    """Ideal mathematical model (Dubins car)"""

    def __init__(self) -> None:
        super().__init__("ideal_agent")

        self.declare_parameter(
            "init_position",
            descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY),
        )
        self.declare_parameter("init_yaw", 0.0, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter(
            "world_frame", "world", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter(
            "dt", 0.1, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )
        self.declare_parameter(
            "r_min", 0.2, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE),
        )

        # init_position maps to Point fields; missing elements default to 0.0
        init_position = self.get_parameter("init_position").value
        init_yaw = float(self.get_parameter("init_yaw").value)
        qx, qy, qz, qw = quaternion_from_euler(ai=0.0, aj=0.0, ak=init_yaw)

        self.curr_pose = Pose(
            position=Point(**dict(zip(["x", "y", "z"], init_position))),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
        )
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.dt = float(self.get_parameter("dt").value)
        self.r_min = float(self.get_parameter("r_min").value)

        self.v = 0.0
        self.omega = 0.0

        # pub
        self.pose_pub = self.create_publisher(PoseStamped, "pose", 10)

        # sub
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        # timer
        self.create_timer(self.dt, self.timer_callback)

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def timer_callback(self) -> None:
        orientation = self.curr_pose.orientation
        _, _, yaw = euler_from_quaternion(quaternion=[orientation.x, orientation.y, orientation.z, orientation.w])

        # Dubins car model
        position = self.curr_pose.position
        self.curr_pose.position = Point(
            x=position.x + self.dt * np.cos(yaw) * self.v,
            y=position.y + self.dt * np.sin(yaw) * self.v,
            z=WATER_SURFACE_Z_OFFSET,
        )

        max_omega = self.v / self.r_min
        self.omega = float(np.clip(self.omega, -max_omega, max_omega))

        qx, qy, qz, qw = quaternion_from_euler(ai=0.0, aj=0.0, ak=yaw + self.dt * self.omega)
        self.curr_pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        self.pose_pub.publish(
            PoseStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=self.world_frame,
                ),
                pose=self.curr_pose,
            )
        )


def main() -> None:
    rclpy.init()
    node = IdealAgent()

    try:
        rclpy.spin(node)
    except Exception:
        node.get_logger().error("Unexpected error", exc_info=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
