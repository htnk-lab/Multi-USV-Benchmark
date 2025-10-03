#!/usr/bin/env python

import traceback

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import Header
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class IdealAgent(Node):
    """Ideal mathmatical model"""

    def __init__(self) -> None:
        super().__init__("ideal_agent")

        # declare parameter
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
        # get parameter
        # initialize curr_pose
        # if the number of elements in the parameter init_position is insufficient, the corresponding elements of Point are initialized to 0.0
        self.curr_pose = Pose(
            position=Point(**dict(zip(["x", "y", "z"], self.get_parameter("init_position").value))),
            orientation=Quaternion(
                **dict(
                    zip(
                        ["x", "y", "z", "w"],
                        quaternion_from_euler(ai=0.0, aj=0.0, ak=float(self.get_parameter("init_yaw").value)),
                    )
                )
            ),
        )
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.dt = float(self.get_parameter("dt").value)
        self.r_min = float(self.get_parameter("r_min").value)

        self.v = 0.0
        self.omega = 0.0
        self.z = 0.08  # offset to display robot model above water surface

        # pub
        self.pose_pub = self.create_publisher(PoseStamped, "pose", 10)

        # sub
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        # timer
        self.create_timer(self.dt, self.timer_callback)

    def cmd_vel_callback(self, msg: Twist) -> None:
        # linear and angular velocity
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
            z=self.z,
        )

        self.max_omega = self.v/self.r_min
        self.omega = np.clip(self.omega, -self.max_omega, self.max_omega)

        self.curr_pose.orientation = Quaternion(
            **dict(
                zip(
                    ["x", "y", "z", "w"],
                    quaternion_from_euler(ai=0.0, aj=0.0, ak=yaw + self.dt * self.omega),
                )
            )
        )

        curr_pose = self.curr_pose
        # publish
        self.pose_pub.publish(
            PoseStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=self.world_frame,
                ),
                pose=curr_pose,
            )
        )

def main() -> None:
    rclpy.init()
    ideal_agent = IdealAgent()

    try:
        rclpy.spin(ideal_agent)
    except:
        ideal_agent.get_logger().error(traceback.format_exc())
    finally:
        ideal_agent.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
