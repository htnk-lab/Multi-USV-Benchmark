#!/usr/bin/env python

import traceback

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Twist
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import Header
from tf_transformations import euler_from_quaternion, quaternion_from_euler


class KinematicAgent(Node):
    """Ideal mathmatical model"""

    def __init__(self) -> None:
        super().__init__("kinematic_agent")

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
            "agent_frame", "base", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter(
            "timer_period", 0.1, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )

        # get parameter
        # curr_pose初期化
        # parameterのinit_positionの要素数が足りない場合，対応するPointの要素は0.0で初期化される
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
        self.agent_frame = str(self.get_namespace() + "/" + self.get_parameter("agent_frame").value)
        self.dt = float(self.get_parameter("timer_period").value)

        self.v = 0.0
        self.omega = 0.0

        # pub
        self.pose_pub = self.create_publisher(PoseStamped, "pose", 10)
        self.curr_pose_pub = self.create_publisher(Pose, "curr_pose", 10)

        # sub
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 10)

        # timer
        self.create_timer(self.dt, self.timer_callback)

    def cmd_vel_callback(self, msg: Twist) -> None:
        # 並進速度/回転速度
        self.v = msg.linear.x
        self.omega = msg.angular.z

    def timer_callback(self) -> None:
        orientation = self.curr_pose.orientation
        _, _, yaw = euler_from_quaternion(quaternion=[orientation.x, orientation.y, orientation.z, orientation.w])

        # unicycle model
        position = self.curr_pose.position
        self.curr_pose.position = Point(
            x=position.x + self.dt * np.cos(yaw + self.dt * self.omega / 2) * self.v,
            y=position.y + self.dt * np.sin(yaw + self.dt * self.omega / 2) * self.v,
        )

        self.curr_pose.orientation = Quaternion(
            **dict(
                zip(
                    ["x", "y", "z", "w"],
                    quaternion_from_euler(ai=0.0, aj=0.0, ak=yaw + self.dt * self.omega),
                )
            )
        )

        curr_pose = self.curr_pose

        self.curr_pose_pub.publish(curr_pose)
        # for footprinter
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
    kinematic_agent = KinematicAgent()

    try:
        rclpy.spin(kinematic_agent)
    except:
        kinematic_agent.get_logger().error(traceback.format_exc())
    finally:
        kinematic_agent.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
