#!/usr/bin/env python3

import math
import traceback
from collections import deque

import numpy as np
import rclpy
import tf_transformations as tf_trans
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import Float32, Header

from tf_transformations import euler_from_quaternion, quaternion_from_euler

class VirtualAgent(Node):
    """Publish a virtual agent under a system identified 1st order dead time model"""

    def __init__(self) -> None:
        super().__init__("identified_agent")

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
            "timer_period", 0.02, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )
        self.declare_parameter(
            "init_forward_velocity", 0.26, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
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
        self.timer_period = float(self.get_parameter("timer_period").value)

        dead_time = 0.02
        self.max_dead_hold = int(dead_time // self.timer_period)
        self.held_inputs = deque(maxlen=self.max_dead_hold)
        self.forward_velocity = float(self.get_parameter("init_forward_velocity").value)

        # initialize
        self.time_past = self.get_clock().now().nanoseconds * (10 ** (-9))
        self.z = 0.08  # karugamotモデルを水上へ表示するためのオフセット
        self.state_of_omega = 0.0
        self.past_input = 0.0
        self.left_vel, self.right_vel = 0.0, 0.0

        # pub
        self.robot_pose_pub = self.create_publisher(PoseStamped, "pose", 10)
        self.debug_float_pub = self.create_publisher(Float32, "debug_float", 10)

        # sub
        self.create_subscription(Float32, "left/vel", self.left_vel_callback, 10)
        self.create_subscription(Float32, "right/vel", self.right_vel_callback, 10)

        # timer
        self.create_timer(0.02, self.timer_callback)

    def left_vel_callback(self, msg: Float32) -> None:
        self.left_vel = msg.data

    def right_vel_callback(self, msg: Float32) -> None:
        self.right_vel = msg.data

    def timer_callback(self) -> None:
        orientation = self.curr_pose.orientation
        _, _, yaw = euler_from_quaternion(quaternion=[orientation.x, orientation.y, orientation.z, orientation.w])

        time_now = self.get_clock().now().nanoseconds * (10 ** (-9))
        delta_time = time_now - self.time_past

        vx = -1 * (self.left_vel / 1.6 * self.forward_velocity + self.right_vel / 1.6 * self.forward_velocity)
        vy = 0.0

        # G(s) = e^(-0.01)-10/(s+2.1)
        command_input = -1 * (self.left_vel - self.right_vel) / 2
        state_update = (-1.8 * self.state_of_omega + self.past_input) * delta_time
        wz = -9.7 * self.state_of_omega
        self.state_of_omega += state_update
        self.past_input = command_input

        v = np.array(
            [[math.cos(yaw), -1 * math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]]
        ) @ np.matrix([[vx], [vy]])

        position = self.curr_pose.position
        self.curr_pose.position = Point(
            x=position.x + v[0, 0] * delta_time,
            y=position.y + v[1, 0] * delta_time,
            z=self.z,
        )

        self.curr_pose.orientation = Quaternion(
            **dict(
                zip(
                    ["x", "y", "z", "w"],
                    quaternion_from_euler(ai=0.0, aj=0.0, ak=yaw + wz * delta_time),
                )
            )
        )
        self.time_past = time_now

        curr_pose = self.curr_pose
        # publish
        self.robot_pose_pub.publish(
            PoseStamped(
                header=Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=self.world_frame,
                ),
                pose=curr_pose,
            )
        )
        debug_time = self.get_clock().now().nanoseconds * (10 ** (-9)) - time_now
        self.debug_float_pub.publish(Float32(data=debug_time))


def main() -> None:
    rclpy.init()
    identified_agent = VirtualAgent()

    try:
        rclpy.spin(identified_agent)
    except:
        identified_agent.get_logger().error(traceback.format_exc())
    finally:
        identified_agent.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
