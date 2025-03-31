#!/usr/bin/env python3

import math
import traceback
from collections import deque

import numpy as np
import rclpy
import tf_transformations as tf_trans
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from rclpy.node import Node
from std_msgs.msg import Float32, Header


class VirtualAgent(Node):
    """Publish a virtual agent under a system identified 1st order dead time model"""

    def __init__(self) -> None:
        super().__init__("identified_agent")

        # declare parameter
        self.declare_parameters(
            namespace="",
            parameters=[
                ("timer_period", 0.02),
                ("init_position", [0.5, 1.7]),
                ("init_yaw", 0.0),
                ("agent_frame", "base"),
                ("init_forward_velocity", 0.26),
            ],
        )

        # get parameter
        dead_time = 0.02
        self.timer_period = self.get_parameter("timer_period").get_parameter_value().double_value
        self.max_dead_hold = int(dead_time // self.timer_period)
        self.held_inputs = deque(maxlen=self.max_dead_hold)

        self.x, self.y = self.get_parameter("init_position").get_parameter_value().double_array_value
        self.yaw = self.get_parameter("init_yaw").get_parameter_value().double_value
        self.forward_velocity: float = self.get_parameter("init_forward_velocity").get_parameter_value().double_value

        self.agent_frame = str(
            self.get_namespace() + "/" + self.get_parameter("agent_frame").get_parameter_value().string_value
        )
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
        time_now = self.get_clock().now().nanoseconds * (10 ** (-9))
        delta_time = time_now - self.time_past

        vx = -1 * (self.left_vel / 1.6 * self.forward_velocity + self.right_vel / 1.6 * self.forward_velocity)
        vy = 0.0

        # G(s) = e^(-0.01)-10/(s+2.1)
        command_input = -1 * (self.left_vel - self.right_vel) / 2
        state_update = (-1.8 * self.state_of_omega + self.past_input) * delta_time
        self.wz = -9.7 * self.state_of_omega
        self.state_of_omega += state_update
        self.past_input = command_input

        v = np.array(
            [[math.cos(self.yaw), -1 * math.sin(self.yaw)], [math.sin(self.yaw), math.cos(self.yaw)]]
        ) @ np.matrix([[vx], [vy]])
        self.x = self.x + v[0, 0] * delta_time
        self.y = self.y + v[1, 0] * delta_time
        self.yaw = self.yaw + self.wz * delta_time
        self.time_past = time_now

        qx, qy, qz, qw = tf_trans.quaternion_from_euler(0.0, 0.0, self.yaw)

        robot_curr_pose = Pose(
            position=Point(x=self.x, y=self.y, z=self.z),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
        )

        robot_pose = PoseStamped(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id="world"),
            pose=robot_curr_pose,
        )
        # self.get_logger().info(f"{delta_time}")

        # publish calculated velocity
        self.robot_pose_pub.publish(robot_pose)
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
