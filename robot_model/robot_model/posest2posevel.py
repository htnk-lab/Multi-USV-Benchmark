#!/usr/bin/env python3

import traceback

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped, Twist, Vector3
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf_transformations import euler_from_quaternion


class PoseST2PoseVel(Node):
    """convert PoseStamped to Pose for position and Twist for velocity"""

    def __init__(self) -> None:
        super().__init__("posest2posevel")
        # init states
        """Linear forward velocity initial assumption should not be zero to calculate target angular velocity"""
        self.declare_parameter(
            "init_forward_velocity", 0.26, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )
        self.declare_parameter("init_omega", 0.0, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter(
            "world_frame", "world", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter(
            "agent_frame", "base", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )

        init_forward_velocity = float(self.get_parameter("init_forward_velocity").value)
        init_omega = float(self.get_parameter("init_omega").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.agent_frame = str(self.get_namespace() + "/" + self.get_parameter("agent_frame").value)

        # filter param
        self.filter_par = 0.9

        # init prev states
        self.prev_time = self.get_clock().now().nanoseconds * (10 ** (-9))
        self.prev_x_position, self.prev_y_position = 0.0, 0.0
        self.prev_vrx, self.prev_vry = init_forward_velocity, 0.0
        self.prev_yaw = 0.0
        self.prev_omega = init_omega
        self.prev_err_omega = 0.0

        # tf2
        self.broadcaster = TransformBroadcaster(self)

        # pub
        self.curr_pose_pub = self.create_publisher(Pose, "curr_pose", 10)
        self.curr_vel_pub = self.create_publisher(Twist, "curr_vel", 10)

        # sub
        # topic名"pose"はmotive由来
        self.create_subscription(PoseStamped, "pose", self.curr_posest_callback, 10)

    def curr_posest_callback(self, msg: PoseStamped) -> None:
        # current state
        curr_posest = msg
        curr_position = curr_posest.pose.position
        orientation = curr_posest.pose.orientation
        _, _, curr_yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        curr_time = Time.from_msg(curr_posest.header.stamp).nanoseconds * (10 ** (-9))

        # cal raw data
        delta_time = float(curr_time - self.prev_time)
        delta_yaw = curr_yaw - self.prev_yaw

        if delta_yaw < -3 * np.pi / 2:
            omega = (delta_yaw + (2 * np.pi)) / delta_time
            yaw_h = (curr_yaw + self.prev_yaw - (2 * np.pi)) / 2
        elif delta_yaw > 3 * np.pi / 2:
            omega = (delta_yaw - (2 * np.pi)) / delta_time
            yaw_h = (curr_yaw + self.prev_yaw + (2 * np.pi)) / 2
        else:
            omega = (delta_yaw) / delta_time
            yaw_h = (curr_yaw + self.prev_yaw) / 2

        v_wx = (curr_position.x - self.prev_x_position) / delta_time
        v_wy = (curr_position.y - self.prev_y_position) / delta_time
        v_r = np.array([[np.cos(yaw_h), np.sin(yaw_h)], [-np.sin(yaw_h), np.cos(yaw_h)]]) @ np.array([[v_wx], [v_wy]])

        # filtering
        curr_vrx = self.EMA_filter(v_r[0, 0], self.prev_vrx, self.filter_par)
        curr_vry = self.EMA_filter(v_r[1, 0], self.prev_vry, self.filter_par)
        # curr_omega = self.EMA_filter(omega, self.prev_omega, self.filter_par)
        curr_omega = omega

        # put current states as prev states
        self.prev_time = curr_time
        self.prev_x_position, self.prev_y_position = curr_position.x, curr_position.y
        self.prev_yaw = curr_yaw
        self.prev_vrx = curr_vrx
        self.prev_vry = curr_vry
        self.prev_omega = curr_omega

        # 描画用に現在位置/姿勢をtf形式で送信
        transform_stamped = TransformStamped(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.world_frame),
            child_frame_id=self.agent_frame,
            transform=Transform(
                translation=Vector3(
                    x=curr_position.x,
                    y=curr_position.y,
                    z=0.08,
                ),
                rotation=orientation,
            ),
        )
        self.broadcaster.sendTransform(transform_stamped)

        # pub
        self.curr_pose_pub.publish(curr_posest.pose)
        curr_twist = Twist(linear=Vector3(x=curr_vrx, y=curr_vry, z=0.0), angular=Vector3(x=0.0, y=0.0, z=curr_omega))
        self.curr_vel_pub.publish(curr_twist)

    def EMA_filter(self, curr_data, prev_data, filter_par) -> float:
        filtered_data = filter_par * curr_data + (1 - filter_par) * prev_data
        return filtered_data


def main() -> None:
    rclpy.init()
    posest2posevel = PoseST2PoseVel()

    try:
        rclpy.spin(posest2posevel)
    except:
        posest2posevel.get_logger().error(traceback.format_exc())
    finally:
        posest2posevel.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
