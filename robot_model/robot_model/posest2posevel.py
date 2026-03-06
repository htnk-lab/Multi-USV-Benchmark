#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped, Transform, TransformStamped, Twist, Vector3
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Header
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf_transformations import euler_from_quaternion

WATER_SURFACE_Z_OFFSET = 0.08


class PoseST2PoseVel(Node):
    """Convert PoseStamped to Pose for position and Twist for velocity"""

    def __init__(self) -> None:
        super().__init__("posest2posevel")
        # init_forward_velocity must be nonzero to compute target angular velocity
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
        self.declare_parameter(
            "ema_filter_param", 0.9, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )

        init_forward_velocity = float(self.get_parameter("init_forward_velocity").value)
        init_omega = float(self.get_parameter("init_omega").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.agent_frame = str(self.get_namespace() + "/" + self.get_parameter("agent_frame").value)
        self.filter_par = float(self.get_parameter("ema_filter_param").value)

        # init prev states
        self.prev_time = self.get_clock().now().nanoseconds * 1e-9
        self.prev_x_position = 0.0
        self.prev_y_position = 0.0
        self.prev_vrx = init_forward_velocity
        self.prev_vry = 0.0
        self.prev_yaw = 0.0
        self.prev_omega = init_omega

        # tf2
        self.broadcaster = TransformBroadcaster(self)

        # pub
        self.curr_pose_pub = self.create_publisher(Pose, "curr_pose", 10)
        self.curr_vel_pub = self.create_publisher(Twist, "curr_vel", 10)

        # sub: "pose" topic is derived from Motive
        self.create_subscription(PoseStamped, "pose", self.curr_posest_callback, 10)

    def curr_posest_callback(self, msg: PoseStamped) -> None:
        curr_position = msg.pose.position
        orientation = msg.pose.orientation
        _, _, curr_yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        curr_time = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

        # finite differences
        delta_time = float(curr_time - self.prev_time)
        delta_yaw = curr_yaw - self.prev_yaw

        # Handle yaw wrapping
        if delta_yaw < -3 * np.pi / 2:
            omega = (delta_yaw + 2 * np.pi) / delta_time
            yaw_h = (curr_yaw + self.prev_yaw - 2 * np.pi) / 2
        elif delta_yaw > 3 * np.pi / 2:
            omega = (delta_yaw - 2 * np.pi) / delta_time
            yaw_h = (curr_yaw + self.prev_yaw + 2 * np.pi) / 2
        else:
            omega = delta_yaw / delta_time
            yaw_h = (curr_yaw + self.prev_yaw) / 2

        v_wx = (curr_position.x - self.prev_x_position) / delta_time
        v_wy = (curr_position.y - self.prev_y_position) / delta_time

        # Transform world velocity to robot frame
        rot_matrix = np.array([
            [np.cos(yaw_h), np.sin(yaw_h)],
            [-np.sin(yaw_h), np.cos(yaw_h)],
        ])
        v_r = rot_matrix @ np.array([[v_wx], [v_wy]])

        # EMA filtering
        curr_vrx = self._ema_filter(v_r[0, 0], self.prev_vrx)
        curr_vry = self._ema_filter(v_r[1, 0], self.prev_vry)
        curr_omega = omega

        # update previous states
        self.prev_time = curr_time
        self.prev_x_position = curr_position.x
        self.prev_y_position = curr_position.y
        self.prev_yaw = curr_yaw
        self.prev_vrx = curr_vrx
        self.prev_vry = curr_vry
        self.prev_omega = curr_omega

        # broadcast tf for visualization
        transform_stamped = TransformStamped(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.world_frame),
            child_frame_id=self.agent_frame,
            transform=Transform(
                translation=Vector3(x=curr_position.x, y=curr_position.y, z=WATER_SURFACE_Z_OFFSET),
                rotation=orientation,
            ),
        )
        self.broadcaster.sendTransform(transform_stamped)

        self.curr_pose_pub.publish(msg.pose)
        self.curr_vel_pub.publish(
            Twist(
                linear=Vector3(x=curr_vrx, y=curr_vry, z=0.0),
                angular=Vector3(x=0.0, y=0.0, z=curr_omega),
            )
        )

    def _ema_filter(self, curr_data: float, prev_data: float) -> float:
        return self.filter_par * curr_data + (1 - self.filter_par) * prev_data


def main() -> None:
    rclpy.init()
    node = PoseST2PoseVel()

    try:
        rclpy.spin(node)
    except Exception:
        node.get_logger().error("Unexpected error", exc_info=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
