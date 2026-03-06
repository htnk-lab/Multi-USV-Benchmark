#!/usr/bin/env python3

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, Vector3
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import Float32, Header
from visualization_msgs.msg import Marker

from .viz_utils import color_list, get_color_rgba

# Z offset for visualization markers
MARKER_Z = 0.05


class LOS(Node):
    """Calculate reference angle based on Line-of-Sight Algorithm"""

    def __init__(self) -> None:
        super().__init__("los")

        self.declare_parameter("delta", 0.2, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter("agent_id", 0, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter(
            "world_frame", "world", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )

        self.delta = float(self.get_parameter("delta").value)
        self.agent_id = int(self.get_parameter("agent_id").value)
        self.world_frame = str(self.get_parameter("world_frame").value)

        # initialization
        self.x_position = 0.0
        self.y_position = 0.0
        self.pose_point = Point()

        # pub
        self.nav_fromto_pub = self.create_publisher(Marker, "nav_fromto", 10)
        self.nav_los_pub = self.create_publisher(Marker, "los", 10)
        self.target_angle_pub = self.create_publisher(Float32, "target_angle", 10)

        # sub
        self.create_subscription(Pose, "curr_pose", self.pose_callback, 10)
        self.create_subscription(PoseArray, "target_line", self.target_line_callback, 10)

    def pose_callback(self, msg: Pose) -> None:
        self.x_position = msg.position.x
        self.y_position = msg.position.y
        self.pose_point = Point(x=msg.position.x, y=msg.position.y, z=MARKER_Z)

    def target_line_callback(self, msg: PoseArray) -> None:
        nav_from = msg.poses[0].position
        nav_to = msg.poses[1].position

        nav_from.z = MARKER_Z
        nav_to.z = MARKER_Z

        # publish navigation line marker
        nav_fromto = Marker(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.world_frame),
            ns="nav_fromto",
            id=1,
            type=Marker.LINE_STRIP,
            action=Marker.ADD,
            points=[nav_from, nav_to],
            scale=Vector3(x=0.03),
            color=get_color_rgba(color_list[self.agent_id], alpha=0.9),
        )
        self.nav_fromto_pub.publish(nav_fromto)

        # LOS algorithm
        nav_dx = nav_to.x - nav_from.x
        nav_dy = nav_to.y - nav_from.y
        nav_angle = np.arctan2(nav_dy, nav_dx)
        nav_norm = np.hypot(nav_dx, nav_dy)

        # Rotation matrix (transpose = inverse for orthogonal)
        cos_a, sin_a = np.cos(nav_angle), np.sin(nav_angle)
        rot_t = np.array([[cos_a, sin_a], [-sin_a, cos_a]])

        # Project agent position onto path-tangential/normal frame
        perp = rot_t @ np.array([[self.x_position - nav_from.x], [self.y_position - nav_from.y]])

        nav_unit = np.array([[nav_dx], [nav_dy]]) / nav_norm
        los_position = (self.delta + perp[0, 0]) * nav_unit + np.array([[nav_from.x], [nav_from.y]])
        target_angle = nav_angle - np.arctan(perp[1, 0] / self.delta)

        los_point = Point(x=float(los_position[0, 0]), y=float(los_position[1, 0]), z=MARKER_Z)

        # publish LOS line marker
        nav_los = Marker(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.world_frame),
            ns="nav_los",
            id=3,
            type=Marker.LINE_STRIP,
            action=Marker.ADD,
            points=[los_point, self.pose_point],
            scale=Vector3(x=0.03, y=0.03, z=0.03),
            color=get_color_rgba(color_list[self.agent_id], alpha=0.9),
        )
        self.nav_los_pub.publish(nav_los)

        self.target_angle_pub.publish(Float32(data=float(target_angle)))


def main() -> None:
    rclpy.init()
    node = LOS()

    try:
        rclpy.spin(node)
    except Exception:
        node.get_logger().error("Unexpected error", exc_info=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
