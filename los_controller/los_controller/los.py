#!/usr/bin/env python3
import traceback

import numpy as np
import rclpy
from field_manager.coverage_utils.utils import color_list, get_color_rgba
from geometry_msgs.msg import Point, Pose, PoseArray, Vector3
from rclpy.node import Node
from std_msgs.msg import Float32, Header
from visualization_msgs.msg import Marker


class LOS(Node):
    """Calculate reference angle based on Line-of-Sight Algorithm"""

    def __init__(self):
        super().__init__("los")

        # declare parameter
        self.declare_parameters(
            namespace="",
            parameters=[
                ("delta_max", 0.8),
                ("delta_min", 0.2),
                ("agent_id", 0),
            ],
        )
        self.delta_max: float = self.get_parameter("delta_max").get_parameter_value().double_value
        self.delta_min: float = self.get_parameter("delta_min").get_parameter_value().double_value
        self.agent_id: int = self.get_parameter("agent_id").get_parameter_value().integer_value

        # initialization
        self.x_position, self.y_position = 1.0, 0.2
        self.start = self.get_clock().now().nanoseconds
        self.target_angle = Float32()
        self.pose_point = Point()

        # pub
        self.nav_fromto_pub = self.create_publisher(Marker, "nav_fromto", 10)
        self.nav_los_pub = self.create_publisher(Marker, "los", 10)
        self.khi_d_pub = self.create_publisher(Float32, "target_angle", 10)

        # sub
        self.create_subscription(Pose, "curr_pose", self.pose_callback, 10)
        self.create_subscription(PoseArray, "target_line", self.target_line_callback, 10)

    def pose_callback(self, msg: Pose) -> None:
        self.x_position = msg.position.x
        self.y_position = msg.position.y
        self.pose_point = Point(x=msg.position.x, y=msg.position.y, z=0.05)

    def target_line_callback(self, msg: PoseArray) -> None:
        nav_from = msg.poses[0].position
        nav_to = msg.poses[1].position

        # visualization
        nav_from.z = 0.05
        nav_to.z = 0.05
        nav_fromto = Marker(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id="world"),
            ns="nav_fromto",
            id=1,
            type=Marker.LINE_STRIP,
            action=Marker.ADD,
            points=[nav_from, nav_to],
            scale=Vector3(x=0.03),
            color=get_color_rgba(color_list[self.agent_id], alpha=0.9),
        )
        self.nav_fromto_pub.publish(nav_fromto)

        nav_vec = np.matrix([[nav_to.x - nav_from.x], [nav_to.y - nav_from.y]])
        nav_angle = np.arctan2((nav_to.y - nav_from.y), (nav_to.x - nav_from.x))
        perp = np.matrix.transpose(
            np.matrix(
                [
                    [np.cos(nav_angle), -1 * np.sin(nav_angle)],
                    [np.sin(nav_angle), np.cos(nav_angle)],
                ]
            )
        ) @ np.matrix([[self.x_position - nav_from.x], [self.y_position - nav_from.y]])
        delta = (self.delta_max - self.delta_min) * np.exp(-2 * abs(perp[1, 0])) + self.delta_min
        los_position = (delta + perp[0, 0]) * (nav_vec / np.linalg.norm(nav_vec)) + np.matrix(
            [[nav_from.x], [nav_from.y]]
        )
        target_angle = nav_angle - np.arctan(perp[1, 0] / delta)
        los_point = Point(x=los_position[0, 0], y=los_position[1, 0], z=0.0)

        # publish navigation points and target angle

        los_point.z = 0.05

        nav_los = Marker(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id="world"),
            ns="nav_los",
            id=3,
            type=Marker.LINE_STRIP,
            action=Marker.ADD,
            points=[los_point, self.pose_point],
            scale=Vector3(x=0.03, y=0.03, z=0.03),
            color=get_color_rgba(color_list[self.agent_id], alpha=0.9),
        )

        self.nav_los_pub.publish(nav_los)
        self.target_angle.data = target_angle
        self.khi_d_pub.publish(self.target_angle)


def main() -> None:
    rclpy.init()
    los = LOS()

    try:
        rclpy.spin(los)
    except:
        los.get_logger().error(traceback.format_exc())
    finally:
        los.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
