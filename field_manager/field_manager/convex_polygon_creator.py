#!/usr/bin/env python

from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import Point32, PolygonStamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node


class ConvexPolygonCreator(Node):
    """Divide the field into convex regions for each agent"""

    def __init__(self) -> None:
        super().__init__("convex_polygon_creator")

        self.declare_parameter("agent_num", 2, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter(
            "x_limit", [0.0, 10.0], descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        )
        self.declare_parameter(
            "y_limit", [0.0, 10.0], descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        )
        self.declare_parameter(
            "agent_prefix", "agent", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter(
            "world_frame", "world", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter(
            "timer_period", 1.0, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )

        self.agent_num: int = self.get_parameter("agent_num").value
        self.x_limit: List[float] = self.get_parameter("x_limit").value
        self.y_limit: List[float] = self.get_parameter("y_limit").value
        agent_prefix: str = self.get_parameter("agent_prefix").value
        self.world_frame: str = self.get_parameter("world_frame").value
        timer_period: float = self.get_parameter("timer_period").value

        self.polygon_pub_list = [
            self.create_publisher(PolygonStamped, f"{agent_prefix}{i}/field_range", 10)
            for i in range(self.agent_num)
        ]

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self) -> None:
        x_length = self.x_limit[1] - self.x_limit[0]
        for agent_id in range(self.agent_num):
            part_x = [
                self.x_limit[0] + x_length * agent_id / self.agent_num,
                self.x_limit[0] + x_length * (agent_id + 1) / self.agent_num,
            ]
            points = np.array(
                [
                    [part_x[0], self.y_limit[0], 0.0],
                    [part_x[0], self.y_limit[1], 0.0],
                    [part_x[1], self.y_limit[1], 0.0],
                    [part_x[1], self.y_limit[0], 0.0],
                ]
            )
            if points.shape[0] > 2 and not self.is_convex(points):
                points = np.zeros((0, 3))
            self.publish_polygon(points, agent_id)

    def is_convex(self, points: np.ndarray) -> bool:
        if points.shape[0] < 3:
            return True
        n = points.shape[0]
        cross_signs = 0
        for i in range(n):
            a = points[i]
            b = points[(i + 1) % n]
            c = points[(i + 2) % n]
            cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
            cross_signs += 1 if cross < 0 else -1
        return abs(cross_signs) == n

    def publish_polygon(self, points: np.ndarray, agent_id: int) -> None:
        polygon_msg = PolygonStamped()
        polygon_msg.header.frame_id = self.world_frame
        for point in points:
            polygon_msg.polygon.points.append(Point32(x=point[0], y=point[1], z=point[2]))
        self.polygon_pub_list[agent_id].publish(polygon_msg)


def main() -> None:
    rclpy.init()
    node = ConvexPolygonCreator()

    try:
        rclpy.spin(node)
    except Exception:
        node.get_logger().error("Unexpected error", exc_info=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
