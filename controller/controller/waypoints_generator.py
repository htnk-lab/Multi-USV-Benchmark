#!/usr/bin/env python3

from typing import List

import numpy as np
import rclpy
from dubins import path_sample
from geometry_msgs.msg import Point, PolygonStamped, Pose, PoseArray, Vector3
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import Float32, Header
from visualization_msgs.msg import Marker

from .viz_utils import color_list, get_color_rgba

# Z offset for waypoint visualization
WAYPOINT_Z = 0.05


class WayPointsGenerator(Node):
    """Generate preset way-points in lawn-mower pattern"""

    def __init__(self) -> None:
        super().__init__("waypoints_generator")

        self.declare_parameter("R_switch", 0.3, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter("r_min", 0.2, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter("bandwidth", 0.2, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter("agent_id", 1, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter("agent_num", 1, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter(
            "world_frame", "world", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter(
            "dubins_step_size", 0.2, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )

        self.r_switch = float(self.get_parameter("R_switch").value)
        self.band_width = float(self.get_parameter("bandwidth").value)
        self.r_min = float(self.get_parameter("r_min").value)
        self.agent_id = int(self.get_parameter("agent_id").value)
        self.agent_num = int(self.get_parameter("agent_num").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.dubins_step_size = float(self.get_parameter("dubins_step_size").value)

        if self.agent_num == 1:
            self.agent_id = 1

        # initialization
        self.target_line = PoseArray(poses=[Pose(), Pose()])
        self.wp_index = -1
        self.wp_index_next = 0
        self.is_ready_path = False
        self.way_points: List[Point] = []
        self.n_points = 0
        self.wp_markers = Marker()

        # sub
        self.create_subscription(PolygonStamped, "field_range", self.polygon_callback, 10)
        self.create_subscription(Pose, "curr_pose", self.pose_callback, 10)

        # pub
        self.wp_markers_pub = self.create_publisher(Marker, "way_points", 10)
        self.los_distance_pub = self.create_publisher(Float32, "los_distance", 10)
        self.target_line_pub = self.create_publisher(PoseArray, "target_line", 10)

    def pose_callback(self, msg: Pose) -> None:
        if not self.is_ready_path:
            return

        x_position = msg.position.x
        y_position = msg.position.y

        # find initial way-point index
        if self.wp_index == -1:
            min_distance = float("inf")
            for i, wp in enumerate(self.way_points):
                dist = np.hypot(wp.x - x_position, wp.y - y_position)
                if dist < min_distance:
                    min_distance = dist
                    self.wp_index = i
            self.wp_index_next = (self.wp_index + 1) % self.n_points

        # determine next step target line
        nav_from = self.way_points[self.wp_index]
        nav_to = self.way_points[self.wp_index_next]
        self.target_line.poses[0].position = nav_from
        self.target_line.poses[1].position = nav_to

        # check if waypoint switch is needed
        los_distance = float(np.hypot(nav_to.x - x_position, nav_to.y - y_position))
        if los_distance < self.r_switch:
            self.wp_index = (self.wp_index + 1) % self.n_points
            self.wp_index_next = (self.wp_index + 1) % self.n_points

        self.target_line_pub.publish(self.target_line)
        self.los_distance_pub.publish(Float32(data=los_distance))
        self.wp_markers_pub.publish(self.wp_markers)

    def polygon_callback(self, msg: PolygonStamped) -> None:
        polygon_points = np.array([[p.x, p.y, 0.0] for p in msg.polygon.points])

        zigzag_points = self._create_zigzag_waypoint(polygon_points, self.band_width)
        raw_wo = [Point(x=pt[0], y=pt[1], z=WAYPOINT_Z) for pt in zigzag_points]
        base_rot_angle = np.arctan2(raw_wo[1].y - raw_wo[0].y, raw_wo[1].x - raw_wo[0].x)

        # Interpolate waypoints with Dubins path
        dubins_path: List[tuple] = []
        turning_radius = self.r_min
        mod_distance = turning_radius + 0.1

        for index in range(len(raw_wo)):
            case_index = index % 4
            i = -1 if index == len(raw_wo) - 1 else index

            if case_index == 0:
                q0 = (
                    raw_wo[i].x + mod_distance * np.cos(base_rot_angle),
                    raw_wo[i].y + mod_distance * np.sin(base_rot_angle),
                    base_rot_angle,
                )
                q1 = (
                    raw_wo[i + 1].x - mod_distance * np.cos(base_rot_angle),
                    raw_wo[i + 1].y - mod_distance * np.sin(base_rot_angle),
                    base_rot_angle,
                )
            elif case_index == 1:
                q0 = (
                    raw_wo[i].x - mod_distance * np.cos(base_rot_angle),
                    raw_wo[i].y - mod_distance * np.sin(base_rot_angle),
                    base_rot_angle,
                )
                q1 = (
                    raw_wo[i + 1].x + mod_distance * np.cos(base_rot_angle - np.pi),
                    raw_wo[i + 1].y + mod_distance * np.sin(base_rot_angle - np.pi),
                    base_rot_angle - np.pi,
                )
            elif case_index == 2:
                q0 = (
                    raw_wo[i].x + mod_distance * np.cos(base_rot_angle - np.pi),
                    raw_wo[i].y + mod_distance * np.sin(base_rot_angle - np.pi),
                    base_rot_angle - np.pi,
                )
                q1 = (
                    raw_wo[i + 1].x - mod_distance * np.cos(base_rot_angle - np.pi),
                    raw_wo[i + 1].y - mod_distance * np.sin(base_rot_angle - np.pi),
                    base_rot_angle - np.pi,
                )
            else:
                q0 = (
                    raw_wo[i].x - mod_distance * np.cos(base_rot_angle - np.pi),
                    raw_wo[i].y - mod_distance * np.sin(base_rot_angle - np.pi),
                    base_rot_angle - np.pi,
                )
                q1 = (
                    raw_wo[i + 1].x + mod_distance * np.cos(base_rot_angle),
                    raw_wo[i + 1].y + mod_distance * np.sin(base_rot_angle),
                    base_rot_angle,
                )

            # Last waypoint connects back to first
            if index == len(raw_wo) - 1:
                q1 = (
                    raw_wo[0].x + mod_distance * np.cos(base_rot_angle),
                    raw_wo[0].y + mod_distance * np.sin(base_rot_angle),
                    base_rot_angle,
                )

            path = path_sample(q0, q1, turning_radius, self.dubins_step_size)
            dubins_path.extend(path[0])

        self.way_points = [Point(x=pt[0], y=pt[1], z=WAYPOINT_Z) for pt in dubins_path]
        self.n_points = len(self.way_points)

        self.wp_markers = Marker(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=self.world_frame),
            ns="s_shaped",
            id=1,
            type=Marker.LINE_STRIP,
            action=Marker.ADD,
            points=self.way_points,
            scale=Vector3(x=0.03),
            color=get_color_rgba(color_list[self.agent_id], alpha=1.0),
        )
        self.is_ready_path = True

    def _create_zigzag_waypoint(self, points: np.ndarray, width: float) -> np.ndarray:
        if points.shape[0] < 3:
            return np.zeros((0, 2))

        # sort the points in counter-clockwise order
        center = points.mean(axis=0)
        points = points[np.argsort(np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0]))]

        # align shortest side first
        min_len = np.argmin(np.sum((points - np.roll(points, -1, axis=0)) ** 2, axis=1))
        points = np.roll(points, -min_len, axis=0)

        # rotate the polygon so that the first edge is aligned with x-axis
        theta = np.arctan2(points[1, 1] - points[0, 1], points[1, 0] - points[0, 0])
        rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        normalized_points = np.dot(rot.T, (points - points[0, :])[:, :2].T).T
        normalized_points[1, 1] = 0.0
        next_point = np.roll(normalized_points, -1, axis=0)

        # create waypoints along zigzag pattern
        y_values = np.arange(
            np.min(normalized_points[:, 1]), np.max(normalized_points[:, 1]), width * 2
        ).reshape(1, -1)
        waypoints = normalized_points[0:1, :]
        for i in range(1, normalized_points.shape[0]):
            if normalized_points[i, 1] <= next_point[i, 1]:
                y = y_values[(y_values >= normalized_points[i, 1]) & (y_values < next_point[i, 1])]
            else:
                y = y_values[(y_values <= normalized_points[i, 1]) & (y_values > next_point[i, 1])]
            for y_ in y:
                x = normalized_points[i, 0] + (y_ - normalized_points[i, 1]) / (
                    next_point[i, 1] - normalized_points[i, 1]
                ) * (next_point[i, 0] - normalized_points[i, 0])
                waypoints = np.append(waypoints, [[x, y_]], axis=0)

        # sort the waypoints in zigzag order
        buffer = waypoints[np.argsort(waypoints[:, 1])]

        odd = True
        for i in range(waypoints.shape[0] // 2):
            if odd:
                waypoints[2 * i], waypoints[2 * i + 1] = min(
                    buffer[2 * i], buffer[2 * i + 1], key=lambda p: p[0]
                ), max(buffer[2 * i], buffer[2 * i + 1], key=lambda p: p[0])
            else:
                waypoints[2 * i], waypoints[2 * i + 1] = max(
                    buffer[2 * i], buffer[2 * i + 1], key=lambda p: p[0]
                ), min(buffer[2 * i], buffer[2 * i + 1], key=lambda p: p[0])
            odd = not odd

        # rotate the waypoints back to original frame
        waypoints = np.dot(rot, waypoints.T).T + points[0, :2]

        return waypoints


def main() -> None:
    rclpy.init()
    node = WayPointsGenerator()

    try:
        rclpy.spin(node)
    except Exception:
        node.get_logger().error("Unexpected error", exc_info=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
