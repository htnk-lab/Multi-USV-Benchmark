#!/usr/bin/env python

from dataclasses import dataclass
from functools import partial
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from numpy.typing import NDArray
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray, Int8MultiArray

from .coverage_utils.field_generator import FieldGenerator
from .coverage_utils.sensing_performance import SensingPerformance
from .coverage_utils.utils import multiarray_to_ndarray, ndarray_to_multiarray


@dataclass
class Data:
    """Data class for sensing region management"""

    sensing_region: NDArray
    is_ready: bool = False


class PhiUpdate(Node):
    """Centralized controller to manage importance distribution"""

    def __init__(self) -> None:
        super().__init__("phi_update")

        self.declare_parameter(
            "grid_accuracy", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)
        )
        self.declare_parameter(
            "x_limit", [-1.0, 1.0], descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        )
        self.declare_parameter(
            "y_limit", [-1.0, 1.0], descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        )
        self.declare_parameter("agent_num", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter(
            "agent_prefix", "agent", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter(
            "delta_increase", 0.05, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )
        self.declare_parameter(
            "delta_decrease", 0.1, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )
        self.declare_parameter(
            "sensing_radius", 0.3, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )
        self.declare_parameter(
            "dt", 0.1, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )

        grid_accuracy = np.array(self.get_parameter("grid_accuracy").value)
        limit = np.array(
            [
                self.get_parameter("x_limit").value,
                self.get_parameter("y_limit").value,
            ]
        )

        field_generator = FieldGenerator(grid_accuracy=grid_accuracy, limit=limit)
        self.grid_map = field_generator.generate_grid_map()

        # Initialize importance distribution
        self.phi = field_generator.generate_phi()

        self.dt = float(self.get_parameter("dt").value)
        agent_num = int(self.get_parameter("agent_num").value)
        agent_prefix = str(self.get_parameter("agent_prefix").value)
        self.delta_increase = float(self.get_parameter("delta_increase").value)
        self.delta_decrease = float(self.get_parameter("delta_decrease").value)
        self.sensing_radius = float(self.get_parameter("sensing_radius").value)
        self.curr_pose_list: List[Pose] = [Pose() for _ in range(agent_num)]
        self.data_list: List[Data] = [
            Data(sensing_region=np.zeros_like(self.grid_map[0], np.bool_)) for _ in range(agent_num)
        ]
        self.pose_array_is_ready = False
        self.central_is_ready = False

        # pub
        self.phi_pub = self.create_publisher(Float32MultiArray, "/phi", 10)
        self.sum_phi_pub = self.create_publisher(Float32, "/sum_phi", 10)

        # sub
        self.create_subscription(PoseArray, "curr_pose_array", self.curr_pose_array_callback, 10)
        for agent_id in range(agent_num):
            agent_name = agent_prefix + str(agent_id)
            topic_name = agent_name + "/sensing_region"
            self.create_subscription(
                Int8MultiArray, topic_name, partial(self.sensing_region_callback, agent_id=agent_id), 10
            )

        # timer
        self.create_timer(self.dt, self.timer_callback)

    def sensing_region_callback(self, msg: Int8MultiArray, agent_id: int) -> None:
        self.data_list[agent_id] = Data(
            sensing_region=multiarray_to_ndarray(bool, np.bool_, msg),
            is_ready=True,
        )

    def curr_pose_array_callback(self, msg: PoseArray) -> None:
        self.curr_pose_list = msg.poses
        self.pose_array_is_ready = True

    def timer_callback(self) -> None:
        # Do not update until all agents' coverage paths are ready
        if self.central_is_ready:
            all_agent_position_list = [np.array([pose.position.x, pose.position.y]) for pose in self.curr_pose_list]

            for agent_id, agent_position in enumerate(all_agent_position_list):
                self.phi = self.update_phi(agent_position, self.phi, self.data_list[agent_id].sensing_region)
        else:
            if all(data.is_ready for data in self.data_list) and self.pose_array_is_ready:
                self.get_logger().info("phi_update is ready")
                self.central_is_ready = True

        self.phi_pub.publish(ndarray_to_multiarray(Float32MultiArray, self.phi))
        self.sum_phi_pub.publish(Float32(data=float(np.sum(self.phi))))

    def update_phi(self, agent_position: NDArray, phi: NDArray, region: NDArray) -> NDArray:
        grid_points = np.hstack([self.grid_map[i].reshape(-1, 1) for i in range(len(self.grid_map))])

        # NOTE: phi update law in TCST paper
        phi += (
            self.dt
            * (
                self.delta_increase
                - self.delta_decrease
                * SensingPerformance.f_ufunc(agent_position, grid_points.T, self.sensing_radius).reshape(
                    self.grid_map[0].shape
                )
                * phi
            )
            * region
        )
        # Clip to [0, 1] range
        np.clip(phi, 0.01, 1.0, out=phi)
        return phi


def main() -> None:
    rclpy.init()
    phi_update = PhiUpdate()

    try:
        rclpy.spin(phi_update)
    except Exception:
        phi_update.get_logger().error("Unexpected error", exc_info=True)
    finally:
        phi_update.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
