#!/usr/bin/env python

import traceback
from typing import List, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Point, Pose, PoseArray, Twist, Vector3
from numpy.typing import NDArray
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray

from .coverage_utils.field_generator import FieldGenerator
from .coverage_utils.utils import multiarray_to_ndarray, ndarray_to_multiarray, padding
from .coverage_utils.voronoi import Voronoi


class SensingRegionCalculator(Node):
    """Calculate sensing region"""

    def __init__(self) -> None:
        super().__init__("sensing_region_controller")

        # declare parameter
        self.declare_parameter("agent_id", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter(
            "grid_accuracy", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)
        )
        self.declare_parameter(
            "x_limit", [-1.0, 1.0], descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        )
        self.declare_parameter(
            "y_limit", [-1.0, 1.0], descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        )
        self.declare_parameter(
            "z_limit", [-1.0, 1.0], descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        )

        # get parameter
        self.agent_id = int(self.get_parameter("agent_id").value)

        # get parameters defining the field
        grid_accuracy = np.array(self.get_parameter("grid_accuracy").value)
        limit = np.array(
            [
                self.get_parameter("x_limit").value,
                self.get_parameter("y_limit").value,
                self.get_parameter("z_limit").value,
            ]
        )

        field_generator = FieldGenerator(grid_accuracy=grid_accuracy, limit=limit)

        # initialize importance indices
        self.phi = field_generator.generate_phi()
        self.grid_map = field_generator.generate_grid_map()
        self.point_density = np.prod(field_generator.grid_span)
        self.voronoi = Voronoi()

        self.ref_pose = Pose()

        # pub
        self.sensing_region_pub = self.create_publisher(Int8MultiArray, "sensing_region", 10)

        # sub
        self.create_subscription(PoseArray, "/curr_pose_array", self.curr_pose_array_callback, 10)
        self.create_subscription(Float32MultiArray, "/phi", self.phi_callback, 10)


    def curr_pose_array_callback(self, msg: PoseArray) -> None:
        """Calculate sensing region (Voronoi region) and its centroid from neighboring agents' poses"""

        # Calculate sensing region using neighboring agents' positions
        centroid_position, sensing_region_grid_points, sensing_region = self.calc_voronoi_tesselation(msg.poses)

        # Set reference pose by filling unused dimensions with 0 and unpacking
        self.ref_pose = Pose(position=Point(**dict(zip(["x", "y", "z"], padding(centroid_position)))))

        # Publish sensing region
        self.sensing_region_pub.publish(ndarray_to_multiarray(Int8MultiArray, sensing_region))

    def calc_voronoi_tesselation(self, pose_list: List[Pose]) -> Tuple[NDArray, List[NDArray], NDArray]:
        all_agent_position_list: List[NDArray] = []

        # Transform list of Pose objects into a list of NDArray objects containing agent positions
        for pose in pose_list:
            all_agent_position_list.append(np.array([pose.position.x, pose.position.y, pose.position.z]))

        agent_position = all_agent_position_list[self.agent_id]

        # Exclude own position
        neighbor_agent_position_list = [
            neighbor_agent_position
            for agent_id, neighbor_agent_position in enumerate(all_agent_position_list)
            if agent_id != self.agent_id
        ]

        # Calculate sensing region
        # When using a circular sensor model, this corresponds to the optimal solution when adopting h_{1} as the performance function according to K.Sugimoto et al. 2015
        centroid_position, sensing_region_grid_points, sensing_region = self.voronoi.calc_tesselation(
            agent_position=agent_position,
            neighbor_agent_position_list=neighbor_agent_position_list,
            phi=self.phi,
            grid_map=self.grid_map,
            point_density=self.point_density,
        )
        return centroid_position, sensing_region_grid_points, sensing_region

    def phi_callback(self, msg: Float32MultiArray) -> None:
        self.phi = multiarray_to_ndarray(float, np.float32, msg)

def main() -> None:
    rclpy.init()
    sensing_region_controller = SensingRegionCalculator()

    try:
        rclpy.spin(sensing_region_controller)
    except:
        sensing_region_controller.get_logger().error(traceback.format_exc())
    finally:
        sensing_region_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
