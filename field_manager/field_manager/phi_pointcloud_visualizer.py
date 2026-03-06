#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from numpy.typing import NDArray
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Header

from .coverage_utils.field_generator import FieldGenerator
from .coverage_utils.utils import multiarray_to_ndarray

POINTCLOUD_FIELD_NAMES = ["x", "y", "z", "r", "g", "b"]
BYTES_PER_FIELD = 4  # float32


class PhiPointCloudVisualizer(Node):
    """Visualize importance indices using PointCloud2

    Note:
        grid_map is arranged by reshaping and transposing as follows:
        self.rows = [
            [x1, y1, ...],
            [x2, y2, ...]
                :
            [xn, yn, ...]]
    """

    def __init__(self) -> None:
        super().__init__("phi_pointcloud_visualizer")

        self.declare_parameter(
            "world_frame", "world", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter(
            "grid_accuracy", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)
        )
        self.declare_parameter(
            "x_limit", [-1.0, 1.0], descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        )
        self.declare_parameter(
            "y_limit", [-1.0, 1.0], descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        )

        world_frame = str(self.get_parameter("world_frame").value)
        grid_accuracy = np.array(self.get_parameter("grid_accuracy").value)
        self.dim = len(self.get_parameter("grid_accuracy").value)
        limit = np.array(
            [
                self.get_parameter("x_limit").value,
                self.get_parameter("y_limit").value,
            ]
        )

        field_generator = FieldGenerator(grid_accuracy=grid_accuracy, limit=limit)
        grid_map = field_generator.generate_grid_map()
        self.rows: NDArray = np.array(grid_map).reshape([self.dim, -1]).T

        self.phi_pointcloud = PointCloud2(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id=world_frame,
            ),
            height=1,
            fields=[
                PointField(name=name, offset=i * BYTES_PER_FIELD, datatype=PointField.FLOAT32, count=1)
                for i, name in enumerate(POINTCLOUD_FIELD_NAMES)
            ],
            is_bigendian=False,
            point_step=len(POINTCLOUD_FIELD_NAMES) * BYTES_PER_FIELD,
            is_dense=True,
        )

        # pub
        self.phi_pointcloud_pub = self.create_publisher(PointCloud2, "phi_pointcloud", 10)

        # sub
        self.create_subscription(Float32MultiArray, "phi", self.phi_callback, 10)

    def phi_callback(self, msg: Float32MultiArray) -> None:
        """Generate pointcloud from Float32Multiarray and publish it

        Args:
            msg (Float32MultiArray): Importance indices

        Note:
            points = [
            [x1, y1, z1, r1, g1, b1],
            [x2, y2, z2, r2, g2, b2],
                    :
            [xn, yn, zn, rn, gn, bn]]

            By reflecting phi in the z coordinate or else, importance can be visualized by the coordinates
        """
        phi = multiarray_to_ndarray(float, np.float32, msg).reshape([-1, 1])

        # map importance values to RGB via colormap
        rgba_phi: NDArray = plt.get_cmap("jet")(phi).squeeze()

        # pad missing coordinates with 0 for dim <= 2
        points = np.hstack(
            [
                self.rows,
                np.zeros([self.rows.shape[0], 3 - self.dim]),
                rgba_phi[:, 0:3],
            ]
        ).astype(np.float32)

        num_points = len(points)
        self.phi_pointcloud.width = num_points
        self.phi_pointcloud.row_step = self.phi_pointcloud.point_step * num_points
        self.phi_pointcloud.data = points.tobytes()
        self.phi_pointcloud_pub.publish(self.phi_pointcloud)


def main() -> None:
    rclpy.init()
    node = PhiPointCloudVisualizer()

    try:
        rclpy.spin(node)
    except Exception:
        node.get_logger().error("Unexpected error", exc_info=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
