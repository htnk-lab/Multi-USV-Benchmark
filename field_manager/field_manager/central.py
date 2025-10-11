#!/usr/bin/env python

import traceback
from enum import IntEnum

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from .coverage_utils.field_generator import FieldGenerator
from .coverage_utils.utils import ndarray_to_multiarray

class Central(Node):
    """重要度分布を管理する集中制御器"""

    def __init__(self) -> None:
        super().__init__("central")

        # declare parameter
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
            "dt", 0.1, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        )

        # get parameter
        grid_accuracy = np.array(self.get_parameter("grid_accuracy").value)
        self.dim = len(self.get_parameter("grid_accuracy").value)
        limit = np.array(
            [
                self.get_parameter("x_limit").value,
                self.get_parameter("y_limit").value,
            ]
        )
        center = np.sum(limit, axis=1) / 2.0
        width = np.diff(limit, axis=1)

        field_generator = FieldGenerator(grid_accuracy=grid_accuracy, limit=limit)
        self.grid_map = field_generator.generate_grid_map()

        # 重要度分布初期化
        self.phi = field_generator.generate_phi()

        dt = float(self.get_parameter("dt").value)

        # pub
        self.phi_pub = self.create_publisher(Float32MultiArray, "/phi", 10)

        # timer
        self.create_timer(dt, self.timer_callback)

    def timer_callback(self) -> None:
        """生成した重要度分布をpublish"""
        self.phi_pub.publish(ndarray_to_multiarray(Float32MultiArray, self.phi))


def main() -> None:
    rclpy.init()
    central = Central()

    try:
        rclpy.spin(central)
    except:
        central.get_logger().error(traceback.format_exc())
    finally:
        central.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
