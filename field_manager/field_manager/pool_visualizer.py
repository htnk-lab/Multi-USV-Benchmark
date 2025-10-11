#!/usr/bin/env python3

import os
import traceback

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, Pose, Quaternion, Transform, TransformStamped, Vector3
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, Header
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from visualization_msgs.msg import Marker


class PoolVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("pool_visualizer")

        # declare parameter
        self.declare_parameter(
            "world_frame", "world", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter(
            "pool_frame", "pool_origin", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )
        self.declare_parameter(
            "pool_stl_file_name", "aqua_pool.stl", descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )

        self.declare_parameter(
            "dt", 0.1, descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        )

        self.declare_parameter(
            "pool_origin_position", [0.0, 0.0, 0.0], descriptor=ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        )

        # get parameter
        world_frame = str(self.get_parameter("world_frame").value)
        pool_frame = str(self.get_parameter("pool_frame").value)
        pool_stl_file_name = str(self.get_parameter("pool_stl_file_name").value)
        timer_period = float(self.get_parameter("dt").value)

        # center of the top surface of the pool(=origin of the pool model) in world coordinate(origin within motive)
        pool_origin_position_in_world = Vector3(**dict(zip(["x", "y", "z"], self.get_parameter("pool_origin_position").value)))

        world_to_pool_origin = TransformStamped(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=world_frame),
            child_frame_id=pool_frame,
            transform=Transform(
                translation=pool_origin_position_in_world,
                rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        )

        tf_static_broadcaster = StaticTransformBroadcaster(self)
        tf_static_broadcaster.sendTransform(world_to_pool_origin)

        pkg_pool_description = get_package_share_directory("field_manager")
        pool_stl_file_path = os.path.join(pkg_pool_description, "meshes", pool_stl_file_name)
        assert os.path.exists(pool_stl_file_path)

        self.pool_makrer = Marker(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id=pool_frame),
            ns="pool",
            id=0,
            type=Marker.MESH_RESOURCE,
            action=Marker.ADD,
            pose=Pose(position=Point(x=0.0, y=0.0, z=0.0), orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
            scale=Vector3(x=1.0, y=1.0, z=1.0),
            color=ColorRGBA(r=0.0, g=0.8, b=1.0, a=0.5),
            mesh_resource="file://" + pool_stl_file_path,
            mesh_use_embedded_materials=False,
        )

        # pub
        self.pool_marker_pub = self.create_publisher(Marker, "pool_marker", 10)

        # timer
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self) -> None:
        self.pool_marker_pub.publish(self.pool_makrer)


def main() -> None:
    rclpy.init()
    pool_visualizer = PoolVisualizer()

    try:
        rclpy.spin(pool_visualizer)
    except:
        pool_visualizer.get_logger().error(traceback.format_exc())
    finally:
        pool_visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
