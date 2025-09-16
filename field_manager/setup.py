import os
from glob import glob

from setuptools import setup

package_name = "field_manager"
coverage_utils = package_name + "/coverage_utils"


setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, coverage_utils],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*.stl")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Toshiyuki Oshima",
    maintainer_email="toshiyuki67026@gmail.com",
    description="Package for managing monitoring field",
    license="Apache License2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "central = field_manager.central:main",
            "phi_pointcloud_visualizer = field_manager.phi_pointcloud_visualizer:main",
            "pose_collector = field_manager.pose_collector:main",
            "sensing_region_calculator = field_manager.sensing_region_calculator:main",
            "sensing_region_marker_visualizer = field_manager.sensing_region_marker_visualizer:main",
            "pool_visualizer = field_manager.pool_visualizer:main",
        ],
    },
)
