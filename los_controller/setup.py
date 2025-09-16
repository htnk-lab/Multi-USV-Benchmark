import os
from glob import glob

from setuptools import setup

package_name = 'los_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dolphin',
    maintainer_email='tymtox4@gmail.com',
    description='LOS controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "angle_fbcontroller = los_controller.angle_fbcontroller:main",
            "convex_polygon_creator = los_controller.convex_polygon_creator:main",
            "los = los_controller.los:main",
            "waypoints_generator = los_controller.waypoints_generator:main",
        ],
    },
)