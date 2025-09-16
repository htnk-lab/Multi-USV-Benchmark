import os
from glob import glob

from setuptools import setup

package_name = 'robot_model'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*.stl")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf.xacro")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dolphin',
    maintainer_email='tymtox4@gmail.com',
    description='Models of robots',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ideal_agent = robot_model.ideal_agent:main",
            "footprinter = robot_model.footprinter:main",
            "posest2posevel = robot_model.posest2posevel:main",
        ],
    },
)
