from glob import glob
import os
from setuptools import find_packages, setup

package_name = "robocommand"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "proto"), glob("proto/*.proto")),
        (os.path.join("share", package_name, "proto/robotx"), glob("proto/robotx/*.proto")),
    ],
    install_requires=[
        "setuptools",
        "paho-mqtt>=2.0",
        "protobuf>=7.35",
    ],
    zip_safe=True,
    maintainer="Tumi Robotics PUCP",
    maintainer_email="tumi@pucp.edu.pe",
    description="RobotX 2026 unified ROS 2 RoboCommand compliance bridge package for UAV (PX4), USV (ArduPilot), and UUV (ArduSub) via MAVROS2, MQTT, and Protocol Buffers.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "uav_bridge_node = robocommand.uav_bridge_node:main",
            "usv_bridge_node = robocommand.usv_bridge_node:main",
            "uuv_bridge_node = robocommand.uuv_bridge_node:main",
        ],
    },
)
