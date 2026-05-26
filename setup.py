import os
from glob import glob

from setuptools import setup

package_name = "safety_controller_layer"

setup(
    name=package_name,
    version="0.0.1",
    # Only the runtime module is installed; tests/ stays out of the wheel.
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nathan Chin-Lue",
    maintainer_email="nathanchinlue19@gmail.com",
    description="ROS2 ExecuteCommand action server wrapping the RoverMind "
                "safety controller for the Agilex LIMO Pro.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "safety_controller_node = "
            "safety_controller_layer.safety_controller_node:main",
            "aeb_node = safety_controller_layer.aeb_node:main",
        ],
    },
)
