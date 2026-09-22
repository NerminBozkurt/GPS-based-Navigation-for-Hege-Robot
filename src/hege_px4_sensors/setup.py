import os
from glob import glob
from setuptools import setup

package_name = "hege_px4_sensors"

setup(
    name=package_name,
    version="0.4.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Oguzhan Enes Isik",
    maintainer_email="ouz.3406@gmail.com",
    description="PX4 /fmu/out topics to standard ROS 2 sensor topics",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sensors = hege_px4_sensors.sensors_node:main",
        ],
    },
)
