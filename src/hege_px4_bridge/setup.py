import os
from glob import glob
from setuptools import setup

package_name = "hege_px4_bridge"

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
    description="Twist -> PX4 Ackermann offboard velocity bridge with safety supervisor",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge = hege_px4_bridge.bridge_node:main",
            "step_test = hege_px4_bridge.step_test_node:main",
        ],
    },
)
