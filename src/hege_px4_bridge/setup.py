import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hege_px4_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michal Czaplinski',
    maintainer_email='michal.czaplinski@basegroup.pl',
    description='ROS 2 bridge between Nav2 / ROS 2 stack and PX4 Autopilot on the physical Hege tractor via MicroXRCE-DDS.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_bridge_node = hege_px4_bridge.px4_bridge_node:main',
            'test_px4_connection = hege_px4_bridge.test_px4_connection:main',
        ],
    },
)
