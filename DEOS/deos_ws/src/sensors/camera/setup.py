from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools', 'opencv-python', 'cv-bridge', 'pyrealsense2'],
    zip_safe=True,
    maintainer='aaltindas',
    maintainer_email='aaltindas.work@gmail.com',
    description='USB/CSI camera and RealSense drivers for ROS2',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = camera.camera_node:main',
            'realsense_d415_node = camera.realsense_d415_node:main',
        ],
    },
)
