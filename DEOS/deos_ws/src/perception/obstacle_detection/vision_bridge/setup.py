from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'vision_bridge'

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
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'cv_bridge', 'opencv-python', 'numpy'],
    zip_safe=True,
    maintainer='aaltindas',
    maintainer_email='aaltindas.work@gmail.com',
    description='Vision bridge package for lane detection and pointcloud mapping',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_bridge_node = vision_bridge.vision_bridge_node:main'
        ],
    },
)
