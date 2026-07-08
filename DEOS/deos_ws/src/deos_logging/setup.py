from setuptools import find_packages, setup

package_name = 'deos_logging'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='DEUDEOS-Team',
    maintainer_email='buraksahin81178@gmail.com',
    description='Shared file+ROS2 logging wrapper for DEOS nodes.',
    license='Apache-2.0',
    entry_points={},
)
