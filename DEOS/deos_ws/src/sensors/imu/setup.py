from setuptools import find_packages, setup

package_name = 'imu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial', 'pynmea2'],
    zip_safe=True,
    maintainer='aaltindas',
    maintainer_email='aaltindas.work@gmail.com',
    description='IMU and GPS drivers for serial-based sensors',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_node = imu.imu_node:main',
            'gps_node = imu.gps_node:main',
        ],
    },
)
