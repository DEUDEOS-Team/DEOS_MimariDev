from setuptools import find_packages, setup

package_name = 'vision_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aaltindas',
    maintainer_email='aaltindas.work@gmail.com',
    description='Vision bridge node for processing camera data to point clouds',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Nodun terminalden çalıştırılabilmesi için gereken tanımlama:
            'vision_bridge_node = vision_bridge.vision_bridge_node:main'
        ],
    },
)