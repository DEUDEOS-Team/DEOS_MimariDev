from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu',
            executable='imu_node',
            name='imu',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB1',      # Adjust to your IMU serial port
                'baudrate': 115200,
                'frame_id': 'imu_link',
                'imu_type': 'mpu9250'        # Change based on your IMU: 'mpu9250', 'vectornav', 'xsens'
            }]
        ),
        Node(
            package='imu',
            executable='gps_node',
            name='gps',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB0',      # Adjust to your GPS serial port
                'baudrate': 9600,
                'frame_id': 'gps_link'
            }]
        )
    ])
