import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ekf_params = os.path.join(
        get_package_share_directory('sensor_fusion'),
        'config',
        'ekf.yaml'
    )
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params], # Yukarıda tanımladığımız değişkeni burada kullanıyoruz
            remappings=[('odometry/filtered', '/odom')] 
        ),

        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[ekf_params],
            remappings=[
                ('imu', '/imu/data'),            
                ('gps/fix', '/gps/fix'),         
                ('gps/filtered', '/gps/filtered'),
                ('odometry/gps', '/odometry/gps') 
            ]
        )

    ])
