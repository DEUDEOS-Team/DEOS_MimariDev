import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    vision_bridge_dir = get_package_share_directory('vision_bridge')

    # Lane detection from camera
    vision_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(vision_bridge_dir, 'launch', 'vision_bridge.launch.py'))
    )

    return LaunchDescription([
        vision_bridge_launch
    ])
