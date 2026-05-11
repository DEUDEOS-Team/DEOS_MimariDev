import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mission_file_arg = DeclareLaunchArgument(
        "mission_file",
        default_value="",
        description="GeoJSON mission file path"
    )
    
    hardware_motion_enable_topic_arg = DeclareLaunchArgument(
        "hardware_motion_enable_topic",
        default_value="/hardware/motion_enable",
        description="STM32 motion enable topic"
    )
    
    hardware_motion_enable_timeout_arg = DeclareLaunchArgument(
        "hardware_motion_enable_timeout_s",
        default_value="0.5",
        description="STM32 timeout seconds"
    )
    
    sensor_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sensor_fusion"),
                "launch",
                "sensorfus.launch.py"
            )
        )
    )

    mission_planning_node = Node(
        package="mission_planning",
        executable="mission_planning_node",
        name="mission_planning_node",
        parameters=[{
            "mission_file": LaunchConfiguration("mission_file"),
        }],
        output="screen",
        respawn=True,
        respawn_delay=2,
    )

    vehicle_controller_node = Node(
        package="vehicle_controller",
        executable="vehicle_controller_node",
        name="vehicle_controller_node",
        parameters=[{
            "max_speed_mps": 3.0,
            "max_steer_rads": 1.0,
            "subscribe_hardware_motion_enable": True,
            "hardware_motion_enable_topic": LaunchConfiguration("hardware_motion_enable_topic"),
            "hardware_motion_enable_timeout_s": LaunchConfiguration("hardware_motion_enable_timeout_s"),
        }],
        output="screen",
        respawn=True,
        respawn_delay=2,
    )

    return LaunchDescription([
        mission_file_arg,
        hardware_motion_enable_topic_arg,
        hardware_motion_enable_timeout_arg,
        sensor_fusion_launch,
        mission_planning_node,
        vehicle_controller_node,
    ])
