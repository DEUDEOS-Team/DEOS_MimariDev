import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Arguments
    mission_file_arg = DeclareLaunchArgument(
        "mission_file", default_value="", description="GeoJSON mission file"
    )
    
    # ============ SENSOR NODES ============
    # GPS and IMU nodes
    gps_node = Node(
        package="imu",
        executable="gps_node",
        name="gps_node",
        output="screen",
        remappings=[],
    )
    
    imu_node = Node(
        package="imu",
        executable="imu_node",
        name="imu_node",
        output="screen",
        remappings=[],
    )
    
    # Camera nodes
    camera_node = Node(
        package="camera",
        executable="camera_node",
        name="camera_node",
        output="screen",
        remappings=[],
    )
    
    realsense_node = Node(
        package="camera",
        executable="realsense_d415_node",
        name="realsense_d415_node",
        output="screen",
        remappings=[],
    )
    
    # ============ SENSOR FUSION - EKF + ROBOT LOCALIZATION ============
    sensor_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sensor_fusion"),
                "launch",
                "sensorfus.launch.py"
            )
        )
    )
    
    # Final odometry node
    final_odom_node = Node(
        package="sensor_fusion",
        executable="final_odom_node",
        name="final_odom_node",
        output="screen",
        remappings=[],
    )
    
    # Lidar obstacle node
    lidar_obstacle_node = Node(
        package="sensor_fusion",
        executable="lidar_obstacle_node",
        name="lidar_obstacle_node",
        output="screen",
        remappings=[],
    )
    
    # ============ PERCEPTION - VISION/OBSTACLE DETECTION ============
    vision_bridge_node = Node(
        package="vision_bridge",
        executable="vision_bridge_node",
        name="vision_bridge_node",
        output="screen",
        remappings=[],
    )
    
    stereo_detector_node = Node(
        package="vision_bridge",
        executable="stereo_detector_node",
        name="stereo_detector_node",
        output="screen",
        remappings=[],
    )
    
    perception_fusion_node = Node(
        package="vision_bridge",
        executable="perception_fusion_node",
        name="perception_fusion_node",
        output="screen",
        remappings=[],
    )
    
    # ============ LANE TRACKING ============
    lane_detection_node = Node(
        package="lane_tracking",
        executable="lane_detection_node",
        name="lane_detection_node",
        output="screen",
        remappings=[],
    )
    
    lane_control_node = Node(
        package="lane_tracking",
        executable="lane_control_node",
        name="lane_control_node",
        output="screen",
        remappings=[],
    )
    
    # ============ PLANNING ============
    mission_planning_node = Node(
        package="mission_planning",
        executable="mission_planning_node",
        name="mission_planning_node",
        parameters=[{"mission_file": LaunchConfiguration("mission_file")}],
        output="screen",
    )
    
    # ============ CONTROL ============
    vehicle_controller_node = Node(
        package="vehicle_controller",
        executable="vehicle_controller_node",
        name="vehicle_controller_node",
        parameters=[{"max_speed_mps": 3.0}],
        output="screen",
    )
    
    stm32_bridge_node = Node(
        package="vehicle_controller",
        executable="stm32_bridge_node",
        name="stm32_bridge_node",
        output="screen",
        remappings=[],
    )

    return LaunchDescription([
        mission_file_arg,
        # Sensors
        gps_node,
        imu_node,
        camera_node,
        realsense_node,
        # Sensor Fusion
        sensor_fusion_launch,
        final_odom_node,
        lidar_obstacle_node,
        # Perception
        vision_bridge_node,
        stereo_detector_node,
        perception_fusion_node,
        # Lane Tracking
        lane_detection_node,
        lane_control_node,
        # Planning
        mission_planning_node,
        # Control
        vehicle_controller_node,
        stm32_bridge_node,
    ])
