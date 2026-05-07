from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    DEOS Main Launch File - Complete Autonomous Vehicle Stack
    
    Launches:
    1. Sensors (Camera, GPS, IMU, LiDAR)
    2. Perception (Lane detection)
    3. Localization (PCL localization)
    
    Data Flow:
    /camera/color/image_raw (30 Hz) → Lane detection → /lane_walls (obstacles)
    /gps/fix (5-10 Hz) → Mission planning → /goal_pose
    /imu/data (100 Hz) ──┐
    /scan_fullframe (20 Hz) ├→ PCL Localization → /odometry/icp
    /cloud_unstructured (20 Hz) ┘
    
    Algorithm pipeline:
    - stereo_detector_node: /camera/* -> /perception/stereo_detections (JSON String)
    - lidar_obstacle_node:  /points_downsampled -> /perception/lidar_obstacles (JSON String)
    - perception_fusion_node: stereo+lidar+imu -> /perception/* (Bool/Float32)
    - mission_planning_node: gps+imu + mission_file -> /planning/* (Float32/Bool/String)
    - vehicle_controller_node: perception+planning -> /cmd_vel + /safety/emergency_stop
    - stm32_bridge_node: /cmd_vel -> /stm32/* (STM32 via micro-ROS)
    """

    mission_file_arg = DeclareLaunchArgument(
        "mission_file",
        default_value="",
        description="Görev rotası GeoJSON dosyası tam yolu",
    )

    centerlines_file_arg = DeclareLaunchArgument(
        "centerlines_file",
        default_value="",
        description="Pist şerit centerline GeoJSON (LineString) tam yolu (routing graph üretimi için)",
    )

    hardware_motion_enable_topic_arg = DeclareLaunchArgument(
        "hardware_motion_enable_topic",
        default_value="/hardware/motion_enable",
        description="STM32 -> Pi tek komut topic'i (std_msgs/Bool): false=DUR, true=DEVAM",
    )
    hardware_motion_enable_timeout_arg = DeclareLaunchArgument(
        "hardware_motion_enable_timeout_s",
        default_value="0.5",
        description="STM32 mesajı gelmezse (olay bazlı akış) Pi tarafında fail-safe süresi (saniye)",
    )
    autonomy_enable_topic_arg = DeclareLaunchArgument(
        "autonomy_enable_topic",
        default_value="/hardware/autonomy_enable",
        description="Otonom/Manuel geçiş topic'i (std_msgs/Bool): false=MANUEL, true=OTONOM",
    )

    require_go_signal_arg = DeclareLaunchArgument(
        "require_go_signal",
        default_value="true",
        description="UMS-2 Go sinyali gelmeden göreve başlamayı engelle (mission_planning_node speed=0).",
    )

    tunnel_mandatory_arg = DeclareLaunchArgument(
        "tunnel_mandatory",
        default_value="true",
        description="Centerlines GeoJSON'da tunnel: true varsa her bacak en az bir tünel kenarından geçer (görev dosyasında alan gerekmez).",
    )
    
    # Sensor Nodes
    camera_node = Node(
        package='camera',
        executable='realsense_d415_node',
        name='realsense_d415_node',
        output='screen',
        respawn=True,
        respawn_delay=2,
    )
    
    gps_node = Node(
        package='imu',
        executable='gps_node',
        name='gps_node',
        output='screen',
        respawn=True,
        respawn_delay=2,
    )
    
    imu_node = Node(
        package='imu',
        executable='imu_node',
        name='imu_node',
        output='screen',
        respawn=True,
        respawn_delay=2,
    )
    
    lidar_node = Node(
        package='sick_scan_xd',
        executable='sick_generic_caller',
        name='sick_multiscan165',
        parameters=[{
            'hostname': '192.168.0.1',
            'scanner_type': 'sick_multiscan',
            'frame_id': 'laser',
        }],
        output='screen',
        respawn=True,
        respawn_delay=2,
    )
    
    # Perception (algorithms)
    stereo_detector_node = Node(
        package="vision_bridge",
        executable="stereo_detector_node",
        name="stereo_detector_node",
        parameters=[{
            "image_width": 640,
            "image_height": 480,
            "focal_length_px": 320.0,
        }],
        output="screen",
        respawn=True,
        respawn_delay=2,
    )

    lidar_obstacle_node = Node(
        package="sensor_fusion",
        executable="lidar_obstacle_node",
        name="lidar_obstacle_node",
        parameters=[{
            "cluster_epsilon_m": 0.5,
            "cluster_min_points": 5,
            "max_distance_m": 20.0,
            "corridor_half_width_m": 3.0,
        }],
        output="screen",
        respawn=True,
        respawn_delay=2,
    )

    perception_fusion_node = Node(
        package="vision_bridge",
        executable="perception_fusion_node",
        name="perception_fusion_node",
        parameters=[{
            "hardware_motion_enable_topic": LaunchConfiguration("hardware_motion_enable_topic"),
            "hardware_motion_enable_timeout_s": LaunchConfiguration("hardware_motion_enable_timeout_s"),
            "hardware_motion_enable_fail_safe_stop": True,
            "autonomy_enable_topic": LaunchConfiguration("autonomy_enable_topic"),
            "require_autonomy_enable": True,
        }],
        output="screen",
        respawn=True,
        respawn_delay=2,
    )

    # Lane tracking (Raspberry/Hailo)
    lane_detection_node = Node(
        package="lane_tracking",
        executable="lane_detection_node",
        name="lane_detection_node",
        parameters=[{
            "image_topic": "/camera/color/image_raw",
            "out_center_pts_topic": "/perception/center_pts",
            "hef_path": "model.hef",
        }],
        output="screen",
        respawn=True,
        respawn_delay=2,
    )

    lane_control_node = Node(
        package="lane_tracking",
        executable="lane_control_node",
        name="lane_control_node",
        parameters=[{
            "center_pts_topic": "/perception/center_pts",
            "lane_steer_topic": "/lane/steering_ref",
            "lane_speed_topic": "/lane/speed_limit",
            "use_intent": False,
        }],
        output="screen",
        respawn=True,
        respawn_delay=2,
    )
    
    # Localization - PCL based (scan matching)
    pcl_localization_node = Node(
        package='pcl_localization_ros2',
        executable='pcl_localization_node',
        name='pcl_localization_node',
        output='screen',
        respawn=True,
        respawn_delay=2,
    )

    # Planning
    mission_planning_node = Node(
        package="mission_planning",
        executable="mission_planning_node",
        name="mission_planning_node",
        parameters=[{
            "mission_file": LaunchConfiguration("mission_file"),
            "centerlines_file": LaunchConfiguration("centerlines_file"),
            "centerlines_round_decimals": 7,
            "require_go_signal": LaunchConfiguration("require_go_signal"),
            "go_topic": LaunchConfiguration("hardware_motion_enable_topic"),
            "heading_offset_deg": 0.0,
            "tunnel_mandatory": LaunchConfiguration("tunnel_mandatory"),
        }],
        output="screen",
        respawn=True,
        respawn_delay=2,
    )

    # STM32 Bridge: cmd_vel -> (speed_delta, steering_deg) Float32 topics
    stm32_bridge_node = Node(
        package="vehicle_controller",
        executable="stm32_bridge_node",
        name="stm32_bridge_node",
        parameters=[{
            "cmd_vel_topic": "/cmd_vel",
            "motion_enable_topic": LaunchConfiguration("hardware_motion_enable_topic"),
            "require_motion_enable": True,
            "autonomy_enable_topic": LaunchConfiguration("autonomy_enable_topic"),
            "require_autonomy_enable": True,
            "speed_delta_topic": "/stm32/speed_delta_mps",
            "speed_target_topic": "/stm32/speed_target_mps",
            "publish_speed_delta": True,
            "publish_speed_target": False,
            "steering_deg_topic": "/stm32/steering_deg",
            # Keep in sync with vehicle_controller_node max_steer_rads
            "max_steer_rads": 1.0,
            "steer_deg_limit": 540.0,
            "round_decimals": 2,
        }],
        output="screen",
        respawn=True,
        respawn_delay=2,
    )

    # Control
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
            "hardware_motion_enable_fail_safe_stop": True,
            "subscribe_autonomy_enable": True,
            "autonomy_enable_topic": LaunchConfiguration("autonomy_enable_topic"),
        }],
        output="screen",
        respawn=True,
        respawn_delay=2,
    )
    
    return LaunchDescription([
        mission_file_arg,
        centerlines_file_arg,
        hardware_motion_enable_topic_arg,
        hardware_motion_enable_timeout_arg,
        autonomy_enable_topic_arg,
        require_go_signal_arg,
        tunnel_mandatory_arg,
        # === SENSORS (Raw Data Acquisition) ===
        camera_node,           # RGB frames: /camera/color/image_raw (30 Hz)
        gps_node,              # GPS location: /gps/fix (5-10 Hz)
        imu_node,              # Inertial data: /imu/data (100 Hz)
        lidar_node,            # 3D scans: /scan_fullframe, /cloud_unstructured_fullframe (20 Hz)
        
        # === PERCEPTION ===
        stereo_detector_node,
        lidar_obstacle_node,
        lane_detection_node,
        lane_control_node,
        perception_fusion_node,
        
        # === LOCALIZATION (Position Estimation) ===
        pcl_localization_node, # Scan matching: /odometry/icp, /tf (20 Hz)

        # === PLANNING ===
        mission_planning_node,

        # === CONTROL ===
        vehicle_controller_node,
        stm32_bridge_node,
        
        # === PIPELINE ===
        # Sensors -> Perception -> Planning -> Controller -> /cmd_vel
    ])
