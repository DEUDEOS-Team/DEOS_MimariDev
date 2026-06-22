import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from deos_algorithms.ros_topic_layout import build_deos_topics

_T0 = build_deos_topics("/deos")


def _localization_stack_from_deos(context):
    """EKF/navsat ve PCL için topic yollarını ``deos_root`` ile üretir."""
    dr = LaunchConfiguration("deos_root").perform(context).strip()
    if not dr:
        dr = "/deos"
    T = build_deos_topics(dr)
    pcl_share = get_package_share_directory("pcl_localization_ros2")
    sf_share = get_package_share_directory("sensor_fusion")
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sf_share, "launch", "sensorfus.launch.py")
            ),
            launch_arguments={
                "imu_topic": T["sensors_imu"],
                "gps_fix_topic": T["sensors_gps_fix"],
                "gps_filtered_topic": T["sensors_gps_filtered"],
                "odometry_gps_topic": T["localization_odom_gps"],
                "odom_ekf_out_topic": T["localization_odom_ekf"],
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pcl_share, "launch", "pcl_localization.launch.py")
            ),
            launch_arguments={
                "localization_param_dir": os.path.join(pcl_share, "param", "localization.yaml"),
                "cloud_topic": T["sensors_lidar_cloud_unstructured_fullframe"],
                "publish_static_sensor_tfs": "false",
            }.items(),
        ),
    ]


def generate_launch_description():
    """
    DEOS Main Launch File - Complete Autonomous Vehicle Stack

    ``deos_root`` (varsayılan ``/deos``) altında topic ağacı ``build_deos_topics`` ile tutarlıdır.
    EKF/navsat + PCL ``OpaqueFunction`` ile aynı kökten beslenir.
    """

    mission_file_arg = DeclareLaunchArgument(
        "mission_file",
        default_value="",
        description="Görev rotası GeoJSON dosyası tam yolu",
    )

    deos_root_arg = DeclareLaunchArgument(
        "deos_root",
        default_value="/deos",
        description="DEOS topic kökü; sensör/algı/planlama/kontrol/failsafe yolları build_deos_topics ile türetilir",
    )

    centerlines_file_arg = DeclareLaunchArgument(
        "centerlines_file",
        default_value="",
        description="Pist şerit centerline GeoJSON (LineString) tam yolu (routing graph üretimi için)",
    )

    hardware_motion_enable_topic_arg = DeclareLaunchArgument(
        "hardware_motion_enable_topic",
        default_value=_T0["hardware_motion_enable"],
        description="STM32 -> Pi tek komut topic'i (std_msgs/Bool): false=DUR, true=DEVAM",
    )
    hardware_motion_enable_timeout_arg = DeclareLaunchArgument(
        "hardware_motion_enable_timeout_s",
        default_value="0.5",
        description="STM32 mesajı gelmezse (olay bazlı akış) Pi tarafında fail-safe süresi (saniye)",
    )
    autonomy_enable_topic_arg = DeclareLaunchArgument(
        "autonomy_enable_topic",
        default_value=_T0["hardware_autonomy_enable"],
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
        parameters=[{
            "deos_root": LaunchConfiguration("deos_root"),
        }],
        output='screen',
        respawn=True,
        respawn_delay=2,
    )
    
    gps_node = Node(
        package='imu',
        executable='gps_node',
        name='gps_node',
        parameters=[{
            "deos_root": LaunchConfiguration("deos_root"),
        }],
        output='screen',
        respawn=True,
        respawn_delay=2,
    )
    
    imu_node = Node(
        package='imu',
        executable='imu_node',
        name='imu_node',
        parameters=[{
            "deos_root": LaunchConfiguration("deos_root"),
        }],
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
            "deos_root": LaunchConfiguration("deos_root"),
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
            "deos_root": LaunchConfiguration("deos_root"),
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
            "deos_root": LaunchConfiguration("deos_root"),
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
            "deos_root": LaunchConfiguration("deos_root"),
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
            "deos_root": LaunchConfiguration("deos_root"),
            "use_intent": False,
        }],
        output="screen",
        respawn=True,
        respawn_delay=2,
    )
    
    localization_stack = OpaqueFunction(function=_localization_stack_from_deos)

    # Level-3 fusion: prefer ICP when fresh, else EKF odom
    final_odom_node = Node(
        package="sensor_fusion",
        executable="final_odom_node",
        name="final_odom_node",
        parameters=[{
            "deos_root": LaunchConfiguration("deos_root"),
            "icp_timeout_s": 0.2,
        }],
        output="screen",
        respawn=True,
        respawn_delay=2,
    )

    # Planning
    mission_planning_node = Node(
        package="mission_planning",
        executable="mission_planning_node",
        name="mission_planning_node",
        parameters=[{
            "deos_root": LaunchConfiguration("deos_root"),
            "mission_file": LaunchConfiguration("mission_file"),
            "centerlines_file": LaunchConfiguration("centerlines_file"),
            "centerlines_round_decimals": 7,
            "require_go_signal": LaunchConfiguration("require_go_signal"),
            "go_topic": LaunchConfiguration("hardware_motion_enable_topic"),
            "heading_offset_deg": 0.0,
            "heading_source": "final_odom",
            "tunnel_mandatory": LaunchConfiguration("tunnel_mandatory"),
            "mission_only_reorder_by_nearest": True,
            "mission_only_keep_park_last": True,
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
            "deos_root": LaunchConfiguration("deos_root"),
            "motion_enable_topic": LaunchConfiguration("hardware_motion_enable_topic"),
            "require_motion_enable": True,
            "autonomy_enable_topic": LaunchConfiguration("autonomy_enable_topic"),
            "require_autonomy_enable": True,
            "publish_speed_delta": True,
            "publish_speed_target": False,
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
            "deos_root": LaunchConfiguration("deos_root"),
            "max_speed_mps": 3.0,
            "max_steer_rads": 1.0,
            "subscribe_hardware_motion_enable": True,
            "hardware_motion_enable_topic": LaunchConfiguration("hardware_motion_enable_topic"),
            "hardware_motion_enable_timeout_s": LaunchConfiguration("hardware_motion_enable_timeout_s"),
            "hardware_motion_enable_fail_safe_stop": True,
            "subscribe_autonomy_enable": True,
            "autonomy_enable_topic": LaunchConfiguration("autonomy_enable_topic"),
            "subscribe_failsafe": True,
        }],
        output="screen",
        respawn=True,
        respawn_delay=2,
    )

    failsafe_supervisor_node = Node(
        package="deos_failsafe",
        executable="failsafe_supervisor_node",
        name="failsafe_supervisor_node",
        parameters=[{
            "deos_root": LaunchConfiguration("deos_root"),
            "max_vehicle_speed_mps": 3.0,
            "max_vehicle_steer_rad": 1.0,
            "planning_max_speed_mps": 4.0,
            "startup_grace_s": 4.0,
        }],
        output="screen",
        respawn=True,
        respawn_delay=2,
    )
    
    return LaunchDescription([
        mission_file_arg,
        centerlines_file_arg,
        deos_root_arg,
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
        localization_stack,
        final_odom_node,        # /deos/localization/odom/final

        # === PLANNING ===
        mission_planning_node,

        # === CONTROL ===
        failsafe_supervisor_node,
        vehicle_controller_node,
        stm32_bridge_node,
        
        # === PIPELINE ===
        # Sensors -> Perception -> Planning -> Controller -> /cmd_vel
    ])
