import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ekf_params = os.path.join(
        get_package_share_directory("sensor_fusion"),
        "config",
        "ekf.yaml",
    )

    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value="/deos/sensors/imu/data",
        description="EKF ve navsat için IMU topic",
    )
    gps_fix_topic_arg = DeclareLaunchArgument(
        "gps_fix_topic",
        default_value="/deos/sensors/gps/fix",
        description="navsat için NavSatFix",
    )
    gps_filtered_topic_arg = DeclareLaunchArgument(
        "gps_filtered_topic",
        default_value="/deos/sensors/gps/filtered",
        description="navsat çıkışı filtrelenmiş GPS",
    )
    odometry_gps_topic_arg = DeclareLaunchArgument(
        "odometry_gps_topic",
        default_value="/deos/localization/odom/gps",
        description="navsat çıkışı GPS odometri",
    )
    odom_ekf_out_topic_arg = DeclareLaunchArgument(
        "odom_ekf_out_topic",
        default_value="/deos/localization/odom/ekf",
        description="EKF filtrelenmiş odometri (eski /odom)",
    )

    imu_topic = LaunchConfiguration("imu_topic")
    gps_fix_topic = LaunchConfiguration("gps_fix_topic")
    gps_filtered_topic = LaunchConfiguration("gps_filtered_topic")
    odometry_gps_topic = LaunchConfiguration("odometry_gps_topic")
    odom_ekf_out_topic = LaunchConfiguration("odom_ekf_out_topic")

    ekf_param_overlay = {
        "ekf_filter_node": {
            "ros__parameters": {
                "imu0": ParameterValue(imu_topic, value_type=str),
                "odom0": ParameterValue(odometry_gps_topic, value_type=str),
            }
        }
    }

    return LaunchDescription(
        [
            imu_topic_arg,
            gps_fix_topic_arg,
            gps_filtered_topic_arg,
            odometry_gps_topic_arg,
            odom_ekf_out_topic_arg,
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[ekf_params, ekf_param_overlay],
                remappings=[("odometry/filtered", odom_ekf_out_topic)],
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[ekf_params],
                remappings=[
                    ("imu", imu_topic),
                    ("gps/fix", gps_fix_topic),
                    ("gps/filtered", gps_filtered_topic),
                    ("odometry/gps", odometry_gps_topic),
                ],
            ),
        ]
    )
