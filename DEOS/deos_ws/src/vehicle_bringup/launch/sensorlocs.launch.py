from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    DEOS araç üstü statik sensör dönüşümleri (REP-103 uyumlu).
    - base_link (+X ileri, +Y sol, +Z yukarı) -> laser / cloud
    - base_link -> camera_link -> camera_color_frame (optik frame: +X sağ, +Y aşağı, +Z ileri)
    """
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="lidar_tf_publisher",
            arguments=[
                "--x", "0.95", "--y", "0.0", "--z", "1.37",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link", "--child-frame-id", "laser",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="lidar_cloud_alias_tf_publisher",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.0",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "laser", "--child-frame-id", "cloud",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_tf_publisher",
            arguments=[
                "--x", "0.95", "--y", "0.0", "--z", "1.37",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link", "--child-frame-id", "camera_link",
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_optical_tf_publisher",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.0",
                "--qx", "-0.5", "--qy", "0.5", "--qz", "-0.5", "--qw", "0.5",
                "--frame-id", "camera_link", "--child-frame-id", "camera_color_frame",
            ],
        ),
    ])
