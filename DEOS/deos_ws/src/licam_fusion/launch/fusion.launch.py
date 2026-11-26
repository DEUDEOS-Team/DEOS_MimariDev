# Copyright 2026 [Arda Efe Yilmaz]
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    package_name = 'licam_fusion'

    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'fusion_params.yaml'
    )

    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/points_downsampled',
        description='LiDAR PointCloud2 topic'
    )

    bbox_topic_arg = DeclareLaunchArgument(
        'bbox_topic',
        default_value='/perception/stereo_detections',
        description='2D bounding box Detection2DArray topic'
    )

    lidar_frame_id_arg = DeclareLaunchArgument(
        'lidar_frame_id',
        default_value='cloud',
        description='TF frame ID of the LiDAR sensor'
    )

    output_frame_id_arg = DeclareLaunchArgument(
        'output_frame_id',
        default_value='',
        description='Target output TF frame ID (empty = camera optical frame)'
    )

    publish_markers_arg = DeclareLaunchArgument(
        'publish_markers',
        default_value='true',
        description='Enable or disable RViz marker publishing'
    )

    container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value='licam_fusion_container',
        description='Name of the composable node container for zero-copy intra-process comms'
    )

    fusion_container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package=package_name,
                plugin='licam_fusion::FusionNode',
                name='licam_fusion_node',
                parameters=[
                    config_file,
                    {
                        'lidar_topic': LaunchConfiguration('lidar_topic'),
                        'bbox_topic': LaunchConfiguration('bbox_topic'),
                        'lidar_frame_id': LaunchConfiguration('lidar_frame_id'),
                        'output_frame_id': LaunchConfiguration('output_frame_id'),
                        'publish_markers': LaunchConfiguration('publish_markers'),
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        ],
    )

    return LaunchDescription([
        lidar_topic_arg,
        bbox_topic_arg,
        lidar_frame_id_arg,
        output_frame_id_arg,
        publish_markers_arg,
        container_name_arg,
        fusion_container,
    ])
