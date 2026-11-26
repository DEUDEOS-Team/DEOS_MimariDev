/*
 * Copyright 2026 [Arda Efe Yilmaz]
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef LICAM_FUSION__FUSION_NODE_HPP_
#define LICAM_FUSION__FUSION_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "licam_fusion/fusion_core.hpp"

namespace licam_fusion {

class FusionNode final : public rclcpp::Node {
public:
    explicit FusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~FusionNode() override = default;

    [[nodiscard]] static core::Expected<core::CloudView, core::CloudError>
    makeCloudView(const sensor_msgs::msg::PointCloud2 & msg) noexcept;

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        vision_msgs::msg::Detection2DArray,
        sensor_msgs::msg::PointCloud2>;

    struct Calibration {
        core::Intrinsics k{};
        core::Projection proj{};
        std::string camera_frame;
    };

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg);
    void fusionCallback(
        const vision_msgs::msg::Detection2DArray::ConstSharedPtr & bbox_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg);
    rcl_interfaces::msg::SetParametersResult onSetParameters(
        const std::vector<rclcpp::Parameter> & parameters);
    void populateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
    [[nodiscard]] core::BoxPx extractBoxPrior(const std::string & class_id) const noexcept;

    // Configuration
    std::string lidar_frame_id_;
    std::string output_frame_id_;
    std::string fixed_frame_id_;
    double max_sync_interval_sec_{0.05};
    int detector_width_{0};
    int detector_height_{0};
    bool use_rectified_image_{true};
    bool one_shot_camera_info_{false};
    bool publish_markers_{true};
    std::atomic<bool> active_{true};

    // Immutable calibration snapshot (lock-free thread-safe read/write)
    std::atomic<std::shared_ptr<const Calibration>> calib_{nullptr};

    // Core processing engine & preallocated scratch buffers (protected by core_mutex_ on param update)
    std::mutex core_mutex_;
    core::FusionCore core_;
    std::vector<core::BoxPx> boxes_;

    // Diagnostics & real-time telemetry ring buffers
    diagnostic_updater::Updater diag_updater_;
    core::RingBuffer<std::int64_t, 128> latency_us_;
    core::RingBuffer<double, 128> sync_delta_ms_;
    std::uint64_t processed_frames_{0};
    std::uint64_t dropped_clouds_{0};

    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    // Subscriptions & Synchronizer
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    message_filters::Subscriber<vision_msgs::msg::Detection2DArray> bbox_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    // Publishers
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection3d_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace licam_fusion

#endif  // LICAM_FUSION__FUSION_NODE_HPP_
