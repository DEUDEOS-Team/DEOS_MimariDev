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

#include "licam_fusion/fusion_node.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string_view>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "deos_cpp_common/qos.hpp"
#include "deos_cpp_common/topic_layout.hpp"

namespace licam_fusion {

namespace {

[[nodiscard]] core::Projection projectionFromEigen(
    const core::Intrinsics & k,
    const Eigen::Affine3d & affine_tf) noexcept {
    const Eigen::Matrix3d rot = affine_tf.rotation();
    const Eigen::Vector3d trans = affine_tf.translation();
    const std::array<float, 9> r_row_major{
        static_cast<float>(rot(0, 0)), static_cast<float>(rot(0, 1)), static_cast<float>(rot(0, 2)),
        static_cast<float>(rot(1, 0)), static_cast<float>(rot(1, 1)), static_cast<float>(rot(1, 2)),
        static_cast<float>(rot(2, 0)), static_cast<float>(rot(2, 1)), static_cast<float>(rot(2, 2))
    };
    const std::array<float, 3> t_vec{
        static_cast<float>(trans.x()),
        static_cast<float>(trans.y()),
        static_cast<float>(trans.z())
    };
    return core::Projection::from(k, r_row_major, t_vec);
}

}  // namespace

FusionNode::FusionNode(const rclcpp::NodeOptions & options)
: Node("licam_fusion_node", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  core_(1u << 16u, 128u, core::Params{}),
  diag_updater_(this) {
    // DEOS topic kökü verildiyse (ör. "/deos") varsayılan topic'leri ortak şemadan türet;
    // boş bırakıldığında bağımsız (standalone) varsayılanları koru.
    const auto deos_root = this->declare_parameter<std::string>("deos_root", "");
    const bool use_deos_layout = !deos_root.empty();
    const auto deos_topics = deos::build_deos_topics(use_deos_layout ? deos_root : "/deos");

    // Declare parameters
    const auto camera_info_topic = this->declare_parameter<std::string>(
        "camera_info_topic",
        use_deos_layout ? deos_topics.sensors_camera_color_info : "/camera/color/camera_info");
    const auto bbox_topic = this->declare_parameter<std::string>(
        "bbox_topic",
        use_deos_layout ? deos_topics.perception_stereo_detections_2d : "/perception/stereo_detections");
    const auto lidar_topic = this->declare_parameter<std::string>(
        "lidar_topic",
        use_deos_layout ? deos_topics.sensors_lidar_cloud_unstructured_fullframe : "/points_downsampled");
    const auto output_3d_bbox_topic = this->declare_parameter<std::string>(
        "output_3d_bbox_topic",
        use_deos_layout ? deos_topics.perception_detections_3d : "/perception/detections_3d");
    const auto marker_topic = this->declare_parameter<std::string>(
        "marker_topic",
        use_deos_layout ? deos_topics.perception_detections_3d_markers : "/perception/detections_3d_markers");

    lidar_frame_id_ = this->declare_parameter<std::string>("lidar_frame_id", "cloud");
    output_frame_id_ = this->declare_parameter<std::string>("output_frame_id", "");
    fixed_frame_id_ = this->declare_parameter<std::string>("fixed_frame_id", "");

    const auto sync_queue_raw = this->declare_parameter<int>("sync_queue_size", 15);
    max_sync_interval_sec_ = this->declare_parameter<double>("max_sync_interval_sec", 0.05);
    detector_width_ = static_cast<int>(this->declare_parameter<int>("detector_width", 0));
    detector_height_ = static_cast<int>(this->declare_parameter<int>("detector_height", 0));
    use_rectified_image_ = this->declare_parameter<bool>("use_rectified_image", true);
    one_shot_camera_info_ = this->declare_parameter<bool>("one_shot_camera_info", false);
    publish_markers_ = this->declare_parameter<bool>("publish_markers", true);

    core::Params prm{};
    const auto min_pts_raw = this->declare_parameter<int>("min_points_per_bbox", 5);
    const auto cluster_tol_raw = this->declare_parameter<double>("cluster_tolerance", 1.2);
    const auto min_depth_raw = this->declare_parameter<double>("min_depth", 0.2);
    const auto max_depth_raw = this->declare_parameter<double>("max_depth", 100.0);
    const auto box_shrink_raw = this->declare_parameter<double>("box_shrink", 0.9);
    const auto bin_size_raw = this->declare_parameter<double>("bin_size", 0.1);
    prm.occlusion_handling = this->declare_parameter<bool>("occlusion_handling", true);
    prm.filter_ground = this->declare_parameter<bool>("filter_ground", false);
    prm.max_camera_y = static_cast<float>(this->declare_parameter<double>("max_camera_y", 1.8));
    prm.class_prior_weight =
        static_cast<float>(this->declare_parameter<double>("class_prior_weight", 0.0));

    // Parameter validation
    prm.min_points = static_cast<std::uint32_t>(std::max<std::int64_t>(1, min_pts_raw));
    prm.cluster_tolerance = static_cast<float>(std::max(0.05, cluster_tol_raw));
    prm.min_depth = static_cast<float>(std::max(0.01, min_depth_raw));
    prm.max_depth = static_cast<float>(std::max(static_cast<double>(prm.min_depth) + 0.5, max_depth_raw));
    prm.box_shrink = static_cast<float>(std::clamp(box_shrink_raw, 0.1, 1.0));
    prm.bin_size = static_cast<float>(std::clamp(bin_size_raw, 0.01, 1.0));
    core_.configure(prm);
    boxes_.reserve(128);

    // Dynamic parameter callback
    param_cb_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> & params) {
            return this->onSetParameters(params);
        });

    // Diagnostics setup
    diag_updater_.setHardwareID("licam_fusion");
    diag_updater_.add("Fusion Pipeline Status", this, &FusionNode::populateDiagnostics);

    // TF2 setup
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, this);

    // BestEffort QoS for CameraInfo (deos_cpp_common SensorData profili)
    camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic, deos::qos::sensor_data(10),
        [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
            this->cameraInfoCallback(msg);
        });

    // Publishers
    detection3d_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(
        output_3d_bbox_topic, deos::qos::control_reliable(10));
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        marker_topic, deos::qos::control_reliable(10));

    // Synchronized subscribers (SensorData QoS)
    const int sync_queue = static_cast<int>(std::max<std::int64_t>(2, sync_queue_raw));
    bbox_sub_.subscribe(this, bbox_topic, rmw_qos_profile_sensor_data);
    lidar_sub_.subscribe(this, lidar_topic, rmw_qos_profile_sensor_data);

    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(sync_queue), bbox_sub_, lidar_sub_);
    if (max_sync_interval_sec_ > 0.0) {
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(max_sync_interval_sec_));
    }
    sync_->registerCallback(&FusionNode::fusionCallback, this);

    RCLCPP_INFO(
        this->get_logger(),
        "LiCam Fusion Node ready. LiDAR Frame: %s | Min Points: %u | Depth: [%.1f, %.1f] m | Max Sync dt: %.1f ms",
        lidar_frame_id_.c_str(), prm.min_points,
        static_cast<double>(prm.min_depth), static_cast<double>(prm.max_depth),
        max_sync_interval_sec_ * 1000.0);
}

rcl_interfaces::msg::SetParametersResult FusionNode::onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    std::scoped_lock lock(core_mutex_);
    core::Params prm = core_.params();

    for (const auto & param : parameters) {
        const auto & name = param.get_name();
        if (name == "min_points_per_bbox") {
            if (param.as_int() < 1) {
                result.successful = false;
                result.reason = "min_points_per_bbox must be >= 1";
                return result;
            }
            prm.min_points = static_cast<std::uint32_t>(param.as_int());
        } else if (name == "cluster_tolerance") {
            if (param.as_double() <= 0.0) {
                result.successful = false;
                result.reason = "cluster_tolerance must be > 0.0";
                return result;
            }
            prm.cluster_tolerance = static_cast<float>(param.as_double());
        } else if (name == "min_depth") {
            if (param.as_double() <= 0.0 || param.as_double() >= static_cast<double>(prm.max_depth)) {
                result.successful = false;
                result.reason = "min_depth must be > 0 and < max_depth";
                return result;
            }
            prm.min_depth = static_cast<float>(param.as_double());
        } else if (name == "max_depth") {
            if (param.as_double() <= static_cast<double>(prm.min_depth)) {
                result.successful = false;
                result.reason = "max_depth must be > min_depth";
                return result;
            }
            prm.max_depth = static_cast<float>(param.as_double());
        } else if (name == "box_shrink") {
            if (param.as_double() < 0.1 || param.as_double() > 1.0) {
                result.successful = false;
                result.reason = "box_shrink must be within [0.1, 1.0]";
                return result;
            }
            prm.box_shrink = static_cast<float>(param.as_double());
        } else if (name == "bin_size") {
            if (param.as_double() < 0.01 || param.as_double() > 1.0) {
                result.successful = false;
                result.reason = "bin_size must be within [0.01, 1.0]";
                return result;
            }
            prm.bin_size = static_cast<float>(param.as_double());
        } else if (name == "occlusion_handling") {
            prm.occlusion_handling = param.as_bool();
        } else if (name == "filter_ground") {
            prm.filter_ground = param.as_bool();
        } else if (name == "max_camera_y") {
            prm.max_camera_y = static_cast<float>(param.as_double());
        } else if (name == "class_prior_weight") {
            if (param.as_double() < 0.0 || param.as_double() > 1.0) {
                result.successful = false;
                result.reason = "class_prior_weight must be within [0.0, 1.0]";
                return result;
            }
            prm.class_prior_weight = static_cast<float>(param.as_double());
        } else if (name == "publish_markers") {
            publish_markers_ = param.as_bool();
        }
    }

    core_.configure(prm);
    return result;
}

void FusionNode::populateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat) {
    const auto calib = calib_.load();
    if (!calib) {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Waiting for CameraInfo & TF calibration");
    } else {
        stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Calibrated and active");
        stat.add("camera_frame", calib->camera_frame);
        stat.add("image_size", std::to_string(calib->k.width) + "x" + std::to_string(calib->k.height));
    }
    stat.add("processed_frames", processed_frames_);
    stat.add("dropped_clouds", dropped_clouds_);
    stat.add("latency_mean_us", latency_us_.mean());
    stat.add("latency_p99_us", latency_us_.percentile99());
    stat.add("sync_delta_mean_ms", sync_delta_ms_.mean());
    stat.add("sync_delta_p99_ms", sync_delta_ms_.percentile99());
}

core::Expected<core::CloudView, core::CloudError>
FusionNode::makeCloudView(const sensor_msgs::msg::PointCloud2 & msg) noexcept {
    if (msg.is_bigendian) {
        return core::Unexpected(core::CloudError::BigEndian);
    }
    if (msg.data.empty()) {
        return core::Unexpected(core::CloudError::NullData);
    }

    std::array<std::optional<std::size_t>, 3> off{};
    constexpr std::array<std::string_view, 3> kNames{"x", "y", "z"};
    for (const auto & field : msg.fields) {
        for (std::size_t i = 0; i < 3; ++i) {
            if (field.name == kNames[i]) {
                if (field.datatype != sensor_msgs::msg::PointField::FLOAT32) {
                    return core::Unexpected(core::CloudError::NotFloat32);
                }
                off[i] = static_cast<std::size_t>(field.offset);
            }
        }
    }

    if (!off[0] || !off[1] || !off[2]) {
        return core::Unexpected(core::CloudError::MissingField);
    }

    const std::size_t count = static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);
    const std::size_t step = static_cast<std::size_t>(msg.point_step);
    if (step < 12 || msg.data.size() < count * step) {
        return core::Unexpected(core::CloudError::SizeMismatch);
    }

    return core::CloudView{
        reinterpret_cast<const std::byte *>(msg.data.data()),
        count,
        step,
        *off[0],
        *off[1],
        *off[2]
    };
}

core::BoxPx FusionNode::extractBoxPrior(const std::string & class_id) const noexcept {
    core::BoxPx b{};
    if (class_id == "car" || class_id == "vehicle") {
        b.prior_size_x = 1.85f;
        b.prior_size_y = 1.55f;
        b.prior_size_z = 4.40f;
    } else if (class_id == "truck" || class_id == "bus") {
        b.prior_size_x = 2.50f;
        b.prior_size_y = 3.00f;
        b.prior_size_z = 8.50f;
    } else if (class_id == "person" || class_id == "pedestrian") {
        b.prior_size_x = 0.65f;
        b.prior_size_y = 1.72f;
        b.prior_size_z = 0.65f;
    } else if (class_id == "bicycle" || class_id == "cyclist" || class_id == "motorcycle") {
        b.prior_size_x = 0.75f;
        b.prior_size_y = 1.60f;
        b.prior_size_z = 1.80f;
    } else if (class_id == "cone" || class_id == "koni" || class_id == "traffic_cone") {
        b.prior_size_x = 0.30f;
        b.prior_size_y = 0.70f;
        b.prior_size_z = 0.30f;
    } else if (class_id == "barrier" || class_id == "bariyer") {
        b.prior_size_x = 1.20f;
        b.prior_size_y = 0.90f;
        b.prior_size_z = 0.50f;
    }
    return b;
}

void FusionNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg) {
    if (msg->width == 0 || msg->height == 0) {
        return;
    }

    const auto existing = calib_.load();
    if (existing && one_shot_camera_info_) {
        return;
    }

    core::Intrinsics k{};
    k.width = static_cast<std::int32_t>(msg->width);
    k.height = static_cast<std::int32_t>(msg->height);

    if (use_rectified_image_ && msg->p[0] > 0.0) {
        k.fx = static_cast<float>(msg->p[0]);
        k.fy = static_cast<float>(msg->p[5]);
        k.cx = static_cast<float>(msg->p[2]);
        k.cy = static_cast<float>(msg->p[6]);
        k.tx = static_cast<float>(msg->p[3]);
        k.ty = static_cast<float>(msg->p[7]);
        k.apply_distortion = false;
    } else {
        k.fx = static_cast<float>(msg->k[0]);
        k.fy = static_cast<float>(msg->k[4]);
        k.cx = static_cast<float>(msg->k[2]);
        k.cy = static_cast<float>(msg->k[5]);
        k.tx = 0.0f;
        k.ty = 0.0f;
        if (!use_rectified_image_ && msg->d.size() >= 4) {
            for (std::size_t i = 0; i < std::min<std::size_t>(5, msg->d.size()); ++i) {
                k.dist[i] = static_cast<float>(msg->d[i]);
            }
            k.apply_distortion = true;
        }
    }

    if (k.fx <= 0.0f || k.fy <= 0.0f) {
        return;
    }

    // Re-compute calibration only on first message or if intrinsics/frame changed
    if (existing &&
        existing->camera_frame == msg->header.frame_id &&
        existing->k.width == k.width &&
        existing->k.height == k.height &&
        std::abs(existing->k.fx - k.fx) < 1e-3f &&
        std::abs(existing->k.fy - k.fy) < 1e-3f) {
        return;
    }

    try {
        const auto transform = tf_buffer_->lookupTransform(
            msg->header.frame_id,
            lidar_frame_id_,
            tf2::TimePointZero);

        const Eigen::Affine3d affine_tf = tf2::transformToEigen(transform);
        auto next_calib = std::make_shared<Calibration>();
        next_calib->k = k;
        next_calib->proj = projectionFromEigen(k, affine_tf);
        next_calib->camera_frame = msg->header.frame_id;

        calib_.store(std::move(next_calib));

        if (one_shot_camera_info_) {
            camera_info_sub_.reset();
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Calibration loaded [%s -> %s]. Image: %dx%d (fx: %.1f, fy: %.1f, tx: %.3f)",
            lidar_frame_id_.c_str(), msg->header.frame_id.c_str(),
            k.width, k.height,
            static_cast<double>(k.fx), static_cast<double>(k.fy), static_cast<double>(k.tx));
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "TF connection pending [%s -> %s]: %s",
            lidar_frame_id_.c_str(), msg->header.frame_id.c_str(), ex.what());
    }
}

void FusionNode::fusionCallback(
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr & bbox_msg,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg) {
    if (!active_.load()) {
        return;
    }

    const auto t_start = std::chrono::steady_clock::now();
    const auto calib = calib_.load();
    if (!calib) [[unlikely]] {
        return;
    }

    // Track synchronization delta |t_cam - t_lidar|
    const rclcpp::Time t_bbox(bbox_msg->header.stamp);
    const rclcpp::Time t_lidar(lidar_msg->header.stamp);
    const double sync_dt_ms = std::abs((t_bbox - t_lidar).seconds()) * 1000.0;
    sync_delta_ms_.push(sync_dt_ms);

    const std::string target_out_frame =
        output_frame_id_.empty() ? calib->camera_frame : output_frame_id_;

    auto output_msg = std::make_unique<vision_msgs::msg::Detection3DArray>();
    output_msg->header.stamp = lidar_msg->header.stamp;
    output_msg->header.frame_id = target_out_frame;

    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    if (publish_markers_) {
        // Clear stale markers in RViz on every synchronized frame (Section 3.5)
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header = output_msg->header;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array->markers.push_back(clear_marker);
    }

    if (bbox_msg->detections.empty()) {
        detection3d_pub_->publish(std::move(output_msg));
        if (publish_markers_ && marker_pub_) {
            marker_pub_->publish(std::move(marker_array));
        }
        return;
    }

    const auto cloud_view = makeCloudView(*lidar_msg);
    if (!cloud_view) [[unlikely]] {
        ++dropped_clouds_;
        RCLCPP_ERROR_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Invalid PointCloud2 layout (error code: %u)",
            static_cast<unsigned>(cloud_view.error()));
        detection3d_pub_->publish(std::move(output_msg));
        if (publish_markers_ && marker_pub_) {
            marker_pub_->publish(std::move(marker_array));
        }
        return;
    }

    // Optional ego-motion-compensated projection via fixed_frame_id_ (Section 3.2)
    core::Projection active_proj = calib->proj;
    if (!fixed_frame_id_.empty()) {
        try {
            const auto tf_motion = tf_buffer_->lookupTransform(
                calib->camera_frame, t_bbox,
                lidar_frame_id_, t_lidar,
                fixed_frame_id_, rclcpp::Duration::from_seconds(0.01));
            active_proj = projectionFromEigen(calib->k, tf2::transformToEigen(tf_motion));
        } catch (const tf2::TransformException & ex) {
            RCLCPP_DEBUG(this->get_logger(), "Ego-motion TF fallback to static: %s", ex.what());
        }
    }

    // Optional output frame transform (e.g. camera_optical -> base_link, Section 3.4)
    Eigen::Affine3d tf_cam_to_out = Eigen::Affine3d::Identity();
    bool has_out_tf = false;
    if (target_out_frame != calib->camera_frame) {
        try {
            const auto tf_out = tf_buffer_->lookupTransform(
                target_out_frame, calib->camera_frame, tf2::TimePointZero);
            tf_cam_to_out = tf2::transformToEigen(tf_out);
            has_out_tf = true;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Output frame TF [%s -> %s] unavailable, publishing in camera frame: %s",
                calib->camera_frame.c_str(), target_out_frame.c_str(), ex.what());
            output_msg->header.frame_id = calib->camera_frame;
        }
    }

    // Resolution scaling if 2D detector ran at a different resolution than CameraInfo (Section 3.6)
    const float scale_x = (detector_width_ > 0 && detector_width_ != calib->k.width)
        ? (static_cast<float>(calib->k.width) / static_cast<float>(detector_width_))
        : 1.0f;
    const float scale_y = (detector_height_ > 0 && detector_height_ != calib->k.height)
        ? (static_cast<float>(calib->k.height) / static_cast<float>(detector_height_))
        : 1.0f;

    std::scoped_lock lock(core_mutex_);
    boxes_.clear();
    for (const auto & det : bbox_msg->detections) {
        const float cx = static_cast<float>(det.bbox.center.position.x) * scale_x;
        const float cy = static_cast<float>(det.bbox.center.position.y) * scale_y;
        const float hw = 0.5f * static_cast<float>(det.bbox.size_x) * scale_x;
        const float hh = 0.5f * static_cast<float>(det.bbox.size_y) * scale_y;

        std::string cls_id = det.id;
        if (cls_id.empty() && !det.results.empty()) {
            cls_id = det.results.front().hypothesis.class_id;
        }
        core::BoxPx bx = extractBoxPrior(cls_id);
        bx.x_min = cx - hw;
        bx.x_max = cx + hw;
        bx.y_min = cy - hh;
        bx.y_max = cy + hh;
        boxes_.push_back(bx);
    }

    const auto objects = core_.process(active_proj, calib->k, boxes_, *cloud_view);

    output_msg->detections.reserve(objects.size());
    if (publish_markers_) {
        marker_array->markers.reserve(1 + objects.size() * 2);
    }

    int marker_id = 0;
    for (const auto & obj : objects) {
        const auto & src_det = bbox_msg->detections[obj.box];
        auto & det_3d = output_msg->detections.emplace_back();
        det_3d.header = output_msg->header;
        det_3d.id = src_det.id;
        det_3d.results = src_det.results;

        Eigen::Vector3d pos_cam(
            static_cast<double>(obj.x),
            static_cast<double>(obj.y),
            static_cast<double>(obj.z));
        Eigen::Vector3d size_vec(
            static_cast<double>(obj.size_x),
            static_cast<double>(obj.size_y),
            static_cast<double>(obj.size_z));

        if (has_out_tf) {
            pos_cam = tf_cam_to_out * pos_cam;
            const Eigen::Matrix3d abs_rot = tf_cam_to_out.rotation().cwiseAbs();
            size_vec = abs_rot * size_vec;
        }

        det_3d.bbox.center.position.x = pos_cam.x();
        det_3d.bbox.center.position.y = pos_cam.y();
        det_3d.bbox.center.position.z = pos_cam.z();
        det_3d.bbox.center.orientation.x = 0.0;
        det_3d.bbox.center.orientation.y = 0.0;
        det_3d.bbox.center.orientation.z = 0.0;
        det_3d.bbox.center.orientation.w = 1.0;

        det_3d.bbox.size.x = size_vec.x();
        det_3d.bbox.size.y = size_vec.y();
        det_3d.bbox.size.z = size_vec.z();

        // Populate pose & covariance for downstream 3D trackers (Section 3.3)
        for (auto & res : det_3d.results) {
            res.pose.pose = det_3d.bbox.center;
            res.pose.covariance.fill(0.0);
            res.pose.covariance[0] = static_cast<double>(obj.pos_variance[0]);
            res.pose.covariance[7] = static_cast<double>(obj.pos_variance[1]);
            res.pose.covariance[14] = static_cast<double>(obj.pos_variance[2]);
            res.pose.covariance[21] = 0.1;
            res.pose.covariance[28] = 0.1;
            res.pose.covariance[35] = 0.1;
        }

        if (publish_markers_) {
            auto & box_marker = marker_array->markers.emplace_back();
            box_marker.header = output_msg->header;
            box_marker.ns = "bboxes_3d";
            box_marker.id = marker_id++;
            box_marker.type = visualization_msgs::msg::Marker::CUBE;
            box_marker.action = visualization_msgs::msg::Marker::ADD;
            box_marker.pose = det_3d.bbox.center;
            box_marker.scale = det_3d.bbox.size;
            box_marker.color.r = 0.1f;
            box_marker.color.g = 0.9f;
            box_marker.color.b = 0.2f;
            box_marker.color.a = 0.45f;
            box_marker.lifetime = rclcpp::Duration::from_seconds(0.3);

            auto & text_marker = marker_array->markers.emplace_back();
            text_marker.header = output_msg->header;
            text_marker.ns = "labels_3d";
            text_marker.id = marker_id++;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.position.x = det_3d.bbox.center.position.x;
            text_marker.pose.position.y =
                det_3d.bbox.center.position.y - (det_3d.bbox.size.y * 0.5) - 0.15;
            text_marker.pose.position.z = det_3d.bbox.center.position.z;
            text_marker.pose.orientation.w = 1.0;
            text_marker.scale.z = 0.35;
            text_marker.color.r = 1.0f;
            text_marker.color.g = 1.0f;
            text_marker.color.b = 1.0f;
            text_marker.color.a = 0.95f;

            char buf[64];
            std::snprintf(
                buf, sizeof(buf), "%s [%.2fm]",
                src_det.id.empty() ? "Object" : src_det.id.c_str(),
                static_cast<double>(obj.z));
            text_marker.text = buf;
            text_marker.lifetime = rclcpp::Duration::from_seconds(0.3);
        }

        RCLCPP_DEBUG(
            this->get_logger(),
            "Object: %s | Points: %u | Depth: %.2f m | Size: [%.2f, %.2f, %.2f]",
            src_det.id.c_str(), obj.num_points, static_cast<double>(obj.z),
            det_3d.bbox.size.x, det_3d.bbox.size.y, det_3d.bbox.size.z);
    }

    // Always publish via unique_ptr for zero-copy intra-process & heartbeat monitoring (3.5, 4.1)
    detection3d_pub_->publish(std::move(output_msg));
    if (publish_markers_ && marker_pub_) {
        marker_pub_->publish(std::move(marker_array));
    }

    const auto dt_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now() - t_start).count();
    latency_us_.push(dt_us);
    ++processed_frames_;
    diag_updater_.force_update();
}

}  // namespace licam_fusion

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(licam_fusion::FusionNode)
