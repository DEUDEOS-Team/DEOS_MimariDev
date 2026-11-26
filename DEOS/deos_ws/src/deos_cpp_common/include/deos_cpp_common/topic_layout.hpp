#ifndef DEOS_CPP_COMMON__TOPIC_LAYOUT_HPP_
#define DEOS_CPP_COMMON__TOPIC_LAYOUT_HPP_

#include <initializer_list>
#include <map>
#include <string>
#include <string_view>

namespace deos {

namespace detail {

[[nodiscard]] inline std::string trim_root(std::string_view raw) {
    while (!raw.empty() && (raw.front() == ' ' || raw.front() == '\t' || raw.front() == '\r' || raw.front() == '\n')) {
        raw.remove_prefix(1);
    }
    while (!raw.empty() && (raw.back() == ' ' || raw.back() == '\t' || raw.back() == '\r' || raw.back() == '\n')) {
        raw.remove_suffix(1);
    }
    while (!raw.empty() && raw.back() == '/') {
        raw.remove_suffix(1);
    }
    return std::string{raw};
}

[[nodiscard]] inline std::string join_topic(
    std::string_view normalized_root,
    std::initializer_list<std::string_view> segments) {
    std::string out{normalized_root};
    for (const auto seg : segments) {
        out.push_back('/');
        out.append(seg);
    }
    return out;
}

}  // namespace detail

/**
 * @brief Python `deos_algorithms.ros_topic_layout.build_deos_topics` ile bire bir uyumlu
 * C++20 topic hiyerarşisi. Senkronizasyonu `ops/scripts/check_topic_layout_sync.py` ve
 * GTest tarafından doğrulanır.
 */
struct TopicLayout {
    std::string sensors_camera_color;
    std::string sensors_camera_depth;
    std::string sensors_camera_color_info;
    std::string sensors_camera_depth_info;
    std::string sensors_imu;
    std::string sensors_gps_fix;
    std::string sensors_gps_filtered;
    std::string sensors_lidar_points_downsampled;
    std::string sensors_lidar_cloud_unstructured_fullframe;

    std::string perception_stereo_detections;
    std::string perception_stereo_detections_2d;
    std::string perception_lidar_obstacles;
    std::string perception_detections_3d;
    std::string perception_detections_3d_markers;
    std::string perception_lane_center_pts;
    std::string perception_lane_debug;
    std::string perception_lane_walls;

    std::string perception_fusion_emergency_stop;
    std::string perception_fusion_speed_cap;
    std::string perception_fusion_steering_override;
    std::string perception_fusion_has_steering_override;
    std::string perception_fusion_park_complete;
    std::string perception_fusion_turn_permissions;
    std::string perception_fusion_decision_debug;
    std::string perception_fusion_green_elapsed_s;

    std::string planning_steering_ref;
    std::string planning_speed_limit;
    std::string planning_current_task;
    std::string planning_arrived;
    std::string planning_park_mode;
    std::string planning_park_remaining_s;

    std::string lane_steering_ref;
    std::string lane_speed_limit;

    std::string localization_odom_gps;
    std::string localization_odom_ekf;
    std::string localization_odom_icp;
    std::string localization_odom_final;

    std::string control_intent;
    std::string control_cmd_vel;

    std::string hardware_motion_enable;
    std::string hardware_autonomy_enable;

    std::string safety_emergency_stop;
    std::string safety_lane_violation;
    std::string safety_lane_violation_count;
    std::string safety_lane_violation_seconds;

    std::string actuators_stm32_speed_delta_mps;
    std::string actuators_stm32_speed_target_mps;
    std::string actuators_stm32_steering_deg;

    std::string failsafe_out_emergency_stop;
    std::string failsafe_out_speed_cap;
    std::string failsafe_out_diagnostics;
    std::string failsafe_in_fsm_reset;

    [[nodiscard]] std::map<std::string, std::string> as_map() const {
        return {
            {"sensors_camera_color", sensors_camera_color},
            {"sensors_camera_depth", sensors_camera_depth},
            {"sensors_camera_color_info", sensors_camera_color_info},
            {"sensors_camera_depth_info", sensors_camera_depth_info},
            {"sensors_imu", sensors_imu},
            {"sensors_gps_fix", sensors_gps_fix},
            {"sensors_gps_filtered", sensors_gps_filtered},
            {"sensors_lidar_points_downsampled", sensors_lidar_points_downsampled},
            {"sensors_lidar_cloud_unstructured_fullframe", sensors_lidar_cloud_unstructured_fullframe},
            {"perception_stereo_detections", perception_stereo_detections},
            {"perception_stereo_detections_2d", perception_stereo_detections_2d},
            {"perception_lidar_obstacles", perception_lidar_obstacles},
            {"perception_detections_3d", perception_detections_3d},
            {"perception_detections_3d_markers", perception_detections_3d_markers},
            {"perception_lane_center_pts", perception_lane_center_pts},
            {"perception_lane_debug", perception_lane_debug},
            {"perception_lane_walls", perception_lane_walls},
            {"perception_fusion_emergency_stop", perception_fusion_emergency_stop},
            {"perception_fusion_speed_cap", perception_fusion_speed_cap},
            {"perception_fusion_steering_override", perception_fusion_steering_override},
            {"perception_fusion_has_steering_override", perception_fusion_has_steering_override},
            {"perception_fusion_park_complete", perception_fusion_park_complete},
            {"perception_fusion_turn_permissions", perception_fusion_turn_permissions},
            {"perception_fusion_decision_debug", perception_fusion_decision_debug},
            {"perception_fusion_green_elapsed_s", perception_fusion_green_elapsed_s},
            {"planning_steering_ref", planning_steering_ref},
            {"planning_speed_limit", planning_speed_limit},
            {"planning_current_task", planning_current_task},
            {"planning_arrived", planning_arrived},
            {"planning_park_mode", planning_park_mode},
            {"planning_park_remaining_s", planning_park_remaining_s},
            {"lane_steering_ref", lane_steering_ref},
            {"lane_speed_limit", lane_speed_limit},
            {"localization_odom_gps", localization_odom_gps},
            {"localization_odom_ekf", localization_odom_ekf},
            {"localization_odom_icp", localization_odom_icp},
            {"control_intent", control_intent},
            {"localization_odom_final", localization_odom_final},
            {"hardware_motion_enable", hardware_motion_enable},
            {"hardware_autonomy_enable", hardware_autonomy_enable},
            {"control_cmd_vel", control_cmd_vel},
            {"safety_emergency_stop", safety_emergency_stop},
            {"safety_lane_violation", safety_lane_violation},
            {"safety_lane_violation_count", safety_lane_violation_count},
            {"safety_lane_violation_seconds", safety_lane_violation_seconds},
            {"actuators_stm32_speed_delta_mps", actuators_stm32_speed_delta_mps},
            {"actuators_stm32_speed_target_mps", actuators_stm32_speed_target_mps},
            {"actuators_stm32_steering_deg", actuators_stm32_steering_deg},
            {"failsafe_out_emergency_stop", failsafe_out_emergency_stop},
            {"failsafe_out_speed_cap", failsafe_out_speed_cap},
            {"failsafe_out_diagnostics", failsafe_out_diagnostics},
            {"failsafe_in_fsm_reset", failsafe_in_fsm_reset},
        };
    }
};

[[nodiscard]] inline TopicLayout build_deos_topics(std::string_view deos_root = "/deos") {
    const std::string r = detail::trim_root(deos_root);
    const auto j = [&](std::initializer_list<std::string_view> segs) {
        return detail::join_topic(r, segs);
    };

    return TopicLayout{
        .sensors_camera_color = j({"sensors", "camera", "color", "image_raw"}),
        .sensors_camera_depth = j({"sensors", "camera", "depth", "image_raw"}),
        .sensors_camera_color_info = j({"sensors", "camera", "color", "camera_info"}),
        .sensors_camera_depth_info = j({"sensors", "camera", "depth", "camera_info"}),
        .sensors_imu = j({"sensors", "imu", "data"}),
        .sensors_gps_fix = j({"sensors", "gps", "fix"}),
        .sensors_gps_filtered = j({"sensors", "gps", "filtered"}),
        .sensors_lidar_points_downsampled = j({"sensors", "lidar", "points_downsampled"}),
        .sensors_lidar_cloud_unstructured_fullframe = j({"sensors", "lidar", "cloud_unstructured_fullframe"}),

        .perception_stereo_detections = j({"perception", "stereo", "detections"}),
        .perception_stereo_detections_2d = j({"perception", "stereo", "detections_2d"}),
        .perception_lidar_obstacles = j({"perception", "lidar", "obstacles"}),
        .perception_detections_3d = j({"perception", "detections_3d"}),
        .perception_detections_3d_markers = j({"perception", "detections_3d_markers"}),
        .perception_lane_center_pts = j({"perception", "lane", "center_pts"}),
        .perception_lane_debug = j({"perception", "lane", "debug"}),
        .perception_lane_walls = j({"perception", "lane", "walls"}),

        .perception_fusion_emergency_stop = j({"perception", "fusion", "emergency_stop"}),
        .perception_fusion_speed_cap = j({"perception", "fusion", "speed_cap"}),
        .perception_fusion_steering_override = j({"perception", "fusion", "steering_override"}),
        .perception_fusion_has_steering_override = j({"perception", "fusion", "has_steering_override"}),
        .perception_fusion_park_complete = j({"perception", "fusion", "park_complete"}),
        .perception_fusion_turn_permissions = j({"perception", "fusion", "turn_permissions"}),
        .perception_fusion_decision_debug = j({"perception", "fusion", "decision_debug"}),
        .perception_fusion_green_elapsed_s = j({"perception", "fusion", "green_elapsed_s"}),

        .planning_steering_ref = j({"planning", "steering_ref"}),
        .planning_speed_limit = j({"planning", "speed_limit"}),
        .planning_current_task = j({"planning", "current_task"}),
        .planning_arrived = j({"planning", "arrived"}),
        .planning_park_mode = j({"planning", "park_mode"}),
        .planning_park_remaining_s = j({"planning", "park_remaining_s"}),

        .lane_steering_ref = j({"lane", "steering_ref"}),
        .lane_speed_limit = j({"lane", "speed_limit"}),

        .localization_odom_gps = j({"localization", "odom", "gps"}),
        .localization_odom_ekf = j({"localization", "odom", "ekf"}),
        .localization_odom_icp = "/odometry/icp",
        .localization_odom_final = j({"localization", "odom", "final"}),

        .control_intent = j({"control", "intent"}),
        .control_cmd_vel = j({"control", "cmd_vel"}),

        .hardware_motion_enable = j({"hardware", "motion_enable"}),
        .hardware_autonomy_enable = j({"hardware", "autonomy_enable"}),

        .safety_emergency_stop = j({"safety", "emergency_stop"}),
        .safety_lane_violation = j({"safety", "lane", "violation"}),
        .safety_lane_violation_count = j({"safety", "lane", "violation_count"}),
        .safety_lane_violation_seconds = j({"safety", "lane", "violation_seconds"}),

        .actuators_stm32_speed_delta_mps = j({"actuators", "stm32", "speed_delta_mps"}),
        .actuators_stm32_speed_target_mps = j({"actuators", "stm32", "speed_target_mps"}),
        .actuators_stm32_steering_deg = j({"actuators", "stm32", "steering_deg"}),

        .failsafe_out_emergency_stop = j({"failsafe", "out", "emergency_stop"}),
        .failsafe_out_speed_cap = j({"failsafe", "out", "speed_cap"}),
        .failsafe_out_diagnostics = j({"failsafe", "out", "diagnostics"}),
        .failsafe_in_fsm_reset = j({"failsafe", "in", "fsm_reset"}),
    };
}

}  // namespace deos

#endif  // DEOS_CPP_COMMON__TOPIC_LAYOUT_HPP_
