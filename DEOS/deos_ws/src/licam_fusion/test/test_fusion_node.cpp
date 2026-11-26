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

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "licam_fusion/fusion_node.hpp"

namespace {

using namespace licam_fusion;

sensor_msgs::msg::PointCloud2 makeValidCloudMsg(const rclcpp::Time & stamp) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "cloud";
    msg.height = 1;
    msg.width = 16;
    msg.is_bigendian = false;
    msg.point_step = 12;
    msg.row_step = msg.width * msg.point_step;

    sensor_msgs::msg::PointField fx;
    fx.name = "x";
    fx.offset = 0;
    fx.datatype = sensor_msgs::msg::PointField::FLOAT32;
    fx.count = 1;
    sensor_msgs::msg::PointField fy = fx;
    fy.name = "y";
    fy.offset = 4;
    sensor_msgs::msg::PointField fz = fx;
    fz.name = "z";
    fz.offset = 8;
    msg.fields = {fx, fy, fz};

    msg.data.resize(static_cast<std::size_t>(msg.row_step));
    for (std::uint32_t i = 0; i < msg.width; ++i) {
        const float pt[3] = {0.0f, 0.0f, 8.0f};
        std::memcpy(msg.data.data() + static_cast<std::size_t>(i) * 12u, pt, sizeof(pt));
    }
    return msg;
}

TEST(FusionNodeTest, ValidatesPointCloud2Layout) {
    const rclcpp::Time t(10, 0, RCL_ROS_TIME);
    auto valid = makeValidCloudMsg(t);
    const auto view = FusionNode::makeCloudView(valid);
    ASSERT_TRUE(view.has_value());
    EXPECT_EQ(view->count, 16u);

    auto big_endian = valid;
    big_endian.is_bigendian = true;
    EXPECT_EQ(FusionNode::makeCloudView(big_endian).error(), core::CloudError::BigEndian);

    auto bad_type = valid;
    bad_type.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT64;
    EXPECT_EQ(FusionNode::makeCloudView(bad_type).error(), core::CloudError::NotFloat32);

    auto missing = valid;
    missing.fields.pop_back();
    EXPECT_EQ(FusionNode::makeCloudView(missing).error(), core::CloudError::MissingField);

    auto truncated = valid;
    truncated.data.resize(10);
    EXPECT_EQ(FusionNode::makeCloudView(truncated).error(), core::CloudError::SizeMismatch);
}

TEST(FusionNodeTest, RejectsInvalidDynamicParametersAndProcessesEndToEnd) {
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    auto node = std::make_shared<FusionNode>();

    // Dynamic parameter validation test
    const auto bad_res = node->set_parameter(rclcpp::Parameter("min_points_per_bbox", 0));
    EXPECT_FALSE(bad_res.successful);

    const auto good_res = node->set_parameter(rclcpp::Parameter("min_points_per_bbox", 4));
    EXPECT_TRUE(good_res.successful);

    auto helper = std::make_shared<rclcpp::Node>("test_helper");
    tf2_ros::StaticTransformBroadcaster tf_broadcaster(helper);
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = helper->now();
    tf_msg.header.frame_id = "camera_optical";
    tf_msg.child_frame_id = "cloud";
    tf_msg.transform.rotation.w = 1.0;
    tf_broadcaster.sendTransform(tf_msg);

    auto cam_pub = helper->create_publisher<sensor_msgs::msg::CameraInfo>(
        "/camera/color/camera_info", 10);
    auto bbox_pub = helper->create_publisher<vision_msgs::msg::Detection2DArray>(
        "/perception/stereo_detections", rclcpp::SensorDataQoS());
    auto lidar_pub = helper->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/points_downsampled", rclcpp::SensorDataQoS());

    vision_msgs::msg::Detection3DArray::SharedPtr received_det3d;
    visualization_msgs::msg::MarkerArray::SharedPtr received_markers;

    auto det3d_sub = helper->create_subscription<vision_msgs::msg::Detection3DArray>(
        "/perception/detections_3d", 10,
        [&](const vision_msgs::msg::Detection3DArray::SharedPtr msg) {
            received_det3d = msg;
        });
    auto marker_sub = helper->create_subscription<visualization_msgs::msg::MarkerArray>(
        "/perception/detections_3d_markers", 10,
        [&](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
            received_markers = msg;
        });

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.add_node(helper);

    sensor_msgs::msg::CameraInfo cam_info;
    cam_info.header.frame_id = "camera_optical";
    cam_info.width = 640;
    cam_info.height = 480;
    cam_info.p[0] = 500.0;
    cam_info.p[2] = 320.0;
    cam_info.p[5] = 500.0;
    cam_info.p[6] = 240.0;
    cam_info.p[10] = 1.0;

    for (int step = 0; step < 15 && !received_det3d; ++step) {
        const rclcpp::Time stamp = helper->now();
        cam_info.header.stamp = stamp;
        cam_pub->publish(cam_info);

        vision_msgs::msg::Detection2DArray dets;
        dets.header.stamp = stamp;
        dets.header.frame_id = "camera_optical";
        auto & d = dets.detections.emplace_back();
        d.id = "car";
        d.bbox.center.position.x = 320.0;
        d.bbox.center.position.y = 240.0;
        d.bbox.size_x = 120.0;
        d.bbox.size_y = 120.0;
        d.results.emplace_back();

        bbox_pub->publish(dets);
        lidar_pub->publish(makeValidCloudMsg(stamp));

        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    ASSERT_NE(received_det3d, nullptr);
    ASSERT_EQ(received_det3d->detections.size(), 1u);
    EXPECT_NEAR(received_det3d->detections[0].bbox.center.position.z, 8.0, 0.1);
    EXPECT_GT(received_det3d->detections[0].results[0].pose.covariance[14], 0.0);

    ASSERT_NE(received_markers, nullptr);
    ASSERT_GE(received_markers->markers.size(), 1u);
    EXPECT_EQ(received_markers->markers[0].action, visualization_msgs::msg::Marker::DELETEALL);
}

TEST(FusionNodeTest, ResolvesDeosRootTopicsFromCommonLayout) {
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    rclcpp::NodeOptions opts;
    opts.append_parameter_override("deos_root", "/deos");
    auto node = std::make_shared<FusionNode>(opts);

    EXPECT_EQ(node->get_parameter("camera_info_topic").as_string(),
              "/deos/sensors/camera/color/camera_info");
    EXPECT_EQ(node->get_parameter("bbox_topic").as_string(),
              "/deos/perception/stereo/detections_2d");
    EXPECT_EQ(node->get_parameter("lidar_topic").as_string(),
              "/deos/sensors/lidar/cloud_unstructured_fullframe");
    EXPECT_EQ(node->get_parameter("output_3d_bbox_topic").as_string(),
              "/deos/perception/detections_3d");
    EXPECT_EQ(node->get_parameter("marker_topic").as_string(),
              "/deos/perception/detections_3d_markers");
}

}  // namespace
