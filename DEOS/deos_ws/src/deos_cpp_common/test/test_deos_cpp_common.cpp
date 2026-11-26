#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <deos_shm_msgs/msg/fixed_frame640x480.hpp>
#include <deos_shm_msgs/msg/fixed_point_cloud64k.hpp>

#include "deos_cpp_common/qos.hpp"
#include "deos_cpp_common/topic_layout.hpp"
#include "deos_cpp_common/zero_copy.hpp"

namespace {

TEST(TopicLayoutTest, MatchesPythonDefaultRootAndCustomRoot) {
    const auto d = deos::build_deos_topics("/deos");
    EXPECT_EQ(d.sensors_camera_color, "/deos/sensors/camera/color/image_raw");
    EXPECT_EQ(d.sensors_camera_color_info, "/deos/sensors/camera/color/camera_info");
    EXPECT_EQ(d.sensors_lidar_cloud_unstructured_fullframe,
              "/deos/sensors/lidar/cloud_unstructured_fullframe");
    EXPECT_EQ(d.perception_stereo_detections_2d, "/deos/perception/stereo/detections_2d");
    EXPECT_EQ(d.perception_detections_3d, "/deos/perception/detections_3d");
    EXPECT_EQ(d.perception_detections_3d_markers, "/deos/perception/detections_3d_markers");
    EXPECT_EQ(d.localization_odom_icp, "/odometry/icp");
    EXPECT_EQ(d.control_cmd_vel, "/deos/control/cmd_vel");
    EXPECT_EQ(d.as_map().size(), 52u);

    // Trailing slash ve boşluk temizliği Python rstrip("/") ile aynı davranmalı
    const auto d_slash = deos::build_deos_topics(" /arac1/// ");
    EXPECT_EQ(d_slash.sensors_imu, "/arac1/sensors/imu/data");
    EXPECT_EQ(d_slash.localization_odom_icp, "/odometry/icp");
}

TEST(ZeroCopyTraitsTest, DistinguishesFixedSizePodFromDynamicRosMessages) {
    EXPECT_TRUE((deos::is_shm_zero_copy_capable_v<deos_shm_msgs::msg::FixedFrame640x480>));
    EXPECT_TRUE((deos::is_shm_zero_copy_capable_v<deos_shm_msgs::msg::FixedPointCloud64k>));

    // Image ve PointCloud2 icindeki std_msgs/Header (string) ve data[] nedeniyle dinamik boyutludur
    EXPECT_FALSE((deos::is_shm_zero_copy_capable_v<sensor_msgs::msg::Image>));
    EXPECT_FALSE((deos::is_shm_zero_copy_capable_v<sensor_msgs::msg::PointCloud2>));
}

TEST(ZeroCopyHelpersTest, FixedFrameIdRoundtripAndTruncation) {
    std::array<std::uint8_t, 8> buf{};
    deos::set_fixed_frame_id(buf, "laser");
    EXPECT_EQ(deos::get_fixed_frame_id(buf), "laser");

    deos::set_fixed_frame_id(buf, "camera_color_optical_frame_very_long");
    EXPECT_EQ(deos::get_fixed_frame_id(buf), "camera_");
    EXPECT_EQ(buf.back(), 0u);
}

TEST(ZeroCopyPublishTest, IntraProcessDoesNotThrowAndTransfersPointerZeroCopy) {
    if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
    }

    rclcpp::NodeOptions opts;
    opts.use_intra_process_comms(true);
    auto node = std::make_shared<rclcpp::Node>("test_zero_copy_node", opts);

    const std::uint8_t * pub_ptr = nullptr;
    const std::uint8_t * sub_ptr = nullptr;

    auto sub = node->create_subscription<sensor_msgs::msg::Image>(
        "/test/intra_img", deos::qos::sensor_data(5),
        [&](sensor_msgs::msg::Image::ConstSharedPtr msg) {
            sub_ptr = msg->data.data();
        });
    auto pub = node->create_publisher<sensor_msgs::msg::Image>(
        "/test/intra_img", deos::qos::sensor_data(5));

    const auto path = deos::publish_zero_copy(*pub, true, [&](sensor_msgs::msg::Image & m) {
        m.data.resize(1024, 42u);
        pub_ptr = m.data.data();
    });
    EXPECT_EQ(path, deos::PublishPath::IntraProcessUniquePtr);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    for (int i = 0; i < 20 && sub_ptr == nullptr; ++i) {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    ASSERT_NE(sub_ptr, nullptr);
    EXPECT_EQ(pub_ptr, sub_ptr) << "Intra-process unique_ptr yayininda veri adresi degismemeli";
}

}  // namespace
