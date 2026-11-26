/*
 * DEOS CycloneDDS + iceoryx Shared Memory & Intra-Process Zero-Copy Benchmark Aracı
 *
 * 640x480 BGR8 (~921.6 KB) gerçek kamera karesi boyutunda 3 farklı aktarım yolunu ölçer:
 *   1. dds_serialized   : sensor_msgs/Image (dinamik boyutlu, serileştirme + kopya)
 *   2. iceoryx_loaned   : deos_shm_msgs/FixedFrame640x480 (POD, borrow_loaned_message)
 *   3. intra_process    : sensor_msgs/Image + use_intra_process_comms(true) + unique_ptr
 *
 * Kullanım:
 *   # 1) SHM kapalıyken (varsayılan cyclonedds.xml):
 *   ros2 run deos_cpp_common shm_zero_copy_bench --samples 60
 *
 *   # 2) SHM açıkken (iox-roudi + cyclonedds_shm.xml):
 *   DEOS_DDS_SHM=1 bash -c "source /ros2_ws/ops/scripts/deos_dds_env.sh && \
 *     ros2 run deos_cpp_common shm_zero_copy_bench --samples 60"
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <numeric>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <deos_shm_msgs/msg/fixed_frame640x480.hpp>

#include "deos_cpp_common/qos.hpp"
#include "deos_cpp_common/zero_copy.hpp"

namespace {

using Clock = std::chrono::steady_clock;
constexpr std::size_t kFrameBytes = 640u * 480u * 3u;

struct Stats {
    std::string mode;
    std::string active_path;
    bool can_loan{false};
    bool same_address{false};
    std::size_t received{0};
    double mean_us{0.0};
    double p50_us{0.0};
    double p99_us{0.0};
};

std::uint64_t now_ns() noexcept {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            Clock::now().time_since_epoch()).count());
}

Stats summarize(
    std::string mode,
    std::string active_path,
    bool can_loan,
    bool same_address,
    std::vector<double> samples_us) {
    Stats s;
    s.mode = std::move(mode);
    s.active_path = std::move(active_path);
    s.can_loan = can_loan;
    s.same_address = same_address;
    s.received = samples_us.size();
    if (samples_us.empty()) {
        return s;
    }
    std::sort(samples_us.begin(), samples_us.end());
    const double sum = std::accumulate(samples_us.begin(), samples_us.end(), 0.0);
    s.mean_us = sum / static_cast<double>(samples_us.size());
    s.p50_us = samples_us[samples_us.size() / 2];
    s.p99_us = samples_us[(samples_us.size() * 99) / 100];
    return s;
}

Stats run_dynamic_image_bench(std::size_t count, bool intra_process) {
    rclcpp::NodeOptions opts;
    opts.use_intra_process_comms(intra_process);

    auto pub_node = std::make_shared<rclcpp::Node>(
        intra_process ? "bench_intra_pub" : "bench_dds_pub", opts);
    auto sub_node = std::make_shared<rclcpp::Node>(
        intra_process ? "bench_intra_sub" : "bench_dds_sub", opts);

    const std::string topic = intra_process ? "/deos/bench/image_intra" : "/deos/bench/image_dds";
    auto qos = deos::qos::shm_loaned_reliable(10);

    std::vector<double> latencies_us;
    latencies_us.reserve(count);
    std::atomic<const std::uint8_t *> last_pub_data{nullptr};
    std::atomic<bool> ptr_matched{false};

    auto sub = sub_node->create_subscription<sensor_msgs::msg::Image>(
        topic, qos,
        [&](sensor_msgs::msg::Image::ConstSharedPtr msg) {
            const std::uint64_t t_rx = now_ns();
            std::uint64_t t_tx = 0;
            std::memcpy(&t_tx, msg->data.data(), sizeof(t_tx));
            if (msg->data.data() == last_pub_data.load()) {
                ptr_matched.store(true);
            }
            latencies_us.push_back(static_cast<double>(t_rx - t_tx) / 1000.0);
        });

    auto pub = pub_node->create_publisher<sensor_msgs::msg::Image>(topic, qos);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(pub_node);
    exec.add_node(sub_node);

    // Discovery ısınması
    for (int i = 0; i < 10 && pub->get_subscription_count() == 0; ++i) {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    deos::PublishPath used_path = deos::PublishPath::HeapSerialized;
    for (std::size_t i = 0; i < count; ++i) {
        used_path = deos::publish_zero_copy(*pub, intra_process, [&](sensor_msgs::msg::Image & m) {
            m.header.frame_id = "camera_color_frame";
            m.width = 640;
            m.height = 480;
            m.encoding = "bgr8";
            m.step = 640 * 3;
            m.data.resize(kFrameBytes, static_cast<std::uint8_t>(i & 0xFFu));
            last_pub_data.store(m.data.data());
            const std::uint64_t t_tx = now_ns();
            std::memcpy(m.data.data(), &t_tx, sizeof(t_tx));
        });

        const auto deadline = Clock::now() + std::chrono::milliseconds(200);
        const std::size_t target = i + 1;
        while (latencies_us.size() < target && Clock::now() < deadline) {
            exec.spin_some();
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    return summarize(
        intra_process ? "3. Intra-Process (Image 921KB)" : "1. Standart DDS  (Image 921KB)",
        std::string{deos::to_string(used_path)},
        pub->can_loan_messages(),
        ptr_matched.load(),
        std::move(latencies_us));
}

Stats run_fixed_shm_bench(std::size_t count) {
    using FixedMsg = deos_shm_msgs::msg::FixedFrame640x480;
    static_assert(deos::is_shm_zero_copy_capable_v<FixedMsg>,
                  "FixedFrame640x480 sabit boyutlu olmalı");

    rclcpp::NodeOptions opts;
    opts.use_intra_process_comms(false);  // Süreçler arası yolu (iceoryx SHM) zorla

    auto pub_node = std::make_shared<rclcpp::Node>("bench_shm_pub", opts);
    auto sub_node = std::make_shared<rclcpp::Node>("bench_shm_sub", opts);

    const std::string topic = "/deos/bench/fixed_frame_shm";
    auto qos = deos::qos::shm_loaned_reliable(10);

    std::vector<double> latencies_us;
    latencies_us.reserve(count);

    auto sub = sub_node->create_subscription<FixedMsg>(
        topic, qos,
        [&](FixedMsg::ConstSharedPtr msg) {
            const std::uint64_t t_rx = now_ns();
            latencies_us.push_back(static_cast<double>(t_rx - msg->stamp_ns) / 1000.0);
        });

    auto pub = pub_node->create_publisher<FixedMsg>(topic, qos);

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(pub_node);
    exec.add_node(sub_node);

    for (int i = 0; i < 10 && pub->get_subscription_count() == 0; ++i) {
        exec.spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    deos::PublishPath used_path = deos::PublishPath::HeapSerialized;
    for (std::size_t i = 0; i < count; ++i) {
        used_path = deos::publish_zero_copy(*pub, false, [&](FixedMsg & m) {
            m.seq = static_cast<std::uint32_t>(i);
            m.width = 640;
            m.height = 480;
            m.channels = 3;
            deos::set_fixed_frame_id(m.frame_id, "camera_color_frame");
            m.data[0] = static_cast<std::uint8_t>(i & 0xFFu);
            m.data[kFrameBytes - 1] = static_cast<std::uint8_t>(i & 0xFFu);
            m.stamp_ns = now_ns();
        });

        const auto deadline = Clock::now() + std::chrono::milliseconds(200);
        const std::size_t target = i + 1;
        while (latencies_us.size() < target && Clock::now() < deadline) {
            exec.spin_some();
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    return summarize(
        "2. Sabit Mesaj   (Fixed 921KB)",
        std::string{deos::to_string(used_path)},
        pub->can_loan_messages(),
        false,
        std::move(latencies_us));
}

}  // namespace

int main(int argc, char ** argv) {
    std::size_t samples = 50;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--samples") == 0 && i + 1 < argc) {
            samples = static_cast<std::size_t>(std::max(5, std::atoi(argv[++i])));
        }
    }

    rclcpp::init(argc, argv);

    const char * rmw = std::getenv("RMW_IMPLEMENTATION");
    const char * uri = std::getenv("CYCLONEDDS_URI");
    std::printf("====================================================================================\n");
    std::printf(" DEOS Zero-Copy Benchmark | Kare boyutu: %.1f KB (640x480 BGR8) | Ornek: %zu\n",
                static_cast<double>(kFrameBytes) / 1024.0, samples);
    std::printf(" RMW_IMPLEMENTATION : %s\n", rmw ? rmw : "(varsayilan)");
    std::printf(" CYCLONEDDS_URI     : %s\n", uri ? uri : "(tanimsiz)");
    std::printf("====================================================================================\n");

    const Stats s1 = run_dynamic_image_bench(samples, false);
    const Stats s2 = run_fixed_shm_bench(samples);
    const Stats s3 = run_dynamic_image_bench(samples, true);

    std::printf("%-32s | %-24s | %-8s | %8s | %8s | %8s\n",
                "Mod", "Aktif Yol", "LoanSHM?", "Ort(us)", "P50(us)", "P99(us)");
    std::printf("------------------------------------------------------------------------------------\n");
    for (const auto & s : {s1, s2, s3}) {
        std::printf("%-32s | %-24s | %-8s | %8.1f | %8.1f | %8.1f  (n=%zu%s)\n",
                    s.mode.c_str(),
                    s.active_path.c_str(),
                    s.can_loan ? "EVET" : "HAYIR",
                    s.mean_us,
                    s.p50_us,
                    s.p99_us,
                    s.received,
                    s.same_address ? ", ayni_ptr=EVET" : "");
    }
    std::printf("====================================================================================\n");

    rclcpp::shutdown();
    return (s1.received > 0 && s2.received > 0 && s3.received > 0) ? 0 : 1;
}
