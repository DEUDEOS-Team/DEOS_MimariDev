#ifndef DEOS_CPP_COMMON__QOS_HPP_
#define DEOS_CPP_COMMON__QOS_HPP_

#include <cstddef>

#include <rclcpp/qos.hpp>

namespace deos::qos {

/**
 * @brief Yüksek frekanslı / büyük boyutlu sensör akışları (Image, PointCloud2, Detection2DArray).
 *
 * BEST_EFFORT + VOLATILE + KEEP_LAST:
 *  - Geç kalan paketlerin yeniden iletimi (retransmission) yüzünden kuyruk şişmesini önler.
 *  - RELIABLE yayın yapan mevcut Python/SICK sürücüleriyle de DDS QoS kuralları gereği uyumludur
 *    (RELIABLE publisher -> BEST_EFFORT subscriber eşleşir).
 *  - CycloneDDS + iceoryx shared memory loaned mesajlarıyla tam uyumludur.
 */
[[nodiscard]] inline rclcpp::QoS sensor_data(std::size_t depth = 5) {
    rclcpp::SensorDataQoS qos;
    qos.keep_last(depth);
    return qos;
}

/**
 * @brief Sabit boyutlu mesajlarda (deos_shm_msgs) iceoryx loaned zero-copy ile RELIABLE aktarım.
 *
 * Önemli: rmw_cyclonedds_cpp + iceoryx'te loaned message kullanabilmek için
 * Durability = VOLATILE ve History = KEEP_LAST olmalıdır.
 */
[[nodiscard]] inline rclcpp::QoS shm_loaned_reliable(std::size_t depth = 5) {
    return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().durability_volatile();
}

/**
 * @brief Kontrol, planlama, failsafe ve 3D algı çıktıları (cmd_vel, emergency_stop, detections_3d).
 * micro-ROS (Fast DDS) ve rosbag kayıtlarıyla doğrudan eşleşir.
 */
[[nodiscard]] inline rclcpp::QoS control_reliable(std::size_t depth = 10) {
    return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().durability_volatile();
}

/**
 * @brief Seyrek değişen durum/konfigürasyon mesajları (harita, statik durum).
 * Not: TRANSIENT_LOCAL seçildiğinde iceoryx zero-copy yerine standart DDS geçmişi kullanılır.
 */
[[nodiscard]] inline rclcpp::QoS latched_state(std::size_t depth = 1) {
    return rclcpp::QoS(rclcpp::KeepLast(depth)).reliable().transient_local();
}

}  // namespace deos::qos

#endif  // DEOS_CPP_COMMON__QOS_HPP_
