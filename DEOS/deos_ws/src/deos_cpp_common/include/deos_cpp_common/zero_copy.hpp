#ifndef DEOS_CPP_COMMON__ZERO_COPY_HPP_
#define DEOS_CPP_COMMON__ZERO_COPY_HPP_

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string_view>
#include <type_traits>
#include <utility>

#include <rclcpp/publisher.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

namespace deos {

/**
 * @brief Bir ROS 2 mesaj tipinin CycloneDDS + iceoryx ile süreçler arası (inter-process)
 * gerçek zero-copy (borrow_loaned_message) destekleyip desteklemediğini derleme zamanında sınar.
 *
 * Kural: Mesaj içinde `string` veya sınırsız `T[]` (ör. std_msgs/Header, sensor_msgs/Image,
 * sensor_msgs/PointCloud2) varsa `has_fixed_size<MsgT>::value == false` döner.
 * Bu mesajlar için zero-copy yolu: aynı `ComposableNodeContainer` içinde `use_intra_process_comms(true)`
 * ve `std::unique_ptr<MsgT>` ile publish etmektir.
 */
template <typename MsgT>
inline constexpr bool is_shm_zero_copy_capable_v =
    rosidl_generator_traits::has_fixed_size<MsgT>::value;

enum class PublishPath : std::uint8_t {
    IceoryxLoaned = 0,          // /dev/shm üzerinde gerçek süreçler arası zero-copy
    IntraProcessUniquePtr = 1,  // Aynı container içinde std::unique_ptr devri (0 kopya)
    HeapSerialized = 2          // Standart DDS serileştirme (UDP veya SHM transport)
};

[[nodiscard]] constexpr std::string_view to_string(PublishPath path) noexcept {
    switch (path) {
        case PublishPath::IceoryxLoaned:
            return "iceoryx_loaned_shm";
        case PublishPath::IntraProcessUniquePtr:
            return "intra_process_unique_ptr";
        case PublishPath::HeapSerialized:
            return "dds_serialized";
    }
    return "unknown";
}

/**
 * @brief Sabit boyutlu mesajlardaki `uint8[N] frame_id` alanına güvenli (null-sonlandırmalı)
 * frame adı yazar. `std_msgs/Header` kullanmadan frame bilgisini POD içinde taşır.
 */
template <std::size_t N>
constexpr void set_fixed_frame_id(std::array<std::uint8_t, N> & dst, std::string_view src) noexcept {
    static_assert(N >= 2, "Sabit frame_id dizisi en az 2 byte olmalı");
    dst.fill(0);
    const std::size_t n = std::min(src.size(), N - 1);
    for (std::size_t i = 0; i < n; ++i) {
        dst[i] = static_cast<std::uint8_t>(src[i]);
    }
}

template <std::size_t N>
[[nodiscard]] inline std::string_view get_fixed_frame_id(
    const std::array<std::uint8_t, N> & src) noexcept {
    const auto * begin = reinterpret_cast<const char *>(src.data());
    std::size_t len = 0;
    while (len < N && begin[len] != '\0') {
        ++len;
    }
    return std::string_view{begin, len};
}

/**
 * @brief Hem intra-process hem de iceoryx shared memory ile güvenli zero-copy yayın yardımcısı.
 *
 * Neden doğrudan `pub.borrow_loaned_message()` çağrılmamalı?
 *   ROS 2 Jazzy `rclcpp`'de bir node `use_intra_process_comms(true)` ile açılmışsa,
 *   `pub.can_loan_messages()` RMW seviyesinde `true` dönse bile `borrow_loaned_message()`
 *   çalışma zamanında istisna (exception) fırlatır ("Borrowing loaned messages is not supported
 *   with intra-process communication").
 *
 * Bu fonksiyon:
 *   1. Mesaj sabit boyutluysa (`is_shm_zero_copy_capable_v<MsgT>`), intra-process kapalıysa
 *      ve RouDi aktifse (`pub.can_loan_messages()`): doğrudan `/dev/shm` chunk'ını ödünç alır,
 *      `populate(MsgT&)` ile yerinde doldurur ve kopyasız yayınlar.
 *   2. Aksi halde `std::make_unique<MsgT>()` ayırır, `populate(*msg)` çağırır ve `std::move(msg)`
 *      ile yayınlar (intra-process açıksa pointer devriyle yine 0 kopya çalışır).
 */
template <typename MsgT, typename PopulateFn>
PublishPath publish_zero_copy(
    rclcpp::Publisher<MsgT> & pub,
    bool intra_process_enabled,
    PopulateFn && populate) {
    if constexpr (is_shm_zero_copy_capable_v<MsgT>) {
        if (!intra_process_enabled && pub.can_loan_messages()) {
            auto loaned = pub.borrow_loaned_message();
            std::forward<PopulateFn>(populate)(loaned.get());
            pub.publish(std::move(loaned));
            return PublishPath::IceoryxLoaned;
        }
    }

    auto msg = std::make_unique<MsgT>();
    std::forward<PopulateFn>(populate)(*msg);
    pub.publish(std::move(msg));
    return intra_process_enabled ? PublishPath::IntraProcessUniquePtr
                                 : PublishPath::HeapSerialized;
}

}  // namespace deos

#endif  // DEOS_CPP_COMMON__ZERO_COPY_HPP_
