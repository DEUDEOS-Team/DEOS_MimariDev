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

#ifndef LICAM_FUSION__FUSION_CORE_HPP_
#define LICAM_FUSION__FUSION_CORE_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <optional>
#include <span>
#include <type_traits>
#include <utility>
#include <vector>

namespace licam_fusion::core {

// =============================================================================
// 1. Frame-etiketli güçlü tipler (Strong Types) — derleme zamanı frame güvenliği
// =============================================================================
struct LidarFrame {};
struct CameraFrame {};
struct BaseLink {};

template <typename Frame>
struct Point3 {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

template <typename To, typename From>
struct Transform {
    std::array<float, 9> r{1.0f, 0.0f, 0.0f,
                           0.0f, 1.0f, 0.0f,
                           0.0f, 0.0f, 1.0f};
    std::array<float, 3> t{0.0f, 0.0f, 0.0f};

    [[nodiscard]] constexpr Point3<To> operator*(const Point3<From>& p) const noexcept {
        return {
            r[0] * p.x + r[1] * p.y + r[2] * p.z + t[0],
            r[3] * p.x + r[4] * p.y + r[5] * p.z + t[1],
            r[6] * p.x + r[7] * p.y + r[8] * p.z + t[2]
        };
    }

    template <typename Mid>
    [[nodiscard]] constexpr Transform<To, Mid> operator*(
        const Transform<From, Mid>& o) const noexcept {
        Transform<To, Mid> out{};
        for (std::size_t i = 0; i < 3; ++i) {
            for (std::size_t j = 0; j < 3; ++j) {
                float s = 0.0f;
                for (std::size_t k = 0; k < 3; ++k) {
                    s += r[i * 3 + k] * o.r[k * 3 + j];
                }
                out.r[i * 3 + j] = s;
            }
            out.t[i] = r[i * 3 + 0] * o.t[0] +
                       r[i * 3 + 1] * o.t[1] +
                       r[i * 3 + 2] * o.t[2] + t[i];
        }
        return out;
    }
};

using T_cam_lidar = Transform<CameraFrame, LidarFrame>;
static_assert(sizeof(Point3<LidarFrame>) == 12, "Point3 boyutu 12 byte olmalı");
static_assert(!std::is_convertible_v<Point3<LidarFrame>, Point3<CameraFrame>>,
              "LidarFrame noktası CameraFrame noktasına gizlice dönüşemez");

// =============================================================================
// 2. Exception'sız açık hata yönetimi (C++20 Expected)
// =============================================================================
template <typename E>
struct Unexpected {
    E error;
    constexpr explicit Unexpected(E e) noexcept : error(e) {}
};

template <typename T, typename E>
class Expected {
public:
    // NOLINTBEGIN(cppcoreguidelines-explicit-constructor)
    constexpr Expected(const T& val) noexcept : val_(val), has_val_(true) {}
    constexpr Expected(T&& val) noexcept : val_(std::move(val)), has_val_(true) {}
    constexpr Expected(Unexpected<E> u) noexcept : err_(u.error) {}
    // NOLINTEND(cppcoreguidelines-explicit-constructor)

    [[nodiscard]] constexpr bool has_value() const noexcept { return has_val_; }
    [[nodiscard]] constexpr explicit operator bool() const noexcept { return has_val_; }
    [[nodiscard]] constexpr const T& value() const noexcept { return val_; }
    [[nodiscard]] constexpr const T& operator*() const noexcept { return val_; }
    [[nodiscard]] constexpr const T* operator->() const noexcept { return &val_; }
    [[nodiscard]] constexpr E error() const noexcept { return err_; }

private:
    T val_{};
    E err_{};
    bool has_val_{false};
};

enum class CloudError : std::uint8_t {
    MissingField = 0,
    NotFloat32 = 1,
    BigEndian = 2,
    SizeMismatch = 3,
    NullData = 4
};

// =============================================================================
// 3. Sabit kapasiteli RingBuffer — gecikme ve senkronizasyon istatistikleri için
// =============================================================================
template <typename T, std::size_t N>
class RingBuffer {
    static_assert(N > 0, "RingBuffer kapasitesi > 0 olmalı");

public:
    constexpr void push(const T& value) noexcept {
        data_[head_] = value;
        head_ = (head_ + 1) % N;
        if (count_ < N) {
            ++count_;
        }
    }

    constexpr void clear() noexcept {
        head_ = 0;
        count_ = 0;
    }

    [[nodiscard]] constexpr std::size_t size() const noexcept { return count_; }
    [[nodiscard]] constexpr bool empty() const noexcept { return count_ == 0; }
    [[nodiscard]] constexpr bool full() const noexcept { return count_ == N; }

    [[nodiscard]] constexpr const T& operator[](std::size_t i) const noexcept {
        return data_[(head_ + N - count_ + i) % N];
    }

    [[nodiscard]] constexpr const T& latest() const noexcept { return (*this)[count_ - 1]; }
    [[nodiscard]] constexpr const T& oldest() const noexcept { return (*this)[0]; }

    [[nodiscard]] constexpr double mean() const noexcept {
        if (count_ == 0) {
            return 0.0;
        }
        double sum = 0.0;
        for (std::size_t i = 0; i < count_; ++i) {
            sum += static_cast<double>((*this)[i]);
        }
        return sum / static_cast<double>(count_);
    }

    [[nodiscard]] T percentile99() const noexcept {
        if (count_ == 0) {
            return T{};
        }
        std::array<T, N> tmp{};
        for (std::size_t i = 0; i < count_; ++i) {
            tmp[i] = (*this)[i];
        }
        const std::size_t idx = (count_ * 99) / 100;
        auto nth = tmp.begin() + static_cast<std::ptrdiff_t>(idx);
        std::nth_element(tmp.begin(), nth, tmp.begin() + static_cast<std::ptrdiff_t>(count_));
        return *nth;
    }

private:
    std::array<T, N> data_{};
    std::size_t head_{0};
    std::size_t count_{0};
};

static_assert([] {
    RingBuffer<int, 3> rb;
    for (int i = 1; i <= 5; ++i) {
        rb.push(i);
    }
    return rb.oldest() == 3 && rb.latest() == 5 && rb.size() == 3;
}(), "RingBuffer derleme zamanı testi");

// =============================================================================
// 4. Kamera modeli, projeksiyon matrisi ve veri yapıları
// =============================================================================
struct Intrinsics {
    float fx{0.0f};
    float fy{0.0f};
    float cx{0.0f};
    float cy{0.0f};
    float tx{0.0f};  // Stereo P[3] terimi (fx * baseline)
    float ty{0.0f};  // Stereo P[7] terimi
    std::int32_t width{0};
    std::int32_t height{0};
    // Plumb-bob lens distorsiyon katsayıları (k1, k2, p1, p2, k3)
    std::array<float, 5> dist{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    bool apply_distortion{false};
};

// P = K * [R | t] + [tx, ty, 0]^T  (LiDAR noktası -> homojen piksel)
struct Projection {
    std::array<float, 12> m{};   // 3x4 satır-öncelikli projeksiyon matrisi
    std::array<float, 9> r{};    // 3x3 kamera rotasyonu (distorsiyon aktifse kullanılır)
    std::array<float, 3> t{};    // 3x1 kamera ötelemesi

    [[nodiscard]] static constexpr Projection from(
        const Intrinsics& k,
        const std::array<float, 9>& rot,
        const std::array<float, 3>& trans) noexcept {
        Projection p{};
        p.r = rot;
        p.t = trans;
        for (std::size_t c = 0; c < 3; ++c) {
            p.m[c]      = k.fx * rot[c]      + k.cx * rot[6u + c];
            p.m[4u + c] = k.fy * rot[3u + c] + k.cy * rot[6u + c];
            p.m[8u + c] = rot[6u + c];
        }
        p.m[3]  = k.fx * trans[0] + k.cx * trans[2] + k.tx;
        p.m[7]  = k.fy * trans[1] + k.cy * trans[2] + k.ty;
        p.m[11] = trans[2];
        return p;
    }
};

static_assert([] {
    constexpr Intrinsics k{500.0f, 500.0f, 320.0f, 240.0f, 0.0f, 0.0f, 640, 480};
    constexpr std::array<float, 9> I{1.0f, 0.0f, 0.0f,
                                     0.0f, 1.0f, 0.0f,
                                     0.0f, 0.0f, 1.0f};
    constexpr std::array<float, 3> zero{0.0f, 0.0f, 0.0f};
    constexpr auto P = Projection::from(k, I, zero);
    return P.m[0] == 500.0f && P.m[2] == 320.0f && P.m[5] == 500.0f && P.m[6] == 240.0f && P.m[10] == 1.0f;
}(), "Projection::from derleme zamanı testi");

// İsteğe bağlı 2D segmentasyon maskesi görünümü (kutu içi ikili maske)
struct InstanceMaskView {
    const std::uint8_t* data{nullptr};
    std::int32_t width{0};
    std::int32_t height{0};

    [[nodiscard]] constexpr bool valid() const noexcept {
        return data != nullptr && width > 0 && height > 0;
    }

    [[nodiscard]] constexpr bool contains(float rel_x, float rel_y) const noexcept {
        if (!valid()) {
            return true;
        }
        const auto mx = static_cast<std::int32_t>(rel_x * static_cast<float>(width));
        const auto my = static_cast<std::int32_t>(rel_y * static_cast<float>(height));
        if (mx < 0 || mx >= width || my < 0 || my >= height) {
            return false;
        }
        return data[static_cast<std::size_t>(my) * static_cast<std::size_t>(width) +
                    static_cast<std::size_t>(mx)] != 0;
    }
};

struct BoxPx {
    float x_min{0.0f};
    float y_min{0.0f};
    float x_max{0.0f};
    float y_max{0.0f};
    float prior_size_x{0.0f};
    float prior_size_y{0.0f};
    float prior_size_z{0.0f};
    InstanceMaskView mask{};
};

struct CloudView {
    const std::byte* data{nullptr};
    std::size_t count{0};
    std::size_t point_step{0};
    std::size_t off_x{0};
    std::size_t off_y{4};
    std::size_t off_z{8};
};

struct Params {
    std::uint32_t min_points{5};
    float cluster_tolerance{1.2f};
    float min_depth{0.2f};
    float max_depth{100.0f};
    float box_shrink{0.9f};
    float min_depth_spread{0.3f};
    float bin_size{0.1f};
    bool occlusion_handling{true};
    bool filter_ground{false};
    float max_camera_y{1.8f};       // Kamera optik frame'inde +Y aşağıdır; bu değeri aşan zemin noktaları elenir
    float class_prior_weight{0.0f}; // [0..1]: ölçülen küme boyutu ile sınıf önbilgisini harmanlama ağırlığı
    std::uint32_t grid_box_threshold{24};
};

struct Object3D {
    std::uint32_t box{0};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    float size_x{0.0f};
    float size_y{0.0f};
    float size_z{0.0f};
    std::array<float, 3> pos_variance{0.01f, 0.01f, 0.04f};
    std::uint32_t num_points{0};
};

[[nodiscard]] inline float load_f32(const std::byte* p) noexcept {
    float f = 0.0f;
    std::memcpy(&f, p, sizeof(f));
    return f;
}

static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559,
              "PointCloud2 FLOAT32 alanları IEEE-754 float bekler");

// =============================================================================
// 5. FusionCore — Sıfır runtime heap allocation'lı füzyon motoru
// =============================================================================
class FusionCore final {
public:
    FusionCore(std::size_t expected_hits, std::size_t max_boxes, const Params& prm) {
        reserve_storage(expected_hits, max_boxes);
        configure(prm);
    }

    void reserve_storage(std::size_t expected_hits, std::size_t max_boxes) {
        hits_.reserve(expected_hits);
        bucketed_.reserve(expected_hits);
        scratch_.reserve(expected_hits);
        boxes_.reserve(max_boxes);
        box_areas_.reserve(max_boxes);
        claimed_depth_.reserve(max_boxes);
        offsets_.reserve(max_boxes + 1);
        cursor_.reserve(max_boxes);
        objects_.reserve(max_boxes);
        grid_cells_.resize(kGridCols * kGridRows);
        for (auto& cell : grid_cells_) {
            cell.reserve(16);
        }
    }

    void configure(const Params& prm) {
        prm_ = prm;
        const float safe_bin = std::max(0.01f, prm.bin_size);
        prm_.bin_size = safe_bin;
        const auto num_bins = static_cast<std::size_t>(std::ceil(prm.max_depth / safe_bin)) + 2;
        hist_.assign(num_bins, 0u);
        primary_hist_.assign(num_bins, 0u);
        gap_bins_ = static_cast<std::uint32_t>(std::ceil(prm.cluster_tolerance / safe_bin));
    }

    [[nodiscard]] const Params& params() const noexcept { return prm_; }

    [[nodiscard]] std::span<const Object3D> process(
        const Projection& proj,
        const Intrinsics& k,
        std::span<const BoxPx> boxes,
        const CloudView& cloud) {
        hits_.clear();
        objects_.clear();
        if (boxes.empty() || cloud.data == nullptr || cloud.count == 0 || k.width <= 0 || k.height <= 0) {
            return {};
        }

        shrink_boxes(boxes);
        associate(proj, k, cloud);
        if (hits_.empty()) {
            return {};
        }

        bucket_by_box();

        // 1. Geçiş: Her kutunun birincil (kendine özgü / en dar kutu) derinlik kümesini bul
        claimed_depth_.assign(boxes_.size(), -1.0f);
        if (prm_.occlusion_handling && boxes_.size() > 1) {
            for (std::uint32_t b = 0; b < boxes_.size(); ++b) {
                const auto seg = std::span{bucketed_}.subspan(
                    offsets_[b], offsets_[b + 1] - offsets_[b]);
                claimed_depth_[b] = estimate_primary_depth(b, seg);
            }
        }

        // 2. Geçiş: Örtüşen ön plan kümelerini dışlayarak 3D nesneleri inşa et
        for (std::uint32_t b = 0; b < boxes_.size(); ++b) {
            const auto seg = std::span{bucketed_}.subspan(
                offsets_[b], offsets_[b + 1] - offsets_[b]);
            build_object(b, seg, k);
        }
        return objects_;
    }

private:
    static constexpr std::size_t kGridCols = 32;
    static constexpr std::size_t kGridRows = 32;

    struct Hit {
        std::uint32_t box;
        std::uint32_t tightest_box;
        float depth;
        float u;
        float v;
    };

    void shrink_boxes(std::span<const BoxPx> in) {
        boxes_.clear();
        box_areas_.clear();
        const float s = std::clamp(prm_.box_shrink, 0.1f, 1.0f);
        for (const auto& b : in) {
            const float cx = 0.5f * (b.x_min + b.x_max);
            const float cy = 0.5f * (b.y_min + b.y_max);
            const float hw = 0.5f * s * std::max(1.0f, b.x_max - b.x_min);
            const float hh = 0.5f * s * std::max(1.0f, b.y_max - b.y_min);
            boxes_.push_back({cx - hw, cy - hh, cx + hw, cy + hh,
                              b.prior_size_x, b.prior_size_y, b.prior_size_z, b.mask});
            box_areas_.push_back((2.0f * hw) * (2.0f * hh));
        }
    }

    void build_spatial_grid(float w_img, float h_img) {
        for (auto& cell : grid_cells_) {
            cell.clear();
        }
        const float inv_cell_w = static_cast<float>(kGridCols) / w_img;
        const float inv_cell_h = static_cast<float>(kGridRows) / h_img;
        for (std::uint32_t b = 0; b < boxes_.size(); ++b) {
            const auto& bx = boxes_[b];
            const auto c0 = static_cast<std::size_t>(
                std::clamp(static_cast<int>(bx.x_min * inv_cell_w), 0, static_cast<int>(kGridCols - 1)));
            const auto c1 = static_cast<std::size_t>(
                std::clamp(static_cast<int>(bx.x_max * inv_cell_w), 0, static_cast<int>(kGridCols - 1)));
            const auto r0 = static_cast<std::size_t>(
                std::clamp(static_cast<int>(bx.y_min * inv_cell_h), 0, static_cast<int>(kGridRows - 1)));
            const auto r1 = static_cast<std::size_t>(
                std::clamp(static_cast<int>(bx.y_max * inv_cell_h), 0, static_cast<int>(kGridRows - 1)));
            for (std::size_t r = r0; r <= r1; ++r) {
                for (std::size_t c = c0; c <= c1; ++c) {
                    grid_cells_[r * kGridCols + c].push_back(b);
                }
            }
        }
    }

    [[nodiscard]] static constexpr bool point_in_box(const BoxPx& bx, float u, float v) noexcept {
        if (!(u >= bx.x_min && u <= bx.x_max && v >= bx.y_min && v <= bx.y_max)) {
            return false;
        }
        if (bx.mask.valid()) {
            const float rel_x = (u - bx.x_min) / std::max(1.0f, bx.x_max - bx.x_min);
            const float rel_y = (v - bx.y_min) / std::max(1.0f, bx.y_max - bx.y_min);
            return bx.mask.contains(rel_x, rel_y);
        }
        return true;
    }

    void associate(const Projection& p, const Intrinsics& k, const CloudView& c) {
        const auto& m = p.m;
        const auto w_img = static_cast<float>(k.width);
        const auto h_img = static_cast<float>(k.height);
        const float dmin = prm_.min_depth;
        const float dmax = prm_.max_depth;
        const bool use_grid = boxes_.size() >= prm_.grid_box_threshold;
        if (use_grid) {
            build_spatial_grid(w_img, h_img);
        }
        const float inv_cell_w = static_cast<float>(kGridCols) / w_img;
        const float inv_cell_h = static_cast<float>(kGridRows) / h_img;

        const std::byte* ptr = c.data;
        for (std::size_t i = 0; i < c.count; ++i, ptr += c.point_step) {
            const float x = load_f32(ptr + c.off_x);
            const float y = load_f32(ptr + c.off_y);
            const float z = load_f32(ptr + c.off_z);

            const float w = m[8] * x + m[9] * y + m[10] * z + m[11];
            // NaN ile her karşılaştırma false döner -> std::isfinite çağrısına gerek kalmaz
            if (!(w > dmin && w < dmax)) {
                continue;
            }

            const float inv_w = 1.0f / w;
            float u = 0.0f;
            float v = 0.0f;

            if (!k.apply_distortion) [[likely]] {
                u = (m[0] * x + m[1] * y + m[2] * z + m[3]) * inv_w;
                v = (m[4] * x + m[5] * y + m[6] * z + m[7]) * inv_w;
                if (prm_.filter_ground) {
                    const float y_cam = (v - k.cy) * w / k.fy;
                    if (y_cam > prm_.max_camera_y) {
                        continue;
                    }
                }
            } else {
                const float xc = p.r[0] * x + p.r[1] * y + p.r[2] * z + p.t[0];
                const float yc = p.r[3] * x + p.r[4] * y + p.r[5] * z + p.t[1];
                if (prm_.filter_ground && yc > prm_.max_camera_y) {
                    continue;
                }
                const float xn = xc * inv_w;
                const float yn = yc * inv_w;
                const float r2 = xn * xn + yn * yn;
                const float r4 = r2 * r2;
                const float r6 = r4 * r2;
                const float radial = 1.0f + k.dist[0] * r2 + k.dist[1] * r4 + k.dist[4] * r6;
                const float dx = 2.0f * k.dist[2] * xn * yn + k.dist[3] * (r2 + 2.0f * xn * xn);
                const float dy = k.dist[2] * (r2 + 2.0f * yn * yn) + 2.0f * k.dist[3] * xn * yn;
                u = k.fx * (xn * radial + dx) + k.cx + k.tx * inv_w;
                v = k.fy * (yn * radial + dy) + k.cy + k.ty * inv_w;
            }

            if (!(u >= 0.0f && u < w_img && v >= 0.0f && v < h_img)) {
                continue;
            }

            const std::size_t hit_start = hits_.size();
            std::uint32_t tightest = 0;
            float min_area = std::numeric_limits<float>::max();

            if (!use_grid) {
                for (std::uint32_t b = 0; b < boxes_.size(); ++b) {
                    if (point_in_box(boxes_[b], u, v)) {
                        hits_.push_back({b, b, w, u, v});
                        if (box_areas_[b] < min_area) {
                            min_area = box_areas_[b];
                            tightest = b;
                        }
                    }
                }
            } else {
                const auto col = static_cast<std::size_t>(u * inv_cell_w);
                const auto row = static_cast<std::size_t>(v * inv_cell_h);
                for (const std::uint32_t b : grid_cells_[row * kGridCols + col]) {
                    if (point_in_box(boxes_[b], u, v)) {
                        hits_.push_back({b, b, w, u, v});
                        if (box_areas_[b] < min_area) {
                            min_area = box_areas_[b];
                            tightest = b;
                        }
                    }
                }
            }

            if (hits_.size() > hit_start + 1) {
                for (std::size_t idx = hit_start; idx < hits_.size(); ++idx) {
                    hits_[idx].tightest_box = tightest;
                }
            }
        }
    }

    void bucket_by_box() {
        const std::size_t nb = boxes_.size();
        offsets_.assign(nb + 1, 0);
        for (const auto& h : hits_) {
            ++offsets_[h.box + 1];
        }
        for (std::size_t b = 0; b < nb; ++b) {
            offsets_[b + 1] += offsets_[b];
        }
        bucketed_.resize(hits_.size());
        cursor_.assign(offsets_.begin(), offsets_.end() - 1);
        for (const auto& h : hits_) {
            bucketed_[cursor_[h.box]++] = h;
        }
    }

    [[nodiscard]] std::size_t bin_of(float depth) const noexcept {
        const auto raw = static_cast<std::size_t>(depth / prm_.bin_size);
        return std::min(raw, hist_.size() - 1);
    }

    // Bir kutunun yalnızca kendine özgü (en dar kutu = kendisi olan) noktalarından birincil küme derinliği
    [[nodiscard]] float estimate_primary_depth(std::uint32_t box, std::span<const Hit> h) {
        std::size_t lo = primary_hist_.size();
        std::size_t hi = 0;
        std::uint32_t total_primary = 0;
        for (const auto& p : h) {
            if (p.tightest_box != box) {
                continue;
            }
            const std::size_t b = bin_of(p.depth);
            ++primary_hist_[b];
            lo = std::min(lo, b);
            hi = std::max(hi, b);
            ++total_primary;
        }
        if (total_primary < prm_.min_points) {
            for (std::size_t b = lo; b <= hi && b < primary_hist_.size(); ++b) {
                primary_hist_[b] = 0;
            }
            return -1.0f;
        }

        std::size_t best_lo = 0;
        std::size_t best_hi = 0;
        bool found = false;
        std::size_t c_lo = lo;
        std::size_t last = lo;
        std::uint32_t c_n = 0;
        const auto close_cl = [&](std::size_t c_hi) {
            if (!found && c_n >= prm_.min_points) {
                found = true;
                best_lo = c_lo;
                best_hi = c_hi;
            }
        };
        for (std::size_t b = lo; b <= hi; ++b) {
            if (primary_hist_[b] == 0) {
                continue;
            }
            if (b - last > gap_bins_) {
                close_cl(last);
                c_lo = b;
                c_n = 0;
            }
            c_n += primary_hist_[b];
            last = b;
            primary_hist_[b] = 0;
        }
        close_cl(last);
        if (!found) {
            return -1.0f;
        }
        return 0.5f * static_cast<float>(best_lo + best_hi) * prm_.bin_size;
    }

    [[nodiscard]] bool is_occluded_by_foreground(std::uint32_t box, const Hit& p) const noexcept {
        if (!prm_.occlusion_handling || p.tightest_box == box) {
            return false;
        }
        const float fg_depth = claimed_depth_[p.tightest_box];
        const float my_depth = claimed_depth_[box];
        if (fg_depth > 0.0f && (my_depth < 0.0f || fg_depth + prm_.cluster_tolerance < my_depth)) {
            if (std::abs(p.depth - fg_depth) <= prm_.cluster_tolerance) {
                return true;
            }
        }
        return false;
    }

    void build_object(std::uint32_t box, std::span<const Hit> h, const Intrinsics& k) {
        if (h.size() < prm_.min_points) {
            return;
        }

        std::size_t lo = hist_.size();
        std::size_t hi = 0;
        std::uint32_t valid_count = 0;
        for (const auto& p : h) {
            if (is_occluded_by_foreground(box, p)) {
                continue;
            }
            const std::size_t b = bin_of(p.depth);
            ++hist_[b];
            lo = std::min(lo, b);
            hi = std::max(hi, b);
            ++valid_count;
        }

        if (valid_count < prm_.min_points) {
            for (std::size_t b = lo; b <= hi && b < hist_.size(); ++b) {
                hist_[b] = 0;
            }
            return;
        }

        std::size_t best_lo = 0;
        std::size_t best_hi = 0;
        std::size_t big_lo = 0;
        std::size_t big_hi = 0;
        std::uint32_t big_n = 0;
        bool found = false;
        std::size_t c_lo = lo;
        std::size_t last = lo;
        std::uint32_t c_n = 0;
        const auto close_cluster = [&](std::size_t c_hi) {
            if (c_n > big_n) {
                big_n = c_n;
                big_lo = c_lo;
                big_hi = c_hi;
            }
            if (!found && c_n >= prm_.min_points) {
                found = true;
                best_lo = c_lo;
                best_hi = c_hi;
            }
        };

        for (std::size_t b = lo; b <= hi; ++b) {
            if (hist_[b] == 0) {
                continue;
            }
            if (b - last > gap_bins_) {
                close_cluster(last);
                c_lo = b;
                c_n = 0;
            }
            c_n += hist_[b];
            last = b;
            hist_[b] = 0;
        }
        close_cluster(last);
        if (!found) {
            best_lo = big_lo;
            best_hi = big_hi;
        }

        scratch_.clear();
        float sx = 0.0f;
        float sy = 0.0f;
        float sxx = 0.0f;
        float syy = 0.0f;
        float szz = 0.0f;
        float sz = 0.0f;
        float x_min = std::numeric_limits<float>::max();
        float x_max = std::numeric_limits<float>::lowest();
        float y_min = x_min;
        float y_max = x_max;
        float d_min = x_min;
        float d_max = x_max;

        for (const auto& p : h) {
            if (is_occluded_by_foreground(box, p)) {
                continue;
            }
            const std::size_t b = bin_of(p.depth);
            if (b < best_lo || b > best_hi) {
                continue;
            }
            const float px = ((p.u - k.cx) * p.depth - k.tx) / k.fx;
            const float py = ((p.v - k.cy) * p.depth - k.ty) / k.fy;
            sx += px;
            sy += py;
            sz += p.depth;
            sxx += px * px;
            syy += py * py;
            szz += p.depth * p.depth;
            x_min = std::min(x_min, px);
            x_max = std::max(x_max, px);
            y_min = std::min(y_min, py);
            y_max = std::max(y_max, py);
            d_min = std::min(d_min, p.depth);
            d_max = std::max(d_max, p.depth);
            scratch_.push_back(p.depth);
        }
        if (scratch_.size() < prm_.min_points) {
            return;
        }

        const auto mid = scratch_.begin() + static_cast<std::ptrdiff_t>(scratch_.size() / 2);
        std::nth_element(scratch_.begin(), mid, scratch_.end());
        const float median_z = *mid;

        const auto n = static_cast<float>(scratch_.size());
        const float mean_x = sx / n;
        const float mean_y = sy / n;
        const float mean_z = sz / n;

        const float var_x = std::max(1e-4f, (sxx / n) - (mean_x * mean_x));
        const float var_y = std::max(1e-4f, (syy / n) - (mean_y * mean_y));
        const float var_z = std::max(1e-4f, (szz / n) - (mean_z * mean_z));

        float size_x = std::max(0.1f, x_max - x_min);
        float size_y = std::max(0.1f, y_max - y_min);
        float size_z = std::max(prm_.min_depth_spread, d_max - d_min);

        const auto& bx = boxes_[box];
        const float w_prior = std::clamp(prm_.class_prior_weight, 0.0f, 1.0f);
        if (w_prior > 0.0f && bx.prior_size_x > 0.0f && bx.prior_size_y > 0.0f && bx.prior_size_z > 0.0f) {
            size_x = (1.0f - w_prior) * size_x + w_prior * bx.prior_size_x;
            size_y = (1.0f - w_prior) * size_y + w_prior * bx.prior_size_y;
            size_z = (1.0f - w_prior) * size_z + w_prior * bx.prior_size_z;
        }

        objects_.push_back({
            box,
            mean_x,
            mean_y,
            median_z,
            size_x,
            size_y,
            size_z,
            {var_x, var_y, var_z},
            static_cast<std::uint32_t>(scratch_.size())
        });
    }

    Params prm_{};
    std::uint32_t gap_bins_{0};
    std::vector<Hit> hits_;
    std::vector<Hit> bucketed_;
    std::vector<float> scratch_;
    std::vector<std::uint32_t> hist_;
    std::vector<std::uint32_t> primary_hist_;
    std::vector<std::size_t> offsets_;
    std::vector<std::size_t> cursor_;
    std::vector<BoxPx> boxes_;
    std::vector<float> box_areas_;
    std::vector<float> claimed_depth_;
    std::vector<Object3D> objects_;
    std::vector<std::vector<std::uint32_t>> grid_cells_;
};

}  // namespace licam_fusion::core

#endif  // LICAM_FUSION__FUSION_CORE_HPP_
