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

#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <vector>

#include "licam_fusion/fusion_core.hpp"

namespace {

using namespace licam_fusion::core;

struct TestPoint {
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
    std::uint32_t pad{0};
};

// LiDAR (x ileri, y sol, z yukarı) -> Kamera optik (x sağ, y aşağı, z ileri)
constexpr std::array<float, 9> kLidarToCamR{
    0.0f, -1.0f,  0.0f,
    0.0f,  0.0f, -1.0f,
    1.0f,  0.0f,  0.0f
};
constexpr std::array<float, 3> kZeroT{0.0f, 0.0f, 0.0f};

CloudView make_cloud_view(const std::vector<TestPoint>& pts) {
    return CloudView{
        reinterpret_cast<const std::byte*>(pts.data()),
        pts.size(),
        sizeof(TestPoint),
        0, 4, 8
    };
}

TEST(FusionCoreTest, RejectsBackgroundWallAndEstimatesTrueClusterCenter) {
    const Intrinsics k{600.0f, 600.0f, 320.0f, 240.0f, 0.0f, 0.0f, 640, 480};
    const Projection proj = Projection::from(k, kLidarToCamR, kZeroT);

    std::vector<TestPoint> pts;
    // 10 m'de ön plan nesnesi
    for (int ix = -4; ix <= 4; ++ix) {
        for (int iy = -4; iy <= 4; ++iy) {
            pts.push_back({10.0f, static_cast<float>(ix) * 0.08f, static_cast<float>(iy) * 0.08f});
        }
    }
    // 30 m'de arka plan duvarı
    for (int ix = -10; ix <= 10; ++ix) {
        for (int iy = -10; iy <= 10; ++iy) {
            pts.push_back({30.0f, static_cast<float>(ix) * 0.15f, static_cast<float>(iy) * 0.15f});
        }
    }
    // NaN ve kamera arkası noktalar
    pts.push_back({std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f});
    pts.push_back({-5.0f, 0.0f, 0.0f});

    const BoxPx box{270.0f, 190.0f, 370.0f, 290.0f};
    FusionCore core{2048, 32, Params{}};
    const auto out = core.process(proj, k, std::span{&box, 1}, make_cloud_view(pts));

    ASSERT_EQ(out.size(), 1u);
    EXPECT_NEAR(out[0].z, 10.0f, 0.05f);
    EXPECT_NEAR(out[0].x, 0.0f, 0.05f);
    EXPECT_NEAR(out[0].y, 0.0f, 0.05f);
    EXPECT_GT(out[0].num_points, 50u);
}

TEST(FusionCoreTest, HandlesOccludingBoxesCorrectly) {
    const Intrinsics k{600.0f, 600.0f, 320.0f, 240.0f, 0.0f, 0.0f, 640, 480};
    const Projection proj = Projection::from(k, kLidarToCamR, kZeroT);

    std::vector<TestPoint> pts;
    // Ön planda yaya: 8.0 m derinlik, merkez (u=320, v=240)
    for (int ix = -3; ix <= 3; ++ix) {
        for (int iy = -3; iy <= 3; ++iy) {
            pts.push_back({8.0f, static_cast<float>(ix) * 0.05f, static_cast<float>(iy) * 0.05f});
        }
    }
    // Arka planda araç: 20.0 m derinlik, daha geniş kutu (yaya kutusunu kapsıyor ama yanlarda kendi noktaları var)
    for (int ix = -8; ix <= 8; ++ix) {
        for (int iy = -6; iy <= 6; ++iy) {
            const float y_l = static_cast<float>(ix) * 0.25f;
            if (std::abs(y_l) < 0.45f) {
                continue;  // Yayanın kapattığı orta bölge
            }
            pts.push_back({20.0f, y_l, static_cast<float>(iy) * 0.15f});
        }
    }

    // Box 0: Yaya (dar kutu) | Box 1: Araç (yaya kutusunu tamamen kapsayan geniş kutu)
    const std::array<BoxPx, 2> boxes{{
        {295.0f, 215.0f, 345.0f, 265.0f},
        {230.0f, 180.0f, 410.0f, 300.0f}
    }};

    FusionCore core{2048, 32, Params{}};
    const auto out = core.process(proj, k, boxes, make_cloud_view(pts));

    ASSERT_EQ(out.size(), 2u);
    EXPECT_EQ(out[0].box, 0u);
    EXPECT_NEAR(out[0].z, 8.0f, 0.1f);
    EXPECT_EQ(out[1].box, 1u);
    EXPECT_NEAR(out[1].z, 20.0f, 0.1f);
}

TEST(FusionCoreTest, RejectsSparseNoiseBelowMinPoints) {
    const Intrinsics k{600.0f, 600.0f, 320.0f, 240.0f, 0.0f, 0.0f, 640, 480};
    const Projection proj = Projection::from(k, kLidarToCamR, kZeroT);

    const std::vector<TestPoint> pts{
        {12.0f, 0.0f, 0.0f},
        {12.1f, 0.05f, 0.0f},
        {11.9f, -0.05f, 0.0f}
    };
    const BoxPx box{280.0f, 200.0f, 360.0f, 280.0f};

    Params prm{};
    prm.min_points = 5;
    FusionCore core{256, 16, prm};
    const auto out = core.process(proj, k, std::span{&box, 1}, make_cloud_view(pts));
    EXPECT_TRUE(out.empty());
}

TEST(FusionCoreTest, SpatialGridMatchesLinearAssociation) {
    const Intrinsics k{600.0f, 600.0f, 320.0f, 240.0f, 0.0f, 0.0f, 640, 480};
    const Projection proj = Projection::from(k, kLidarToCamR, kZeroT);

    std::mt19937 rng{42};  // NOLINT(bugprone-random-generator-seed)
    std::uniform_real_distribution<float> dx(5.0f, 30.0f);
    std::uniform_real_distribution<float> dy(-8.0f, 8.0f);
    std::uniform_real_distribution<float> dz(-3.0f, 3.0f);
    std::vector<TestPoint> pts(4000);
    for (auto& p : pts) {
        p = {dx(rng), dy(rng), dz(rng)};
    }

    std::vector<BoxPx> boxes;
    for (std::size_t i = 0; i < 30; ++i) {
        const auto col = i % 6u;
        const auto row = i / 6u;
        const auto u0 = static_cast<float>(col * 100u + 20u);
        const auto v0 = static_cast<float>(row * 85u + 20u);
        boxes.push_back({u0, v0, u0 + 70.0f, v0 + 65.0f});
    }

    Params p_linear{};
    p_linear.grid_box_threshold = 1000;
    Params p_grid{};
    p_grid.grid_box_threshold = 4;

    FusionCore core_lin{8192, 64, p_linear};
    FusionCore core_grd{8192, 64, p_grid};

    const auto out_lin = core_lin.process(proj, k, boxes, make_cloud_view(pts));
    const auto out_grd = core_grd.process(proj, k, boxes, make_cloud_view(pts));

    ASSERT_EQ(out_lin.size(), out_grd.size());
    for (std::size_t i = 0; i < out_lin.size(); ++i) {
        EXPECT_EQ(out_lin[i].box, out_grd[i].box);
        EXPECT_NEAR(out_lin[i].z, out_grd[i].z, 1e-4f);
        EXPECT_EQ(out_lin[i].num_points, out_grd[i].num_points);
    }
}

TEST(FusionCoreTest, FrameStrongTypesAndRingBufferWork) {
    const T_cam_lidar tf{kLidarToCamR, kZeroT};
    const Point3<LidarFrame> p_l{10.0f, -2.0f, 1.0f};
    const Point3<CameraFrame> p_c = tf * p_l;
    EXPECT_FLOAT_EQ(p_c.x, 2.0f);
    EXPECT_FLOAT_EQ(p_c.y, -1.0f);
    EXPECT_FLOAT_EQ(p_c.z, 10.0f);

    RingBuffer<int, 4> rb;
    for (int i = 1; i <= 6; ++i) {
        rb.push(i);
    }
    EXPECT_EQ(rb.size(), 4u);
    EXPECT_EQ(rb.oldest(), 3);
    EXPECT_EQ(rb.latest(), 6);
    EXPECT_NEAR(rb.mean(), 4.5, 1e-6);
}

}  // namespace
