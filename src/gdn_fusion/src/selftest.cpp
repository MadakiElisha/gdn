// Unit tests for the ROS-free navigation core.
// Build+run without ROS: see tools/run_selftest.sh
#include <cstdio>
#include <fstream>
#include <vector>
#include "gdn/map_db.hpp"
#include "gdn/eskf.hpp"

using namespace gdn;
static int g_fail = 0;
#define CHECK(cond, name) do { \
    if (cond) std::printf("PASS  %s\n", name); \
    else { std::printf("FAIL  %s\n", name); g_fail++; } } while (0)

// Planar slope map centered on the map-frame origin.
static void WritePlaneMap(const std::string& path, double slope_n, double base) {
    const int nx = 64, ny = 64;
    const double dlat = 0.001, dlon = 0.001;
    const double lat0 = MapDb::kLatC - 0.032, lon0 = MapDb::kLonC - 0.032;
    std::ofstream f(path, std::ios::binary);
    const int32_t a = nx, b = ny;
    f.write(reinterpret_cast<const char*>(&a), 4);
    f.write(reinterpret_cast<const char*>(&b), 4);
    f.write(reinterpret_cast<const char*>(&lat0), 8);
    f.write(reinterpret_cast<const char*>(&lon0), 8);
    f.write(reinterpret_cast<const char*>(&dlat), 8);
    f.write(reinterpret_cast<const char*>(&dlon), 8);
    std::vector<float> alt(static_cast<size_t>(nx) * ny);
    for (int i = 0; i < ny; ++i)
        for (int j = 0; j < nx; ++j) {
            const double n = (lat0 + i * dlat - MapDb::kLatC) * MapDb::kMPerDegLat;
            alt[static_cast<size_t>(i) * nx + j] = static_cast<float>(base + slope_n * n);
        }
    f.write(reinterpret_cast<const char*>(alt.data()), 4LL * nx * ny);
}

int main() {
    using V3 = Eigen::Vector3d;
    const std::string mp = "/tmp/gdn_selftest_map.bin";
    WritePlaneMap(mp, 0.2, 500.0);

    MapDb map;
    CHECK(map.Load(mp), "map_db: loads binary grid");
    const MapQuery q0 = map.Query(0.0, 0.0);
    CHECK(q0.ok && std::abs(q0.dn - 0.2) < 1e-3 && std::abs(q0.de) < 1e-6,
          "map_db: planar gradient exact");
    CHECK(std::abs(q0.h - 500.0) < 1e-3, "map_db: planar altitude exact");
    CHECK(!map.Query(1e6, 0.0).ok, "map_db: out-of-bounds rejected");

    const V3 b_a(0.05, -0.03, 0.08), b_g(0.0004, -0.0002, 0.0003);
    const Eigen::Quaterniond q_true(
        Eigen::AngleAxisd(0.05, V3::UnitY()) * Eigen::AngleAxisd(-0.03, V3::UnitX()));
    const V3 a_static = q_true.conjugate() * V3(0, 0, -Eskf::kG) + b_a;
    Eskf eskf;
    bool aligned = false;
    for (int i = 0; i < 700 && !aligned; ++i) aligned = eskf.FeedStatic(a_static, b_g);
    CHECK(aligned, "eskf: static alignment converges");
    // Attitude-bias coupling: tolerance 0.1 m/s^2 realistic for static method
    CHECK((eskf.accel_bias() - b_a).norm() < 0.1, "eskf: accel bias recovered (attitude-coupled)");
    CHECK((eskf.gyro_bias() - b_g).norm() < 1e-4, "eskf: gyro bias recovered");
    const double att_err = 2.0 * std::acos(std::min(1.0, std::abs(
        (eskf.q() * q_true.conjugate()).w()))) / Eskf::kDeg;
    CHECK(att_err < 0.5, "eskf: attitude recovered");

    for (int i = 0; i < 600; ++i) eskf.Propagate(a_static, b_g, 0.01);
    CHECK(eskf.vel().norm() < 0.1 && eskf.pos().norm() < 1.0,
          "eskf: static propagation stays put");

    Eskf nav;
    const V3 a0(0, 0, -Eskf::kG), w0 = V3::Zero();
    bool al = false;
    for (int i = 0; i < 700 && !al; ++i) al = nav.FeedStatic(a0, w0);
    nav.TransferAlignYaw(0.0);
    const double meas = 500.0 + 0.2 * 30.0;   // truth 30 m north on the plane
    int applied = 0;
    for (int k = 0; k < 60; ++k) {
        nav.Propagate(a0, w0, 0.5);
        const MapQuery mq = map.Query(nav.pos().x(), nav.pos().y());
        if (nav.ApplyTrn(meas, mq) == Eskf::UpdResult::kApplied) applied++;
    }
    CHECK(applied > 40, "eskf: updates accepted on observable slope");
    CHECK(std::abs(nav.pos().x() - 30.0) < 5.0, "eskf: position converges along gradient");
    CHECK(std::abs(nav.pos().y()) < 5.0, "eskf: unobservable axis not pumped");

    std::printf(g_fail == 0 ? "ALL TESTS PASSED\n" : "%d TEST(S) FAILED\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
