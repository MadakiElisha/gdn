// Unit tests for the ROS-free navigation core.
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
    CHECK((eskf.accel_bias() - b_a).norm() < 0.1, "eskf: accel bias recovered (attitude-coupled)");
    CHECK((eskf.gyro_bias() - b_g).norm() < 1e-4, "eskf: gyro bias recovered");
    const double att_err = 2.0 * std::acos(std::min(1.0, std::abs(
        (eskf.q() * q_true.conjugate()).w()))) / Eskf::kDeg;
    CHECK(att_err < 0.5, "eskf: attitude recovered");

    for (int i = 0; i < 600; ++i) eskf.Propagate(a_static, b_g, 0.01);
    CHECK(eskf.vel().norm() < 0.1 && eskf.pos().norm() < 1.0,
          "eskf: static propagation stays put");

    EskfConfig test_cfg;
    test_cfg.r_trn = 1.0;
    Eskf nav(test_cfg);
    const V3 a0(0, 0, -Eskf::kG), w0 = V3::Zero();
    bool al = false;
    for (int i = 0; i < 700 && !al; ++i) al = nav.FeedStatic(a0, w0);
    nav.TransferAlignYaw(0.0);
    const double meas = 500.0 + 0.2 * 30.0;
    int applied = 0;
    for (int k = 0; k < 60; ++k) {
        nav.Propagate(a0, w0, 0.5);
        const MapQuery mq = map.Query(nav.pos().x(), nav.pos().y());
        if (nav.ApplyTrn(meas, mq) == Eskf::UpdResult::kApplied) applied++;
    }
    CHECK(applied > 40, "eskf: updates accepted on observable slope");
    CHECK(std::abs(nav.pos().x() - 30.0) < 5.0, "eskf: position converges along gradient");
    CHECK(std::abs(nav.pos().y()) < 5.0, "eskf: unobservable axis not pumped");

    // TEST-013: Barometer update (OI-002) - vertical observability
    EskfConfig baro_cfg;
    baro_cfg.r_trn = 1.0;
    baro_cfg.r_baro = 1.0;
    baro_cfg.p0_baro_bias = 100.0;
    baro_cfg.gate_baro_sigma = 50.0;
    Eskf baro_test(baro_cfg);
    al = false;
    for (int i = 0; i < 700 && !al; ++i) al = baro_test.FeedStatic(a0, w0);
    baro_test.TransferAlignYaw(0.0);
    
    const double true_z = 500.0;
    const double true_baro_bias = 5.0;
    const double baro_meas = true_z + true_baro_bias;
    
    int baro_applied = 0;
    for (int k = 0; k < 100; ++k) {
        baro_test.Propagate(a0, w0, 0.1);
        if (baro_test.ApplyBaro(baro_meas) == Eskf::UpdResult::kApplied) baro_applied++;
    }
    
    // The baro measurement only observes the SUM (pos.z + baro_bias).
    // Without independent pos.z observation (e.g. GNSS init), the filter
    // splits the innovation based on relative uncertainties.
    const double sum_final = baro_test.pos().z() + baro_test.baro_bias();
    CHECK(baro_applied > 80, "eskf: baro updates accepted");
    CHECK(std::abs(sum_final - baro_meas) < 2.0, "eskf: baro observable (z + bias) converges");


    // TEST-014: Magnetometer observability (OI-003), two decoupled checks.
    {
        const Eigen::Vector3d m_ref(25.0, 0.0, 45.0);
        // Phase 1: yaw convergence, bias known zero, gateable 20-deg yaw error
        EskfConfig mc1;
        mc1.p0_mag_bias = 0.0;        // bias known: isolates yaw observability
        mc1.clamp_dtheta_deg = 5.0;   // unit-test convergence clamp
        Eskf m1(mc1);
        al = false;
        for (int i = 0; i < 700 && !al; ++i) al = m1.FeedStatic(a0, w0);
        const Eigen::Quaterniond q_true(Eigen::AngleAxisd(20.0 * Eskf::kDeg, V3::UnitZ()));
        const Eigen::Matrix3d R_true = q_true.toRotationMatrix();
        int ap1 = 0;
        for (int k = 0; k < 300; ++k) {
            m1.Propagate(a0, w0, 0.1);
            if (m1.ApplyMag(R_true.transpose() * m_ref) == Eskf::UpdResult::kApplied) ap1++;
        }
        const double yaw_err = 2.0 * std::acos(std::min(1.0, std::abs(
            (m1.q() * q_true.conjugate()).w()))) / Eskf::kDeg;
        std::printf("  [DIAG] Phase 1 final yaw_err=%.2f deg, applied=%d, rejects=%d\n",
                    yaw_err, ap1, m1.rejects());
        CHECK(ap1 > 200, "eskf: mag updates accepted");
        CHECK(yaw_err < 5.0, "eskf: mag yaw converges");

        // Phase 2: hard-iron bias estimation.
        // Use a fresh filter with standard p0_mag_bias (100.0).
        // Keep truth_bias small (norm < 5.0) to avoid triggering clamp_dvm=5.0 on the first step.
        EskfConfig mc2;
        mc2.clamp_dtheta_deg = 15.0;
        Eskf m2(mc2);
        al = false;
        for (int i = 0; i < 700 && !al; ++i) al = m2.FeedStatic(a0, w0);
        const Eigen::Vector3d truth_bias(2.0, -1.0, 1.0);
        const Eigen::Matrix3d R_m2 = Eigen::Matrix3d::Identity();
        int ap2 = 0;
        for (int k = 0; k < 300; ++k) {
            m2.Propagate(a0, w0, 0.1);
            if (m2.ApplyMag(R_m2.transpose() * m_ref + truth_bias) == Eskf::UpdResult::kApplied) ap2++;
        }
        std::printf("  [DIAG] Phase 2 mag_bias=(%.2f, %.2f, %.2f) truth=(2.00, -1.00, 1.00) applied=%d rejects=%d\n",
                    m2.mag_bias().x(), m2.mag_bias().y(), m2.mag_bias().z(), ap2, m2.rejects());
        CHECK((m2.mag_bias() - truth_bias).norm() < 1.0, "eskf: mag bias estimated");
    }

    std::printf(g_fail == 0 ? "ALL TESTS PASSED\n" : "%d TEST(S) FAILED\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
// Note: The main function needs to be modified to include this test.
// For now, just verifying the build compiles.
