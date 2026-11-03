// Unit tests for the ROS-free navigation core.
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <vector>
#include "gdn/map_db.hpp"
#include "gdn/eskf.hpp"

using namespace gdn;
static int g_fail = 0;
#define CHECK(cond, name) do { \
    if (cond) std::printf("PASS  %s\n", name); \
    else { std::printf("FAIL  %s\n", name); g_fail++; } } while (0)

// Planar test map centred on (lat_c, lon_c): h = base + slope_n*N + slope_e*E.
static void WritePlaneMap(const std::string& path, double slope_n, double base,
                          double slope_e = 0.0, double lat_c = 45.9632,
                          double lon_c = 7.6440) {
    const int nx = 64, ny = 64;
    const double dlat = 0.001, dlon = 0.001;
    const double lat0 = lat_c - 0.032, lon0 = lon_c - 0.032;   // grid centre == (lat_c, lon_c)
    const double m_lon = MapDb::kMPerDegLat * std::cos(lat_c * M_PI / 180.0);
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
            const double n = (lat0 + i * dlat - lat_c) * MapDb::kMPerDegLat;
            const double e = (lon0 + j * dlon - lon_c) * m_lon;
            alt[static_cast<size_t>(i) * nx + j] = static_cast<float>(base + slope_n * n + slope_e * e);
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

    // East scaling must use cos(latitude of THIS map), not a hard-coded site.
    {
        const std::string mp_e = "/tmp/gdn_selftest_map_east.bin";
        WritePlaneMap(mp_e, 0.0, 300.0, 0.1, 60.0, 10.0);   // 60 N: cos differs a lot from 46 N
        MapDb me;
        const bool ok = me.Load(mp_e);
        const MapQuery qe = me.Query(0.0, 1000.0);
        CHECK(ok && qe.ok && std::abs(qe.h - 400.0) < 0.05 && std::abs(qe.de - 0.1) < 1e-3,
              "map_db: east axis scaled by map latitude");
        // A truncated file must be rejected, not read as garbage.
        std::ifstream src(mp_e, std::ios::binary);
        std::vector<char> bytes((std::istreambuf_iterator<char>(src)), {});
        std::ofstream(mp_e, std::ios::binary).write(bytes.data(), static_cast<std::streamsize>(bytes.size() - 100));
        MapDb mt;
        CHECK(!mt.Load(mp_e), "map_db: truncated file rejected");
    }

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
        mc1.mag_ref_north = 25.0; mc1.mag_ref_east = 0.0; mc1.mag_ref_down = 45.0; // Match test sim
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
        mc2.mag_ref_north = 25.0; mc2.mag_ref_east = 0.0; mc2.mag_ref_down = 45.0; // Match test sim
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

        // TEST-015: ZUPT pins horizontal states while static
    {
        EskfConfig zc;
        zc.mag_ref_north = 25.0; zc.mag_ref_east = 0.0; zc.mag_ref_down = 45.0;
        Eskf zt(zc);
        bool al = false;
        for (int i = 0; i < 700 && !al; ++i) al = zt.FeedStatic(a0, w0);
        CHECK(al, "eskf: TEST-015 static alignment");
        const Eigen::Vector3d a_bias = a0 + Eigen::Vector3d(0.05, 0.0, 0.0);
        int zap = 0;
        for (int k = 0; k < 300; ++k) {
            zt.Propagate(a_bias, w0, 0.1);
            if (k % 10 == 9 && zt.ApplyZupt() == Eskf::UpdResult::kApplied) zap++;
        }
        CHECK(zap > 20, "eskf: zupt updates applied");
        CHECK(zt.vel().norm() < 0.2, "eskf: zupt pins velocity");
        CHECK(zt.pos().head<2>().norm() < 1.0, "eskf: zupt pins horizontal position");
        Eskf zn(zc);
        al = false;
        for (int i = 0; i < 700 && !al; ++i) al = zn.FeedStatic(a0, w0);
        for (int k = 0; k < 300; ++k) zn.Propagate(a_bias, w0, 0.1);
        CHECK(zn.pos().head<2>().norm() > 5.0, "eskf: control drifts without zupt");
    }

    // R1.1: real altimeter model. AGL = (h0 - p_z) - h_map(n, e). Altitude error
    // now enters the measurement, so z and the baro bias are estimated with it.
    {
        const std::string mp_flat = "/tmp/gdn_selftest_map_flat.bin";
        WritePlaneMap(mp_flat, 0.0, 500.0);
        MapDb flat; flat.Load(mp_flat);
        EskfConfig fc;
        fc.r_agl = 4.0;            // 2 m sensor noise; the flat map is exact
        Eskf a(fc);
        bool ok = false;
        for (int i = 0; i < 700 && !ok; ++i) ok = a.FeedStatic(a0, w0);
        a.TransferAlignYaw(0.0);
        // Without a datum the update must refuse to run.
        CHECK(a.ApplyAgl(500.0, flat.Query(0, 0)) != Eskf::UpdResult::kApplied,
              "eskf: agl refused without altitude datum");
        a.SetAltitudeDatum(1000.0);              // AMSL height of p_z = 0
        // The vehicle holds z = 0 (500 m above the flat map). The barometer then reads
        // 15 m off (NED +15 = reads 15 m too low), as after a pressure change. AGL must
        // hold the altitude while the filter learns the bias (REQ-NAV-010).
        const double z_true = 0.0, b_true = 15.0;
        const double agl_true = (1000.0 - z_true) - 500.0;
        int acc = 0;
        for (int k = 0; k < 400; ++k) {
            a.Propagate(a0, w0, 0.1);
            if (a.ApplyAgl(agl_true, flat.Query(a.pos().x(), a.pos().y())) == Eskf::UpdResult::kApplied) acc++;
            a.ApplyBaro(z_true + b_true);
        }
        CHECK(acc > 300, "eskf: agl updates accepted");
        CHECK(std::abs(a.pos().z() - z_true) < 1.0, "eskf: agl holds altitude against a 15 m baro bias");
        CHECK(std::abs(a.baro_bias() - b_true) < 1.5, "eskf: agl + baro estimate baro bias");

        // Warm start: the datum and baro bias are taken together at start-up, so a
        // 15 m bias doesn't need to be learned from AGL at all.
        Eskf w;
        ok = false;
        for (int i = 0; i < 700 && !ok; ++i) ok = w.FeedStatic(a0, w0);
        w.TransferAlignYaw(0.0);
        w.SetAltitudeDatum(500.0 + 0.5);          // map height + AGL on the ground
        w.InitBaroBias(0.0 + b_true);             // baro reading on the ground
        CHECK(std::abs(w.baro_bias() - b_true) < 1e-9, "eskf: baro bias initialised at datum");
        w.SetAltitudeDatum(500.5, 2.0);
        w.InitBaroBias(b_true, 1.5);
        CHECK(std::abs(w.sigma_pos_z() - 2.0) < 1e-9 && std::abs(w.sigma_baro_bias() - 2.5) < 1e-9 &&
              w.p_mineig() > -1e-9, "eskf: datum and baro calibration set a consistent covariance");

        // Sign of the horizontal Jacobian: on a north slope, a vehicle that is really
        // 30 m north reads less AGL, and the update must move the estimate north.
        EskfConfig sc; sc.r_agl = 1.0;
        Eskf g(sc);
        ok = false;
        for (int i = 0; i < 700 && !ok; ++i) ok = g.FeedStatic(a0, w0);
        g.TransferAlignYaw(0.0);
        g.SetAltitudeDatum(1000.0);
        const double agl_n30 = 1000.0 - (500.0 + 0.2 * 30.0);   // z = 0, 30 m north
        double first = 0.0, last = 0.0;
        for (int k = 0; k < 20; ++k) {
            g.Propagate(a0, w0, 0.1);
            double in = 0.0;
            g.ApplyAgl(agl_n30, map.Query(g.pos().x(), g.pos().y()), &in);
            if (k == 0) first = in;
            last = in;
        }
        CHECK(g.pos().x() > 1.0 && std::abs(last) < std::abs(first),
              "eskf: agl moves position along the gradient");
    }

std::printf(g_fail == 0 ? "ALL TESTS PASSED\n" : "%d TEST(S) FAILED\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
// Note: The main function needs to be modified to include this test.
// For now, just verifying the build compiles.
