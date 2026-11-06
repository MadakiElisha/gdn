// Unit tests for the ROS-free navigation core.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <random>
#include <vector>
#include "gdn/map_db.hpp"
#include "gdn/eskf.hpp"
#include "gdn/consistency.hpp"
#include "gdn/ground_monitor.hpp"
#include "gdn/imu_model.hpp"
#include "gdn/paths.hpp"

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

// Map of a terrain defined in geographic coordinates, cut around (lat_c, lon_c), so two
// maps with different centres sample the same ground (R1.3 shared-origin test).
static void WriteGeoMap(const std::string& path, double lat_c, double lon_c) {
    const int nx = 64, ny = 64;
    const double dlat = 0.001, dlon = 0.001;
    const double lat0 = lat_c - 0.032, lon0 = lon_c - 0.032;
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
            const double n = (lat0 + i * dlat - 45.96) * MapDb::kMPerDegLat;
            const double e = (lon0 + j * dlon - 7.64) * MapDb::kMPerDegLat * std::cos(45.96 * M_PI / 180.0);
            alt[static_cast<size_t>(i) * nx + j] =
                static_cast<float>(1000.0 + 0.2 * n + 50.0 * std::sin(n / 300.0) * std::cos(e / 400.0));
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

    // IMU hole: the filter believes it's level, the vehicle is really pitched 15 deg
    // (the attitude change during lost IMU samples). Without inflation the mag gate
    // rejects the large innovation forever; with InflateForGap the mag pulls it back.
    {
        const V3 m_ref(25.0, 0.0, 45.0);
        const Eigen::Quaterniond q_true(Eigen::AngleAxisd(15.0 * Eskf::kDeg, V3::UnitY()));
        const Eigen::Matrix3d R_true = q_true.toRotationMatrix();
        const V3 a_true = R_true.transpose() * V3(0.0, 0.0, -9.80665);
        auto run = [&](bool inflate, int* applied) {
            EskfConfig c; c.mag_ref_north = 25.0; c.mag_ref_east = 0.0; c.mag_ref_down = 45.0;
            Eskf f(c);
            bool ok = false;
            for (int i = 0; i < 700 && !ok; ++i) ok = f.FeedStatic(V3(0.0, 0.0, -9.80665), V3::Zero());
            for (int k = 0; k < 60; ++k) {                 // settle the covariance, level and still
                f.Propagate(V3(0.0, 0.0, -9.80665), V3::Zero(), 0.1);
                f.ApplyMag(m_ref);
            }
            if (inflate) f.InflateForGap(30.0 * Eskf::kDeg, 1.0);
            *applied = 0;
            for (int k = 0; k < 100; ++k) {                 // 10 s at the true (pitched) attitude
                for (int j = 0; j < 10; ++j) f.Propagate(a_true, V3::Zero(), 0.01);
                if (f.ApplyMag(R_true.transpose() * m_ref) == Eskf::UpdResult::kApplied) (*applied)++;
            }
            return 2.0 * std::acos(std::min(1.0, std::abs((f.q() * q_true.conjugate()).w()))) / Eskf::kDeg;
        };
        int ap_no = 0, ap_yes = 0;
        const double e_no = run(false, &ap_no), e_yes = run(true, &ap_yes);
        std::printf("  [DIAG] IMU hole: no inflation err %.1f deg (mag applied %d); inflated err %.1f deg (applied %d)\n",
                    e_no, ap_no, e_yes, ap_yes);
        CHECK(e_no > 10.0, "eskf: imu hole without inflation stays wrong (test is meaningful)");
        CHECK(e_yes < 3.0 && ap_yes > 50, "eskf: imu hole inflation lets the mag recover attitude");
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

    // TEST-018 (R1.2): on-ground detection from the IMU alone. Before R1.2 the node
    // ran ZUPT until GNSS speed passed 2 m/s, so without GNSS it never ran ZUPT, and
    // with GNSS it kept pinning velocity to zero for the first 2 m/s of the take-off roll.
    {
        std::mt19937 rng(18);
        std::normal_distribution<double> na(0.0, 0.15), nw(0.0, 0.002);
        const V3 ba_true(0.05, -0.03, 0.04), bg_true(0.002, -0.001, 0.0015);
        // Noisy IMU sample for a level vehicle: specific force -g plus body accel acc_b.
        auto imu = [&](const V3& acc_b, const V3& rate_b, V3& a, V3& w) {
            a = a0 + acc_b + ba_true + V3(na(rng), na(rng), na(rng));
            w = rate_b + bg_true + V3(nw(rng), nw(rng), nw(rng));
        };
        const double dt = 0.01;
        // Align on noisy data, then start the monitor from the aligned rest state.
        auto make = [&](Eskf& e, GroundMonitor& g) {
            V3 a, w; bool ok = false;
            for (int i = 0; i < 2000 && !ok; ++i) { imu(V3::Zero(), V3::Zero(), a, w); ok = e.FeedStatic(a, w); }
            g.Start(e.RestSpecificForce(), e.gyro_bias());
            return ok;
        };
        // (a) Parked for 60 s: ZUPT about every 0.1 s, never airborne. Heading is not
        // observable while parked, so ZUPT must leave it alone: with noisy accel samples
        // in the linearisation, a full-gain ZUPT random-walks yaw by degrees.
        auto heading = [](const Eskf& e) {
            const V3 f = e.q() * V3::UnitX(); return std::atan2(f.y(), f.x()); };
        {
            Eskf e; GroundMonitor g;
            CHECK(make(e, g), "ground: TEST-018 aligned on noisy IMU");
            int z = 0; V3 a, w;
            const double yaw0 = heading(e);
            for (int i = 0; i < 6000; ++i) {
                imu(V3::Zero(), V3::Zero(), a, w);
                e.Propagate(a, w, dt);
                if (g.Update(a, w, dt)) { e.ApplyZupt(); z++; }
            }
            CHECK(z > 500 && z < 620 && !g.airborne(), "ground: parked 60 s gives ~10 Hz ZUPT, no latch");
            const double dyaw = std::abs(std::remainder(heading(e) - yaw0, 2 * M_PI)) / Eskf::kDeg;
            std::printf("      heading change over 60 s of ZUPT: %.2f deg\n", dyaw);
            CHECK(dyaw < 0.5, "eskf: zupt leaves heading alone");
        }
        // (b) Hard take-off roll, 2 m/s^2: ZUPT stops within 0.05 s, latch within 0.6 s.
        // (c) Gentle roll, 0.5 m/s^2: ZUPT stops within 0.15 s, latch within 1.0 s.
        struct Roll { double acc, stop_s, latch_s; const char* name; };
        for (const Roll& r : {Roll{2.0, 0.05, 0.6, "ground: 2 m/s^2 roll stops ZUPT at once and latches"},
                              Roll{0.5, 0.15, 1.0, "ground: 0.5 m/s^2 roll stops ZUPT and latches"}}) {
            Eskf e; GroundMonitor g; make(e, g);
            V3 a, w; int late = 0;
            for (int i = 0; i < 1000; ++i) { imu(V3::Zero(), V3::Zero(), a, w); g.Update(a, w, dt); }
            const double t_on = 10.0;
            for (int i = 0; i < 300; ++i) {
                imu(V3(r.acc, 0, 0), V3::Zero(), a, w);
                if (g.Update(a, w, dt) && i * dt > r.stop_s) late++;
            }
            CHECK(late == 0 && g.airborne() && g.latch_time() > t_on && g.latch_time() < t_on + r.latch_s, r.name);
        }
        // (d) A 0.2 s bump (someone touches the vehicle) must not latch, and ZUPT resumes.
        {
            Eskf e; GroundMonitor g; make(e, g);
            V3 a, w; int after = 0;
            for (int i = 0; i < 500; ++i) { imu(V3::Zero(), V3::Zero(), a, w); g.Update(a, w, dt); }
            for (int i = 0; i < 20; ++i) { imu(V3(1.5, 0, 0), V3::Zero(), a, w); g.Update(a, w, dt); }
            for (int i = 0; i < 200; ++i) { imu(V3::Zero(), V3::Zero(), a, w); after += g.Update(a, w, dt); }
            CHECK(!g.airborne() && after > 10, "ground: short bump does not latch, ZUPT resumes");
        }
        // (e) Turning on the spot (0.1 rad/s, no acceleration) counts as motion.
        {
            Eskf e; GroundMonitor g; make(e, g);
            V3 a, w; int z = 0;
            for (int i = 0; i < 500; ++i) { imu(V3::Zero(), V3::Zero(), a, w); g.Update(a, w, dt); }
            for (int i = 0; i < 100; ++i) { imu(V3::Zero(), V3(0, 0, 0.1), a, w); if (g.Update(a, w, dt) && i > 5) z++; }
            CHECK(z == 0 && g.airborne(), "ground: turning on the spot stops ZUPT and latches");
        }
        // (f) End to end: 10 s parked, then 5 s at 2 m/s^2 (truth 10 m/s at the end).
        // With the IMU monitor the velocity comes out right. Control: the old rule
        // (ZUPT until GNSS speed > 2 m/s) holds velocity at zero for the first second.
        auto takeoff = [&](bool gnss_rule) {
            Eskf e; GroundMonitor g; make(e, g);
            V3 a, w; double v_true = 0.0, t_z = 0.0;
            for (int i = 0; i < 1500; ++i) {
                const double acc = i >= 1000 ? 2.0 : 0.0;
                imu(V3(acc, 0, 0), V3::Zero(), a, w);
                e.Propagate(a, w, dt);
                v_true += acc * dt; t_z += dt;
                if (gnss_rule) { if (v_true <= 2.0 && t_z >= 0.1) { e.ApplyZupt(); t_z = 0.0; } }
                else if (g.Update(a, w, dt)) e.ApplyZupt();
            }
            return std::abs(e.vel().x() - v_true);
        };
        const double err_imu = takeoff(false), err_gnss = takeoff(true);
        std::printf("      take-off velocity error: IMU monitor %.2f m/s, GNSS-speed rule %.2f m/s\n",
                    err_imu, err_gnss);
        CHECK(err_imu < 0.3, "ground: take-off velocity right with IMU-driven ZUPT");
        CHECK(err_gnss > 0.5, "ground: control - GNSS-speed rule corrupts take-off velocity");
    }

    // TEST-019 (R1.5): consistency statistics against known distributions. Errors
    // drawn from the covariance the "filter" reports must give the chi-square values;
    // errors twice the reported sigma (overconfident by 2x) must be caught.
    {
        std::mt19937 rng(19);
        std::normal_distribution<double> n01(0.0, 1.0);
        Eigen::Matrix2d P; P << 25.0, 6.0, 6.0, 9.0;           // correlated, not round
        const Eigen::Matrix2d L = P.llt().matrixL();
        const double sig = std::sqrt(P.trace());
        ConsistencyStats ok, over;
        for (int i = 0; i < 200000; ++i) {
            const Eigen::Vector2d z(n01(rng), n01(rng));
            ok.AddPosition(L * z, P, sig);
            over.AddPosition(2.0 * L * z, P, sig);
            const double S = 4.0, in = std::sqrt(S) * n01(rng);
            ok.AddInnovation(in, S); over.AddInnovation(2.0 * in, S);
        }
        std::printf("      consistent: NEES mean %.3f median %.3f >9 %.2f%%, over 3 sigma %.3f%%, NIS mean %.3f >9 %.2f%%\n",
                    ok.nees_mean(), ok.nees_median(), ok.nees_over9_pct(), ok.over3sigma_pct(),
                    ok.nis_mean(), ok.nis_over9_pct());
        std::printf("      2x overconfident: NEES mean %.2f >9 %.1f%%, over 3 sigma %.2f%%, NIS mean %.2f\n",
                    over.nees_mean(), over.nees_over9_pct(), over.over3sigma_pct(), over.nis_mean());
        CHECK(std::abs(ok.nees_mean() - 2.0) < 0.03 && std::abs(ok.nees_median() - 1.386) < 0.03,
              "consistency: TEST-019 NEES matches chi-square(2)");
        CHECK(std::abs(ok.nees_over9_pct() - 1.11) < 0.1, "consistency: 1.1% outside the 3-sigma ellipse");
        CHECK(ok.over3sigma_pct() < 0.1, "consistency: published-sigma check passes REQ-INT-001 when consistent");
        CHECK(std::abs(ok.nis_mean() - 1.0) < 0.02 && std::abs(ok.nis_over9_pct() - 0.27) < 0.05,
              "consistency: NIS matches chi-square(1)");
        CHECK(over.nees_mean() > 7.5 && over.over3sigma_pct() > 1.0 && over.nis_mean() > 3.8,
              "consistency: 2x overconfidence fails NEES, NIS and REQ-INT-001");
        ConsistencyStats m; m.Merge(ok); m.Merge(over);
        CHECK(m.positions() == 400000 && std::abs(m.nees_mean() - 0.5 * (ok.nees_mean() + over.nees_mean())) < 1e-9,
              "consistency: merge pools samples");
        ConsistencyStats z; z.AddPosition(Eigen::Vector2d(1.0, 0.0), Eigen::Matrix2d::Zero(), 0.0);
        CHECK(z.nees_mean() > 1e9 && z.over3sigma_pct() == 100.0, "consistency: collapsed covariance flagged");
    }

    // TEST-020 (R1.3): a truth world map and an onboard map with different grid centres
    // must share one local frame. Same ground, centres 0.004 deg N and 0.003 deg E apart
    // (about 450 m and 230 m). Control: each map's own centre as origin gives the same
    // local coordinate two different places on Earth.
    {
        const std::string pw = "/tmp/gdn_selftest_geo_world.bin", pd = "/tmp/gdn_selftest_geo_db.bin";
        WriteGeoMap(pw, 45.9632, 7.6440);
        WriteGeoMap(pd, 45.96757, 7.64731);   // not a whole number of cells
        MapDb world, db, db_own;
        const bool ok = world.Load(pw) && db.Load(pd) && db_own.Load(pd);
        db.SetOrigin(world.lat_c(), world.lon_c());
        double worst = 0.0, worst_own = 0.0, worst_g = 0.0;
        for (double n = -1200; n <= 1200; n += 97)
            for (double e = -900; e <= 900; e += 83) {
                const MapQuery qw = world.Query(n, e), qd = db.Query(n, e), qo = db_own.Query(n, e);
                if (!(qw.ok && qd.ok && qo.ok)) continue;
                worst = std::max(worst, std::abs(qw.h - qd.h));
                worst_own = std::max(worst_own, std::abs(qw.h - qo.h));
                worst_g = std::max(worst_g, std::hypot(qw.dn - qd.dn, qw.de - qd.de));
            }
        std::printf("      same ground, different centres: max height mismatch %.2f m with a shared origin, "
                    "%.1f m with each map's own centre\n", worst, worst_own);
        CHECK(ok && worst < 2.0 && worst_g < 0.05, "map_db: TEST-020 shared origin puts two maps in one frame");
        CHECK(worst_own > 50.0, "map_db: control - own centres give the same coordinate different ground");
        CHECK(std::abs(db.grid_centre_lat() - 45.96757) < 1e-9 && std::abs(db.lat_c() - world.lat_c()) < 1e-12,
              "map_db: grid centre kept, origin moved");
    }

    // TEST-021 (R1.4): covariance consistency. Each check states what an honest filter
    // must do; each failed before the fix.
    {
        auto aligned = [&](Eskf& e) {
            bool ok = false;
            for (int i = 0; i < 700 && !ok; ++i) ok = e.FeedStatic(a0, w0);
            return ok;
        };
        // (a) Map gradient = slope of the surface the map actually returns (bilinear).
        // Central differences at the cell corner disagree with the interpolated height,
        // so the filter's H doesn't match its own prediction.
        {
            const std::string pg = "/tmp/gdn_selftest_geo_grad.bin";
            WriteGeoMap(pg, 45.9632, 7.6440);
            MapDb g; g.Load(pg);
            double worst = 0.0, scale = 0.0;
            for (int i = 8; i < 56; i += 3)
                for (int j = 8; j < 56; j += 3) {
                    const double n = g.NorthOf(i) + 0.37 * (g.NorthOf(i + 1) - g.NorthOf(i));
                    const double e = g.EastOf(j) + 0.61 * (g.EastOf(j + 1) - g.EastOf(j));
                    const MapQuery q = g.Query(n, e);
                    const double fdn = (g.Query(n + 0.05, e).h - g.Query(n - 0.05, e).h) / 0.1;
                    const double fde = (g.Query(n, e + 0.05).h - g.Query(n, e - 0.05).h) / 0.1;
                    worst = std::max(worst, std::hypot(q.dn - fdn, q.de - fde));
                    scale = std::max(scale, std::hypot(fdn, fde));
                }
            std::printf("      map gradient vs slope of the interpolated height: worst mismatch %.4f (slopes up to %.2f)\n",
                        worst, scale);
            CHECK(worst < 1e-3, "map_db: TEST-021 gradient matches the bilinear surface");
        }
        // (b) An update must not shrink the variance of a state whose correction it
        // throws away. ApplyTrn discards baro and mag-bias corrections; ApplyBaro
        // discards mag bias.
        {
            const std::string ps = "/tmp/gdn_selftest_map_slope.bin";
            WritePlaneMap(ps, 0.2, 500.0, 0.1);
            MapDb sm; sm.Load(ps);
            EskfConfig c; c.r_agl = 4.0;
            Eskf e(c); aligned(e); e.TransferAlignYaw(0.0);
            e.SetAltitudeDatum(1000.0, 5.0); e.InitBaroBias(2.0, 1.5);
            for (int k = 0; k < 20; ++k) {        // AGL couples position, altitude and baro bias
                e.Propagate(a0, w0, 0.1);
                e.ApplyAgl(1000.0 - 500.0 + 1.0, sm.Query(e.pos().x(), e.pos().y()));
                e.ApplyBaro(2.0);
            }
            const Eskf::M19 P0 = e.P();
            e.ApplyTrn(500.0 + 3.0, sm.Query(e.pos().x(), e.pos().y()));
            const Eskf::M19 P1 = e.P();
            const double d_trn = std::abs(P1(15, 15) - P0(15, 15)) + (P1.block<3,3>(16,16) - P0.block<3,3>(16,16)).norm();
            e.ApplyBaro(2.5);
            const Eskf::M19 P2 = e.P();
            const double d_baro = (P2.block<3,3>(16,16) - P1.block<3,3>(16,16)).norm();
            std::printf("      variance change of uncorrected states: TRN on baro/mag bias %.2e, baro on mag bias %.2e\n",
                        d_trn, d_baro);
            CHECK(std::abs(P0(15, 0)) + std::abs(P0(15, 1)) > 1e-6, "eskf: (test is meaningful) baro bias correlated with position");
            CHECK(d_trn < 1e-12 && d_baro < 1e-12, "eskf: updates leave the variance of states they don't correct");
        }
        // (c) A clamped correction must leave a covariance that knows the rest of the
        // error. 5 m/s of velocity error, one ZUPT, 0.5 m/s velocity clamp.
        {
            Eskf e; aligned(e); e.TransferAlignYaw(0.0);   // heading known: P_vel is just the inflation
            for (int k = 0; k < 50; ++k) e.Propagate(a0 + V3(1.0, 0, 0), w0, 0.1);   // 5 m/s of drift
            e.InflateForGap(0.0, 5.0);
            e.ApplyZupt();
            const double v = e.vel().norm(), sv = std::sqrt(e.P()(3, 3) + e.P()(4, 4) + e.P()(5, 5));
            std::printf("      after one clamped ZUPT: velocity error %.2f m/s, claimed sigma %.3f m/s (%.0f sigma)\n",
                        v, sv, v / sv);
            CHECK(v > 1.0, "eskf: (test is meaningful) the clamp left most of the error");
            CHECK(v < 3.0 * sv, "eskf: clamped update keeps the covariance honest");
        }
        // (d) Magnetometer, heading only (OI-012). A body-fixed disturbance on the
        // vertical axis (motor current on a multirotor) must not tilt the attitude. Level
        // and still; 60 s of clean field, then 6 uT on body z for 60 s.
        auto tilt_after = [&](bool heading_only) {
            EskfConfig c; c.mag_ref_north = 21.5; c.mag_ref_east = 1.3; c.mag_ref_down = 42.3;
            Eskf e(c); aligned(e); e.TransferAlignYaw(0.0);
            const V3 m(21.5, 1.3, 42.3);
            for (int k = 0; k < 1200; ++k) {
                e.Propagate(a0, w0, 0.1);
                const V3 z = m + (k >= 600 ? V3(0, 0, 6.0) : V3::Zero());
                if (heading_only) e.ApplyMagHeading(z); else e.ApplyMag(z);
            }
            const V3 down_b = e.q().conjugate() * V3::UnitZ();       // body view of "down"
            return std::acos(std::clamp(down_b.z(), -1.0, 1.0)) / Eskf::kDeg;
        };
        int auto_false = -1;
        auto tilt_auto = [&]() {
            EskfConfig c; c.mag_ref_north = 21.5; c.mag_ref_east = 1.3; c.mag_ref_down = 42.3;
            Eskf e(c); aligned(e); e.TransferAlignYaw(0.0);
            std::mt19937 rng(23); std::normal_distribution<double> nm(0.0, 2.0);
            const V3 m(21.5, 1.3, 42.3);
            for (int k = 0; k < 1200; ++k) {
                e.Propagate(a0, w0, 0.1);
                e.ApplyMagAuto(m + (k >= 600 ? V3(0, 0, 6.0) : V3::Zero()) + V3(nm(rng), nm(rng), nm(rng)));
                if (k == 599) auto_false = e.mag_heading_updates();   // switches on a clean field
            }
            const V3 down_b = e.q().conjugate() * V3::UnitZ();
            std::printf("      auto: %d 3-axis and %d heading-only updates\n", e.mag_full_updates(), e.mag_heading_updates());
            return std::acos(std::clamp(down_b.z(), -1.0, 1.0)) / Eskf::kDeg;
        };
        const double tilt_full = tilt_after(false), tilt_head = tilt_after(true), tilt_au = tilt_auto();
        std::printf("      tilt after a 6 uT vertical disturbance: 3-axis %.2f deg, heading-only %.2f deg\n",
                    tilt_full, tilt_head);
        CHECK(tilt_full > 1.0, "eskf: (control) 3-axis mag update tilts the attitude");
        CHECK(tilt_head < 0.2, "eskf: heading-only mag leaves the tilt alone");
        // Auto needs about 1 s of averaging to see a 6 uT step (the per-sample noise is
        // 2 uT), and the 3-axis updates in that window tilt the attitude. So it cuts the
        // disturbance tilt rather than removing it: the check asks for at least 30% less
        // than 3-axis, and no switching on a clean field.
        std::printf("      auto (3-axis while the field looks clean): tilt %.2f deg, %d switches on the clean field\n",
                    tilt_au, auto_false);
        CHECK(tilt_au < 0.7 * tilt_full && auto_false == 0,
              "eskf: auto mag cuts disturbance tilt, keeps 3-axis on a clean field");
        // (e) Heading-only still finds heading: 10 deg initial error, a calibrated compass
        // (0.3 uT residual hard iron, unknown to the filter), turning 360 deg in 60 s,
        // 2 uT noise. Heading-only fusion assumes calibration (see ApplyMagHeading).
        {
            EskfConfig c; c.mag_ref_north = 21.5; c.mag_ref_east = 1.3; c.mag_ref_down = 42.3;
            Eskf e(c); aligned(e); e.TransferAlignYaw(10.0 * Eskf::kDeg);
            std::mt19937 rng(21); std::normal_distribution<double> nm(0.0, 2.0);
            const double rate = 2.0 * M_PI / 60.0;
            const V3 m(21.5, 1.3, 42.3), b(0.2, -0.2, 0.1);
            double err = 0.0;
            for (int k = 0; k < 1200; ++k) {
                e.Propagate(a0, V3(0, 0, rate), 0.1);
                const double psi = rate * 0.1 * (k + 1);
                const Eigen::Matrix3d Rz = Eigen::AngleAxisd(psi, V3::UnitZ()).toRotationMatrix();
                e.ApplyMagHeading(Rz.transpose() * m + b + V3(nm(rng), nm(rng), nm(rng)));
                const V3 f = e.q() * V3::UnitX();
                err = std::abs(std::remainder(std::atan2(f.y(), f.x()) - psi, 2 * M_PI)) / Eskf::kDeg;
            }
            std::printf("      heading-only, turning, calibrated compass: final heading error %.2f deg\n", err);
            CHECK(err < 2.0, "eskf: heading-only mag converges heading");
        }
        // (f) Heading only sees the field's direction, never its strength, so it can't
        // estimate the hard-iron bias: a bias as large as the field "explains" any
        // reading. Zero true bias, straight then turning, 120 s: the bias estimate must
        // stay put and the heading must stay honest.
        {
            EskfConfig c; c.mag_ref_north = 21.5; c.mag_ref_east = 1.3; c.mag_ref_down = 42.3;
            Eskf e(c); aligned(e); e.TransferAlignYaw(0.0);
            std::mt19937 rng(22); std::normal_distribution<double> nm(0.0, 2.0);
            const V3 m(21.5, 1.3, 42.3);
            double psi = 0.0, worst_z = 0.0;
            for (int k = 0; k < 1200; ++k) {
                const double rate = (k > 600 && k < 900) ? 0.1 : 0.0;
                e.Propagate(a0, V3(0, 0, rate), 0.1);
                psi += rate * 0.1;
                const Eigen::Matrix3d Rz = Eigen::AngleAxisd(psi, V3::UnitZ()).toRotationMatrix();
                e.ApplyMagHeading(Rz.transpose() * m + V3(nm(rng), nm(rng), nm(rng)));
                const V3 f = e.q() * V3::UnitX();
                const double err = std::remainder(std::atan2(f.y(), f.x()) - psi, 2 * M_PI);
                worst_z = std::max(worst_z, std::abs(err) / std::sqrt(e.P()(8, 8)));
            }
            std::printf("      heading-only, no bias: mag bias estimate |b| = %.2f uT, worst heading error %.1f sigma\n",
                        e.mag_bias().norm(), worst_z);
            CHECK(e.mag_bias().norm() < 0.5 && worst_z < 4.0, "eskf: heading-only mag doesn't invent a hard-iron bias");
        }
    }

    // TEST-022 (R1.6): the simulated IMU. Before R1.6 the Monte Carlo added one constant
    // offset per flight to a recorded IMU, so its error had no white noise or drift of
    // its own and no stated source. The placeholder model must produce the statistics it
    // claims, because the Allan fit of the real IMUs will be checked the same way.
    {
        using V3 = Eigen::Vector3d;
        // (a) White noise density, placeholder, 100 Hz, 200 s.
        {
            const ImuNoiseModel m = ImuPlaceholder();
            ImuErrorSim sim(m, 1);
            sim.SetTurnOn(V3::Zero(), V3::Zero());
            const double dt = 0.01; const int n = 20000;
            V3 sa = V3::Zero(), sa2 = V3::Zero(), sg = V3::Zero(), sg2 = V3::Zero();
            for (int k = 0; k < n; ++k) {
                V3 a = V3::Zero(), w = V3::Zero();
                sim.Corrupt(&a, &w, dt);
                sa += a; sa2 += a.cwiseAbs2(); sg += w; sg2 += w.cwiseAbs2();
            }
            double worst = 0.0;
            for (int i = 0; i < 3; ++i) {
                const double na = std::sqrt((sa2[i] - sa[i] * sa[i] / n) / (n - 1) * dt);
                const double ng = std::sqrt((sg2[i] - sg[i] * sg[i] / n) / (n - 1) * dt);
                worst = std::max({worst, std::abs(na / m.accel_noise_density[i] - 1.0),
                                  std::abs(ng / m.gyro_noise_density[i] - 1.0)});
            }
            std::printf("      white noise density, worst axis off by %.1f%%\n", 100.0 * worst);
            CHECK(worst < 0.03, "imu: TEST-022 white noise density matches the model");
        }
        // (b) Bias drift: spread of the drift after 600 s over 300 power-ups against the
        // Gauss-Markov variance K^2 tau/2 (1 - exp(-2T/tau)).
        ImuNoiseModel sens;
        std::string err;
        const bool loaded = LoadImuModel(DataPath("config/imu/sensitivity_bias_drift.yaml"), &sens, &err);
        if (!loaded) std::printf("      %s\n", err.c_str());
        CHECK(loaded, "imu: TEST-022 sensitivity model file loads");
        if (loaded) {
            const double T = 600.0, dt = 0.1;
            double va = 0.0, vg = 0.0; int nn = 0;
            for (int r = 0; r < 300; ++r) {
                ImuErrorSim sim(sens, 100 + r);
                sim.SetTurnOn(V3::Zero(), V3::Zero());
                for (int k = 0; k < static_cast<int>(T / dt); ++k) {
                    V3 a = V3::Zero(), w = V3::Zero(); sim.Corrupt(&a, &w, dt);
                }
                va += sim.accel_bias().squaredNorm(); vg += sim.gyro_bias().squaredNorm(); nn += 3;
            }
            auto gm = [&](double K, double tau) { return K * K * tau / 2.0 * (1.0 - std::exp(-2.0 * T / tau)); };
            const double ra = std::sqrt(va / nn / gm(sens.accel_bias_rw, sens.accel_bias_tau));
            const double rg = std::sqrt(vg / nn / gm(sens.gyro_bias_rw, sens.gyro_bias_tau));
            std::printf("      drift after %.0f s: accel %.2f x model, gyro %.2f x model\n", T, ra, rg);
            CHECK(std::abs(ra - 1.0) < 0.08 && std::abs(rg - 1.0) < 0.08, "imu: TEST-022 bias drift matches Gauss-Markov");
        }
        // (c) Turn-on bias spread over 3000 power-ups.
        {
            const ImuNoiseModel m = ImuPlaceholder();
            double sa = 0.0, sg = 0.0; const int n = 3000;
            for (int r = 0; r < n; ++r) {
                ImuErrorSim sim(m, 5000 + r);
                sa += sim.accel_bias().squaredNorm(); sg += sim.gyro_bias().squaredNorm();
            }
            const double ra = std::sqrt(sa / (3 * n)) / m.accel_turn_on_sigma;
            const double rg = std::sqrt(sg / (3 * n)) / m.gyro_turn_on_sigma;
            CHECK(std::abs(ra - 1.0) < 0.05 && std::abs(rg - 1.0) < 0.05, "imu: TEST-022 turn-on bias spread matches");
        }
        // (d) The placeholder file says what ImuPlaceholder() says; a missing or misspelt
        // key is an error, not a zero.
        {
            ImuNoiseModel f;
            const bool ok = LoadImuModel(DataPath("config/imu/placeholder_px4_default.yaml"), &f, &err);
            const ImuNoiseModel b = ImuPlaceholder();
            auto rel = [](double x, double y) { return std::abs(x - y) <= 1e-4 * std::max(std::abs(y), 1e-12); };
            bool same = ok && f.name == b.name && rel(f.accel_turn_on_sigma, b.accel_turn_on_sigma) &&
                        rel(f.gyro_turn_on_sigma, b.gyro_turn_on_sigma) && f.accel_bias_rw == b.accel_bias_rw &&
                        f.gyro_bias_rw == b.gyro_bias_rw;
            for (int i = 0; i < 3; ++i)
                same = same && rel(f.accel_noise_density[i], b.accel_noise_density[i]) &&
                       rel(f.gyro_noise_density[i], b.gyro_noise_density[i]);
            CHECK(same, "imu: TEST-022 placeholder file matches ImuPlaceholder()");
            const std::string bad = "/tmp/gdn_imu_bad.yaml";
            { std::ofstream o(bad); o << "name: x\naccel_noise_density: 1e-4\ngyro_noise_density: 1e-5\n"
                                         "accel_bias_rw: 0\naccel_bias_tau: 300\ngyro_bias_rw: 0\n"
                                         "accel_turn_on_sigma: 0\ngyro_turn_on_sigma: 0\n"; }
            const bool missing = !LoadImuModel(bad, &f, &err);
            { std::ofstream o(bad, std::ios::app); o << "gyro_bias_tau: 1000\ngyro_bias_rw_typo: 1\n"; }
            const bool unknown = !LoadImuModel(bad, &f, &err);
            std::remove(bad.c_str());
            CHECK(missing && unknown, "imu: TEST-022 model file rejects missing and unknown keys");
        }
        // (e) Allan deviation of a simulated 2 h static log (100 Hz) against the model's
        // curve at 1 s, 10 s, 100 s and 600 s: the check tools/allan_fit.py relies on.
        if (loaded) {
            ImuErrorSim sim(sens, 77);
            sim.SetTurnOn(V3::Zero(), V3::Zero());
            const double dt = 0.01; const int n = 720000;
            std::vector<double> cg(n + 1, 0.0), ca(n + 1, 0.0);   // cumulative sums, x axis
            for (int k = 0; k < n; ++k) {
                V3 a = V3::Zero(), w = V3::Zero(); sim.Corrupt(&a, &w, dt);
                ca[k + 1] = ca[k] + a.x(); cg[k + 1] = cg[k] + w.x();
            }
            auto adev = [&](const std::vector<double>& c, int mm) {   // overlapping
                double s = 0.0; int cnt = 0;
                for (int k = 0; k + 2 * mm <= n; ++k) {
                    const double d = (c[k + 2 * mm] - 2.0 * c[k + mm] + c[k]) / mm;
                    s += d * d; ++cnt;
                }
                return std::sqrt(s / (2.0 * cnt));
            };
            double worst = 0.0;
            for (const int mm : {100, 1000, 10000, 60000}) {
                const double t = mm * dt;
                const double ea = ImuNoiseModel::Adev(sens.accel_noise_density.x(), sens.accel_bias_rw, sens.accel_bias_tau, t);
                const double eg = ImuNoiseModel::Adev(sens.gyro_noise_density.x(), sens.gyro_bias_rw, sens.gyro_bias_tau, t);
                const double fa = adev(ca, mm) / ea, fg = adev(cg, mm) / eg;
                std::printf("      Allan deviation at %5.0f s: accel %.2f x model, gyro %.2f x model\n", t, fa, fg);
                // One 2 h log: few independent clusters at long t, so the tolerance widens.
                const double tol = t < 50 ? 0.10 : 0.45;
                worst = std::max(worst, std::max(std::abs(fa - 1.0), std::abs(fg - 1.0)) / tol);
            }
            CHECK(worst < 1.0, "imu: TEST-022 Allan deviation of a simulated log matches the model");
        }
    }

std::printf(g_fail == 0 ? "ALL TESTS PASSED\n" : "%d TEST(S) FAILED\n", g_fail);
    return g_fail == 0 ? 0 : 1;
}
// Note: The main function needs to be modified to include this test.
// For now, just verifying the build compiles.
