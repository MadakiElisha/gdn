// TEST-012: Monte Carlo accuracy campaign for the ROS-free TRN core.
// Replays recorded IMU + truth CSVs through gdn::Eskf with seeded sensor-error
// draws (accel/gyro bias offsets, TRN noise stream). Reports pooled CEP stats
// over the 60-300 s window and per-seed RMS distribution. No ROS, no DDS.
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include "gdn/map_db.hpp"
#include "gdn/eskf.hpp"

namespace {
struct Table {
    std::map<std::string, std::vector<double>> col;
    bool Load(const std::string& path) {
        std::ifstream f(path);
        if (!f) return false;
        std::string line;
        if (!std::getline(f, line)) return false;
        std::vector<std::string> hdr;
        { std::stringstream ss(line); std::string tok;
          while (std::getline(ss, tok, ',')) {
              const std::string bad = "\xef\xbb\xbf\r\n \t";
              const size_t b = tok.find_first_not_of(bad);
              const size_t e = tok.find_last_not_of(bad);
              tok = (b == std::string::npos) ? std::string() : tok.substr(b, e - b + 1);
              if (!tok.empty()) hdr.push_back(tok);
          } }
        for (auto& h : hdr) col[h] = {};
        while (std::getline(f, line)) {
            if (line.empty()) continue;
            std::stringstream ss(line); std::string tok; size_t i = 0;
            while (std::getline(ss, tok, ',')) {
                if (i < hdr.size()) col[hdr[i]].push_back(std::atof(tok.c_str()));
                ++i;
            }
        }
        return !hdr.empty() && !col[hdr[0]].empty();
    }
    const std::vector<double>& operator[](const std::string& k) const {
        if (col.find(k) == col.end()) {
            std::fprintf(stderr, "ERROR: CSV missing column '%s'\n", k.c_str());
            std::exit(1);
        }
        return col.at(k);
    }
};
double Percentile(std::vector<double> v, double p) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const double idx = p / 100.0 * (v.size() - 1);
    const size_t lo = static_cast<size_t>(idx);
    const size_t hi = lo + 1 < v.size() ? lo + 1 : lo;
    const double f = idx - static_cast<double>(lo);
    return v[lo] * (1 - f) + v[hi] * f;
}
}  // namespace

int main(int argc, char** argv) {
    const int nseeds = argc > 1 ? std::atoi(argv[1]) : 100;
    const std::string csv_dir = argc > 2 ? argv[2]
        : "/home/madakie/gdn_workspace/data/flight001_csv";
    const std::string map_path = argc > 3 ? argv[3]
        : "/home/madakie/gdn_workspace/data/maps/terrain_db_sigma4.bin";

    Table imu, lpos;
    if (!imu.Load(csv_dir + "/imu.csv") || !lpos.Load(csv_dir + "/lpos.csv")) {
        std::printf("ERROR: cannot load CSVs from %s\n", csv_dir.c_str()); return 1;
    }
    gdn::MapDb world;
    if (!world.Load(map_path)) { std::printf("ERROR: cannot load %s\n", map_path.c_str()); return 1; }

    const std::vector<double>& it = imu["t"], ax = imu["ax"], ay = imu["ay"], az = imu["az"],
                               gx = imu["gx"], gy = imu["gy"], gz = imu["gz"];
    const std::vector<double>& lt = lpos["t"], lx = lpos["x"], ly = lpos["y"],
                               lvx = lpos["vx"], lvy = lpos["vy"];
    const double t0 = it.front();

    std::vector<double> pool, seed_rms, seed_max;
    int pass_seeds = 0;

    for (int seed = 0; seed < nseeds; ++seed) {
        std::mt19937_64 rng(0x47444eULL + static_cast<uint64_t>(seed));
        std::normal_distribution<double> d_n5(0, 5), d_n2(0, 2),
                                         d_ba(0, 0.05), d_bg(0, 0.02 * gdn::Eskf::kDeg);
        const int focus = argc > 4 ? std::atoi(argv[4]) : -1;
        const bool diag_mode = (nseeds == 1) || (seed == focus);
        const bool zero_bias = (nseeds == 1);   // 1-seed = zero-bias diagnostic
        const Eigen::Vector3d ba_off = zero_bias ? Eigen::Vector3d::Zero()
                               : Eigen::Vector3d(d_ba(rng), d_ba(rng), d_ba(rng));
        const Eigen::Vector3d bg_off = zero_bias ? Eigen::Vector3d::Zero()
                               : Eigen::Vector3d(d_bg(rng), d_bg(rng), d_bg(rng));

        gdn::Eskf eskf;                      // default config == bench config
        double prev_t = -1.0, se = 0.0, smax = 0.0; int sn = 0, last_k = -1;
        bool yaw_done = false; size_t j = 0;

        for (size_t i = 0; i < it.size(); ++i) {
            const double t = (it[i] - t0) * 1e-6;
            const Eigen::Vector3d a_m(ax[i] + ba_off.x(), ay[i] + ba_off.y(), az[i] + ba_off.z());
            const Eigen::Vector3d w_m(gx[i] + bg_off.x(), gy[i] + bg_off.y(), gz[i] + bg_off.z());
            if (!eskf.aligned()) {
            if (eskf.FeedStatic(a_m, w_m) && diag_mode) {
                const Eigen::Vector3d r0 = eskf.q().toRotationMatrix().eulerAngles(2,1,0)/gdn::Eskf::kDeg;
                std::printf("ALIGN t=%.1f ba=%.4f bg=%.4f yaw=%.1f pit=%.1f rol=%.1f\n", t,
                    eskf.accel_bias().norm(), eskf.gyro_bias().norm()/gdn::Eskf::kDeg,
                    r0.x(), r0.y(), r0.z());
            }
            prev_t = t; continue;
        }
            const double dt = t - prev_t; prev_t = t;
            if (dt <= 0) continue;
            eskf.Propagate(a_m, w_m, dt);

            while (j + 1 < lt.size() && (lt[j + 1] - t0) * 1e-6 <= t) ++j;
            const double spd = std::hypot(lvx[j], lvy[j]);
            if (!yaw_done && spd > 3.0 &&
                std::abs(w_m.z() - eskf.gyro_bias().z()) < 0.05) {
                eskf.TransferAlignYaw(std::atan2(lvy[j], lvx[j]));
                yaw_done = true;
                if (diag_mode) std::printf("YAW t=%.1f yaw=%.1f spd=%.1f pb=(%.1f,%.1f)\n", t,
                    std::atan2(lvy[j], lvx[j])/gdn::Eskf::kDeg, spd, eskf.pos().x(), eskf.pos().y());
            }

            const int k = static_cast<int>(std::floor(t * 2.0));   // 0.5 s grid
            if (k == last_k || !yaw_done) { last_k = k; continue; }
            last_k = k;
            const gdn::MapQuery wq = world.Query(lx[j], ly[j]);
            if (wq.ok) {
                const double meas = wq.h + (diag_mode ? 0.0 : (d_n5(rng) - d_n2(rng)));
                const gdn::MapQuery mq = world.Query(eskf.pos().x(), eskf.pos().y());
                double innov = 0, S = 0;
                const int kc = eskf.updates() + eskf.rejects();   // bench-equivalent k
                const auto res = eskf.ApplyTrn(meas, mq, &innov, &S);
            if (diag_mode && (kc < 12 || focus >= 0)) {
                const Eigen::Vector3d fwd_n = eskf.q() * Eigen::Vector3d::UnitX();
                const double yaw_est = std::atan2(fwd_n.y(), fwd_n.x()) / gdn::Eskf::kDeg;
                const double yaw_truth = std::atan2(lvy[j], lvx[j]) / gdn::Eskf::kDeg;
                std::printf("UPD k=%d meas=%.2f h0=%.2f innov=%.2f S=%.1f acc=%d pb=(%.1f,%.1f) yawE=%.1f yawT=%.1f\n",
                    kc, meas, mq.h, innov, S,
                    res == gdn::Eskf::UpdResult::kApplied ? 1 : 0,
                    eskf.pos().x(), eskf.pos().y(), yaw_est, yaw_truth);
            }
            }
            const double e = std::hypot(eskf.pos().x() - lx[j], eskf.pos().y() - ly[j]);
            if (diag_mode && t >= 25.0 && std::fmod(t, 1.0) < 0.5)
                std::printf("t=%3.0f err=%7.1f pos=(%7.1f,%7.1f) truth=(%7.1f,%7.1f) sp=%5.1f\n",
                            t, e, eskf.pos().x(), eskf.pos().y(), lx[j], ly[j],
                            eskf.sigma_pos());
            if (t >= 60.0 && t <= 300.0) {
                pool.push_back(e); se += e * e; sn++; if (e > smax) smax = e;
            }
        }
        const double rms = sn > 0 ? std::sqrt(se / sn) : 1e9;
        seed_rms.push_back(rms); seed_max.push_back(smax);
        if (rms < 100.0) ++pass_seeds;
    }

    int in_spec = 0; for (double e : pool) if (e < 100.0) ++in_spec;
    std::printf("MC campaign: %d seeds, window 60-300 s, pooled n=%zu\n",
                nseeds, pool.size());
    std::printf("CEP50: %.1f m  CEP95: %.1f m  MAX: %.1f m  in-spec(<100 m): %.1f%%\n",
                Percentile(pool, 50), Percentile(pool, 95), Percentile(pool, 100),
                pool.empty() ? 0.0 : 100.0 * in_spec / pool.size());
    std::printf("per-seed RMS: median %.1f m  p95 %.1f m  worst %.1f m  passing(<100 m): %d/%d\n",
                Percentile(seed_rms, 50), Percentile(seed_rms, 95),
                Percentile(seed_rms, 100), pass_seeds, nseeds);
    { std::ofstream o("/tmp/mc_per_seed.csv"); o << "seed,rms,max\n";
      for (size_t s = 0; s < seed_rms.size(); ++s)
          o << s << "," << seed_rms[s] << "," << seed_max[s] << "\n"; }
    return 0;
}
