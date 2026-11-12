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
#include "gdn/consistency.hpp"
#include "gdn/imu_model.hpp"
#include "gdn/ground_monitor.hpp"
#include "gdn/paths.hpp"

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
    bool has(const std::string& k) const { return col.find(k) != col.end(); }
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
    const double idx = p / 100.0 * static_cast<double>(v.size() - 1);
    const size_t lo = static_cast<size_t>(idx);
    const size_t hi = lo + 1 < v.size() ? lo + 1 : lo;
    const double f = idx - static_cast<double>(lo);
    return v[lo] * (1 - f) + v[hi] * f;
}
}  // namespace

int main(int argc_all, char** argv_all) {
    // Usage: mc_campaign [nseeds] [csv_dir] [map.bin] [focus_seed] [--options]
    //   --model=ideal  TRN measurement is terrain height itself (the original oracle)
    //   --model=naive  terrain = baro altitude - measured AGL, with no baro-bias state:
    //                  what a real system gets if it ignores baro error
    //   --model=agl    measured AGL plus baro through ApplyAgl / ApplyBaro (R1.1)
    //   --baro-drift=M baro bias grows by M metres over the first 300 s after TRN
    //                  starts (pressure change). Both naive and agl calibrate the
    //                  initial bias on the ground, so only the drift is unknown.
    //   --clearance=M  minimum height above the map along the truth path (default 30)
    //   --window=A,B   scoring window in seconds (default 60,300)
    //   --q-baro=Q     baro-bias random walk, m^2/s (EskfConfig q_baro_bias)
    //   --r-agl=R      AGL measurement variance, m^2 (EskfConfig r_agl)
    //   --r-trn=R      TRN measurement variance, m^2 (EskfConfig r_trn)
    //   --startup=truth  heading from truth GNSS velocity at take-off (TransferAlignYaw);
    //                  TRN starts then. The original MC, kept as the CI regression path.
    //   --startup=node   what eskf_node does (R1.2): magnetometer heading while parked,
    //                  ZUPT from the IMU ground monitor, mag updates throughout, TRN
    //                  from yaw alignment on. The mag is simulated as mag_sim.py does:
    //                  R^T m_ref + hard iron (2,-1,1) uT + N(0, 2 uT), 10 Hz.
    //   --ground=imu|gnss  node start-up only: IMU ground monitor (default), or the
    //                  pre-R1.2 rule (ZUPT until GNSS speed > 2 m/s) for comparison
    //   --db-map=PATH  onboard map the filter navigates with (R1.3). The positional map
    //                  stays the truth world that measurements are simulated from.
    //                  Default: the same file (no map error). The onboard map is put in
    //                  the world map's frame (MapDb::SetOrigin). Build degraded maps with
    //                  tools/make_data.sh --db-maps.
    //   --seed0=N      first seed number (default 0): seeds N .. N+nseeds-1. Lets several
    //                  runs (e.g. one per onboard map, tools/mc_map_ensemble.py) use
    //                  distinct flight-noise draws.
    //   --hard-iron=S  node start-up only: scale on the simulated hard-iron bias (2,-1,1) uT.
    //                  Default 0.15 (about 0.35 uT): a calibrated compass, as mag_sim.py
    //                  (R1.4). 1 is the uncalibrated value used before R1.4.
    //   --trace=FILE   write every scored sample of every seed (0.5 s grid, from yaw
    //                  alignment on) as CSV: seed,t,x,y,tx,ty,err,sigma,nees.
    //                  tools/plot_consistency.py draws it.
    //   --mag=auto|heading|full|align  node start-up only, as the node's mag_fusion
    //                  parameter: auto (default, R1.4: 3-axis while the field looks
    //                  clean, else heading only), heading-only, 3-axis (the pre-R1.4
    //                  default), or initial heading only. The
    //                  simulated mag assumes level flight along the velocity vector, true
    //                  for the synthetic Zermatt flight but not for a real recording such
    //                  as flight001 (bank, crab), whose CSVs carry no attitude truth.
    //   --imu=auto|recorded|placeholder|FILE  IMU errors (R1.6). recorded: imu.csv as
    //                  logged plus one constant offset per flight (the pre-R1.6 MC).
    //                  placeholder: the generic MEMS placeholder (PX4's default IMU noise,
    //                  config/imu/placeholder_px4_default.yaml) applied to the flight's
    //                  error-free imu_truth.csv. FILE: any config/imu/ model. auto
    //                  (default): config/imu/h30wp.yaml (the H30WP's Allan fit, TEST-024)
    //                  when imu_truth.csv exists (synthetic flights), else recorded (a
    //                  real log's IMU errors are the real ones).
    //                  The "imu:" line of the report names what was used.
    //   --q-ba=K, --q-bg=K  filter accel-bias (m/s^3/sqrt(Hz)) and gyro-bias
    //                  (deg/s^2/sqrt(Hz)) random walk, for sweeps.
    //   --q-an=N, --q-gn=N  filter accel (m/s^2/sqrt(Hz)) and gyro (deg/s/sqrt(Hz))
    //                  white noise density, for sweeps (R1.7).
    //   --start-error=M  R1.7: the filter starts M metres from the truth, in a random
    //                  direction per flight, and is told sigma max(M, 10) m per axis.
    //                  Applied when heading is aligned (parked for the node start-up, at
    //                  the take-off roll for truth). Reports REQ-ACQ-001 acquisition.
    //   --gnss-loss=T  R1.7: simulated GNSS (5 Hz, 1.5 m and 0.1 m/s per axis, white) is
    //                  fused until log time T s, then lost. The filter keeps the state and
    //                  covariance it had (covariance handover). Scoring starts at T.
    //   --gnss-free=D1,D2  R1.7: with --gnss-loss, also report CEP50/CEP95/max and the
    //                  3-sigma check pooled over [T, T+D] for each D (default 300,600: the
    //                  5 and 10 minutes of docs/requirements.md). The main window becomes
    //                  [T, T+max D] unless --window is given.
    //   --q-imu=match  set the filter's IMU process noise from the IMU model (white
    //                  noise; bias random walk where the model has drift) instead of the
    //                  tuned EskfConfig values.
    // Relative paths resolve against $GDN_ROOT (exported by env.sh).
    std::vector<char*> pos_args;
    std::string model = "ideal", startup = "truth", ground_mode = "imu", mag_mode = "auto", trace_path, db_arg;
    std::string imu_arg = "auto", q_imu;
    double baro_drift = 0.0, clearance = 30.0, w0 = 60.0, w1 = 300.0;
    bool window_set = false, start_set = false;
    double start_err = 0.0, gnss_loss = -1.0;            // R1.7
    std::vector<double> gnss_free;
    int seed0 = 0;
    double hard_iron = 0.15;
    gdn::EskfConfig cfg;   // defaults == node defaults
    for (int a = 0; a < argc_all; ++a) {
        const std::string s = argv_all[a];
        if (s.rfind("--model=", 0) == 0) model = s.substr(8);
        else if (s.rfind("--startup=", 0) == 0) startup = s.substr(10);
        else if (s.rfind("--ground=", 0) == 0) ground_mode = s.substr(9);
        else if (s.rfind("--mag=", 0) == 0) mag_mode = s.substr(6);
        else if (s.rfind("--trace=", 0) == 0) trace_path = s.substr(8);
        else if (s.rfind("--db-map=", 0) == 0) db_arg = s.substr(9);
        else if (s.rfind("--imu=", 0) == 0) imu_arg = s.substr(6);
        else if (s.rfind("--q-imu=", 0) == 0) q_imu = s.substr(8);
        else if (s.rfind("--seed0=", 0) == 0) seed0 = std::atoi(s.c_str() + 8);
        else if (s.rfind("--hard-iron=", 0) == 0) hard_iron = std::atof(s.c_str() + 12);
        else if (s.rfind("--baro-drift=", 0) == 0) baro_drift = std::atof(s.c_str() + 13);
        else if (s.rfind("--clearance=", 0) == 0) clearance = std::atof(s.c_str() + 12);
        else if (s.rfind("--window=", 0) == 0) { std::sscanf(s.c_str() + 9, "%lf,%lf", &w0, &w1); window_set = true; }
        else if (s.rfind("--start-error=", 0) == 0) { start_err = std::atof(s.c_str() + 14); start_set = true; }
        else if (s.rfind("--gnss-loss=", 0) == 0) gnss_loss = std::atof(s.c_str() + 12);
        else if (s.rfind("--gnss-free=", 0) == 0) {
            std::stringstream ss(s.substr(12)); std::string tok;
            while (std::getline(ss, tok, ',')) if (!tok.empty()) gnss_free.push_back(std::atof(tok.c_str()));
        }
        else if (s.rfind("--q-baro=", 0) == 0) cfg.q_baro_bias = std::atof(s.c_str() + 9);
        else if (s.rfind("--r-agl=", 0) == 0) cfg.r_agl = std::atof(s.c_str() + 8);
        else if (s.rfind("--r-trn=", 0) == 0) cfg.r_trn = std::atof(s.c_str() + 8);
        else if (s.rfind("--q-ba=", 0) == 0) cfg.q_accel_bias_rw = std::atof(s.c_str() + 7);
        else if (s.rfind("--q-bg=", 0) == 0) cfg.q_gyro_bias_rw = std::atof(s.c_str() + 7) * gdn::Eskf::kDeg;
        else if (s.rfind("--q-an=", 0) == 0) cfg.q_accel_nd = std::atof(s.c_str() + 7);
        else if (s.rfind("--q-gn=", 0) == 0) cfg.q_gyro_nd = std::atof(s.c_str() + 7) * gdn::Eskf::kDeg;
        else if (s.rfind("--", 0) == 0) { std::printf("ERROR: unknown option %s\n", s.c_str()); return 1; }
        else pos_args.push_back(argv_all[a]);
    }
    if (model != "ideal" && model != "naive" && model != "agl") {
        std::printf("ERROR: --model must be ideal, naive or agl\n"); return 1;
    }
    if ((startup != "truth" && startup != "node") || (ground_mode != "imu" && ground_mode != "gnss") ||
        (mag_mode != "heading" && mag_mode != "full" && mag_mode != "align" && mag_mode != "auto")) {
        std::printf("ERROR: --startup must be truth or node, --ground imu or gnss, --mag auto, heading, full or align\n");
        return 1;
    }
    const bool node_startup = startup == "node";
    // R1.7 scenario options.
    if (start_err < 0 || (gnss_loss >= 0 && start_err > 0)) {
        std::printf("ERROR: --start-error=M needs M >= 0, and applies to flights without GNSS (acquisition):\n"
                    "  with --gnss-loss the filter starts from GNSS, so the two don't combine\n");
        return 1;
    }
    if (!gnss_free.empty() && gnss_loss < 0) {
        std::printf("ERROR: --gnss-free=D scores the D seconds after GNSS is lost: it needs --gnss-loss=T\n");
        return 1;
    }
    if (gnss_loss >= 0 && gnss_free.empty()) gnss_free = {300.0, 600.0};
    std::sort(gnss_free.begin(), gnss_free.end());
    if (gnss_loss >= 0 && !window_set) { w0 = gnss_loss; w1 = gnss_loss + gnss_free.back(); }
    const int argc = static_cast<int>(pos_args.size());
    char** argv = pos_args.data();
    const int nseeds = argc > 1 ? std::atoi(argv[1]) : 100;
    const std::string csv_dir = gdn::DataPath(argc > 2 ? argv[2] : "data/flight001_csv");
    const std::string map_path = gdn::DataPath(argc > 3 ? argv[3] : gdn::kDefaultMap);

    // IMU errors (R1.6).
    const bool have_truth_imu = static_cast<bool>(std::ifstream(csv_dir + "/imu_truth.csv"));
    if (imu_arg == "auto") imu_arg = have_truth_imu ? "config/imu/h30wp.yaml" : "recorded";
    const bool use_imu_model = imu_arg != "recorded";
    gdn::ImuNoiseModel imu_model;
    if (imu_arg == "placeholder") {
        imu_model = gdn::ImuPlaceholder();
    } else if (use_imu_model) {
        std::string err;
        if (!gdn::LoadImuModel(gdn::DataPath(imu_arg), &imu_model, &err)) {
            std::printf("ERROR: --imu: %s\n", err.c_str()); return 1;
        }
    }
    if (use_imu_model && !have_truth_imu) {
        std::printf("ERROR: --imu=%s needs an error-free IMU (imu_truth.csv) in %s. A recorded flight's\n"
                    "  IMU already carries the real sensor's errors: use --imu=recorded.\n",
                    imu_arg.c_str(), csv_dir.c_str());
        return 1;
    }
    if (!q_imu.empty() && (q_imu != "match" || !use_imu_model)) {
        std::printf("ERROR: --q-imu=match is the only value, and needs an IMU model (--imu)\n"); return 1;
    }
    if (q_imu == "match") {
        cfg.q_accel_nd = imu_model.accel_noise_density.maxCoeff();
        cfg.q_gyro_nd = imu_model.gyro_noise_density.maxCoeff();
        if (imu_model.accel_bias_rw > 0) cfg.q_accel_bias_rw = imu_model.accel_bias_rw;
        if (imu_model.gyro_bias_rw > 0) cfg.q_gyro_bias_rw = imu_model.gyro_bias_rw;
    }
    if (use_imu_model)
        std::printf("imu: %s (%s) on imu_truth.csv%s\n", imu_model.name.c_str(), imu_model.source.c_str(),
                    q_imu == "match" ? "; filter process noise matched to it" : "");
    else
        std::printf("imu: recorded (imu.csv as logged, plus a per-flight offset of 0.05 m/s^2 and 0.02 deg/s)\n");

    Table imu, lpos;
    if (!imu.Load(csv_dir + (use_imu_model ? "/imu_truth.csv" : "/imu.csv")) || !lpos.Load(csv_dir + "/lpos.csv")) {
        std::printf("ERROR: cannot load imu.csv/lpos.csv from %s\n"
                    "  (flight001: tools/fetch_data.sh; zermatt: tools/make_data.sh)\n", csv_dir.c_str()); return 1;
    }
    gdn::MapDb world;
    if (!world.Load(map_path)) { std::printf("ERROR: cannot load %s\n", map_path.c_str()); return 1; }
    // Onboard map (R1.3): what the filter predicts measurements from.
    const std::string db_path = db_arg.empty() ? map_path : gdn::DataPath(db_arg);
    gdn::MapDb db;
    if (!db.Load(db_path)) { std::printf("ERROR: cannot load %s\n", db_path.c_str()); return 1; }
    db.SetOrigin(world.lat_c(), world.lon_c());

    const std::vector<double>& it = imu["t"], ax = imu["ax"], ay = imu["ay"], az = imu["az"],
                               gx = imu["gx"], gy = imu["gy"], gz = imu["gz"];
    const std::vector<double>& lt = lpos["t"], lx = lpos["x"], ly = lpos["y"],
                               lvx = lpos["vx"], lvy = lpos["vy"];
    const double t0 = it.front();
    // Truth altitude. The flight was recorded over flat ground, so the synthetic
    // terrain is placed under it: the AMSL datum H is chosen so the vehicle stays
    // at least `clearance` above the map everywhere along the truth path.
    const std::vector<double> zeros(lt.size(), 0.0);
    const std::vector<double>& lz = lpos.has("z") ? lpos["z"] : zeros;
    double H_true = -1e18;
    for (size_t r = 0; r < lt.size(); ++r) {
        const gdn::MapQuery q = world.Query(lx[r], ly[r]);
        if (q.ok) H_true = std::max(H_true, q.h + lz[r] + clearance);
    }

    // Node start-up: heading while parked = heading at first motion (as
    // make_zermatt_bag.py writes the parked attitude).
    double yaw_park = 0.0;
    for (size_t q = 0; q < lt.size(); ++q)
        if (std::hypot(lvx[q], lvy[q]) > 0.5) { yaw_park = std::atan2(lvy[q], lvx[q]); break; }
    const Eigen::Vector3d m_ref(cfg.mag_ref_north, cfg.mag_ref_east, cfg.mag_ref_down);
    const Eigen::Vector3d b_hardiron = hard_iron * Eigen::Vector3d(2.0, -1.0, 1.0);

    // R1.7: scenario. Take-off = first truth speed over 3 m/s (REQ-ACQ-001 counts from it).
    double t_end = std::min((it.back() - t0), (lt.back() - t0)) * 1e-6;
    // A recorded truth can jump (flight001: PX4 reset its local position by about 440 m
    // at 967 s, just before landing). Scoring past a jump would score the reset, so the
    // flight ends there.
    for (size_t q = 1; q < lt.size(); ++q) {
        const double dtq = (lt[q] - lt[q - 1]) * 1e-6;
        const double step = std::hypot(lx[q] - lx[q - 1], ly[q] - ly[q - 1]);
        const double vmax = std::max(std::hypot(lvx[q], lvy[q]), std::hypot(lvx[q - 1], lvy[q - 1]));
        if (step > vmax * dtq + 20.0) {
            t_end = std::min(t_end, (lt[q - 1] - t0) * 1e-6);
            std::printf("truth: position jumps %.0f m at t=%.1f s (a reset in the recorded truth); "
                        "the flight is scored up to there\n", step, t_end);
            break;
        }
    }
    double t_takeoff = 0.0;
    for (size_t q = 0; q < lt.size(); ++q)
        if (std::hypot(lvx[q], lvy[q]) > 3.0) { t_takeoff = (lt[q] - t0) * 1e-6; break; }
    if (gnss_loss >= 0 && gnss_loss + gnss_free.back() > t_end + 1e-6) {
        std::printf("ERROR: GNSS lost at %.0f s plus %.0f s GNSS-free needs a flight of %.0f s; %s ends at %.0f s.\n"
                    "  Longer synthetic flight: tools/gen_zermatt_flight.py --minutes 15 (data/flight_zermatt_long_csv)\n",
                    gnss_loss, gnss_free.back(), gnss_loss + gnss_free.back(), csv_dir.c_str(), t_end);
        return 1;
    }
    constexpr double kGnssRate = 5.0, kGnssPos = 1.5, kGnssVel = 0.1, kAcqLimit = 100.0, kAcqTime = 120.0;
    const double start_sigma = std::max(10.0, start_err);
    if (gnss_loss >= 0)
        std::printf("scenario: GNSS (simulated, %.0f Hz, %.1f m and %.1f m/s per axis) until t=%.0f s, then lost; "
                    "the filter keeps its covariance (handover). Scored from the loss.\n",
                    kGnssRate, kGnssPos, kGnssVel, gnss_loss);
    else if (start_err > 0)
        std::printf("scenario: no GNSS; start error %.0f m in a random direction, filter told sigma %.0f m per axis\n",
                    start_err, start_sigma);
    else
        std::printf("scenario: no GNSS; the filter starts at the true position (sigma 10 m per axis)\n");
    // Per GNSS-free duration: pooled errors, per-seed maxima, consistency.
    std::vector<std::vector<double>> gf_pool(gnss_free.size());
    std::vector<std::vector<double>> gf_seedmax(gnss_free.size());
    std::vector<gdn::ConsistencyStats> gf_cons(gnss_free.size());
    std::vector<double> handover_sigma, handover_err, err_at_acq;
    int acquired = 0, held = 0;

    std::vector<double> pool, vpool, seed_rms, seed_max, latch_t;
    gdn::ConsistencyStats cons_all, cons_pass, cons_fail;   // R1.5
    std::ofstream trace;
    if (!trace_path.empty()) {
        trace.open(trace_path);
        if (!trace) { std::printf("ERROR: cannot write %s\n", trace_path.c_str()); return 1; }
        trace << "seed,t,x,y,tx,ty,err,sigma,nees\n";
    }
    int pass_seeds = 0;

    for (int seed = seed0; seed < seed0 + nseeds; ++seed) {
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

        // Placeholder / file model: the same per-flight turn-on draws, scaled to the
        // model's sigma, so the other random streams are unchanged; noise and drift from
        // a stream of their own.
        gdn::ImuErrorSim imu_sim(imu_model, 0x494d55ULL + static_cast<uint64_t>(seed));
        if (use_imu_model)
            imu_sim.SetTurnOn(ba_off * (imu_model.accel_turn_on_sigma / 0.05),
                              bg_off * (imu_model.gyro_turn_on_sigma / (0.02 * gdn::Eskf::kDeg)));

        gdn::Eskf eskf(cfg);                 // default config == bench config
        double prev_t = -1.0, se = 0.0, smax = 0.0; int sn = 0, last_k = -1;
        bool yaw_done = false; size_t j = 0;
        // Separate stream for baro/AGL noise, so --model=ideal reproduces the
        // original random sequence (and CI baseline) exactly.
        std::mt19937_64 rng2(0x41474cULL + static_cast<uint64_t>(seed));
        std::normal_distribution<double> d_b(0, 1.5), d_a5(0, 5), d_a2(0, 2);
        const double b0 = 5.0;               // constant baro offset, calibrated out on the ground
        double t_trn = -1.0, b_cal = 0.0; int last_k10 = -1;
        auto baro_bias_true = [&](double tt) {
            const double ramp = t_trn < 0 ? 0.0 : std::min(1.0, std::max(0.0, (tt - t_trn) / 300.0));
            return b0 + baro_drift * ramp; };
        // Node start-up: own streams, so the truth path's random sequence is unchanged.
        std::mt19937_64 rng_mag(0x4d4147ULL + static_cast<uint64_t>(seed));
        std::normal_distribution<double> d_mag(0, 2.0);
        gdn::GroundMonitor ground;
        gdn::ConsistencyStats cons;
        bool gnss_airborne = false; int gnss_zc = 0, last_m = -1;
        // R1.7: own streams, so the existing random sequences are unchanged.
        std::mt19937_64 rng_gnss(0x474e53ULL + static_cast<uint64_t>(seed));
        std::mt19937_64 rng_start(0x535452ULL + static_cast<uint64_t>(seed));
        std::normal_distribution<double> d_g(0, 1);
        const double start_dir = std::uniform_real_distribution<double>(0.0, 2.0 * M_PI)(rng_start);
        bool start_applied = false, handed_over = false;
        int last_g = -1;
        double e_acq = -1.0, max_after_acq = 0.0;
        std::vector<double> gf_max(gnss_free.size(), 0.0);
        std::vector<gdn::ConsistencyStats> gf_c(gnss_free.size());

        for (size_t i = 0; i < it.size(); ++i) {
            const double t = (it[i] - t0) * 1e-6;
            if (t > t_end) break;                                 // R1.7: truth reset (above)
            Eigen::Vector3d a_m(ax[i], ay[i], az[i]), w_m(gx[i], gy[i], gz[i]);
            if (use_imu_model) {
                const size_t i0 = i > 0 ? i - 1 : 0, i1 = i > 0 ? i : 1;
                imu_sim.Corrupt(&a_m, &w_m, (it[i1] - it[i0]) * 1e-6);
            } else {
                a_m += ba_off; w_m += bg_off;
            }
            if (!eskf.aligned()) {
            if (eskf.FeedStatic(a_m, w_m)) {
                ground.Start(eskf.RestSpecificForce(), eskf.gyro_bias());
                if (diag_mode) {
                const Eigen::Vector3d r0 = eskf.q().toRotationMatrix().eulerAngles(2,1,0)/gdn::Eskf::kDeg;
                std::printf("ALIGN t=%.1f ba=%.4f bg=%.4f yaw=%.1f pit=%.1f rol=%.1f\n", t,
                    eskf.accel_bias().norm(), eskf.gyro_bias().norm()/gdn::Eskf::kDeg,
                    r0.x(), r0.y(), r0.z());
                }
            }
            prev_t = t; continue;
        }
            const double dt = t - prev_t; prev_t = t;
            if (dt <= 0) continue;
            eskf.Propagate(a_m, w_m, dt);

            while (j + 1 < lt.size() && (lt[j + 1] - t0) * 1e-6 <= t) ++j;
            const double spd = std::hypot(lvx[j], lvy[j]);
            if (node_startup) {
                // Ground detection and ZUPT, as eskf_node.
                if (ground_mode == "imu") {
                    const bool was = ground.airborne();
                    if (ground.Update(a_m, w_m, dt)) eskf.ApplyZupt();
                    if (ground.airborne() && !was) {
                        latch_t.push_back(t);
                        if (diag_mode) std::printf("LATCH t=%.2f (IMU) spd=%.2f\n", t, spd);
                    }
                } else if (!gnss_airborne) {                 // pre-R1.2 node rule
                    if (spd > 2.0) { gnss_airborne = true; latch_t.push_back(t); }
                    else if (++gnss_zc % 10 == 0) eskf.ApplyZupt();
                }
                // Magnetometer at 10 Hz (mag_sim.py), level flight along the velocity.
                const int m = static_cast<int>(std::floor(t * 10.0));
                if (m != last_m) {
                    last_m = m;
                    const double yaw_t = spd > 0.5 ? std::atan2(lvy[j], lvx[j]) : yaw_park;
                    const Eigen::Matrix3d Rt =
                        Eigen::AngleAxisd(yaw_t, Eigen::Vector3d::UnitZ()).toRotationMatrix();
                    const Eigen::Vector3d zm = Rt.transpose() * m_ref + b_hardiron +
                        Eigen::Vector3d(d_mag(rng_mag), d_mag(rng_mag), d_mag(rng_mag));
                    if (!eskf.yaw_aligned()) {
                        eskf.AlignYawFromMag(zm);
                        if (diag_mode) std::printf("YAW t=%.1f mag align\n", t);
                    } else if (mag_mode == "heading") {
                        eskf.ApplyMagHeading(zm);
                    } else if (mag_mode == "auto") {
                        eskf.ApplyMagAuto(zm);
                    } else if (mag_mode == "full") {
                        eskf.ApplyMag(zm);
                    }
                }
                yaw_done = eskf.yaw_aligned();
            } else if (!yaw_done && spd > 3.0 &&
                std::abs(w_m.z() - eskf.gyro_bias().z()) < 0.05) {
                eskf.TransferAlignYaw(std::atan2(lvy[j], lvx[j]));
                yaw_done = true;
                if (diag_mode) std::printf("YAW t=%.1f yaw=%.1f spd=%.1f pb=(%.1f,%.1f)\n", t,
                    std::atan2(lvy[j], lvx[j])/gdn::Eskf::kDeg, spd, eskf.pos().x(), eskf.pos().y());
            }

            // R1.7 start error: once heading is aligned, move the filter's position and
            // tell it how unsure it is.
            if (yaw_done && !start_applied) {
                start_applied = true;
                if (start_err > 0)
                    eskf.SetHorizontalPosition(eskf.pos().head<2>() + start_err *
                        Eigen::Vector2d(std::cos(start_dir), std::sin(start_dir)), start_sigma);
            }
            // R1.7 GNSS until the loss, at 5 Hz, from the truth plus white noise.
            if (gnss_loss >= 0 && t < gnss_loss && yaw_done) {
                const int g = static_cast<int>(std::floor(t * kGnssRate));
                if (g != last_g) {
                    last_g = g;
                    eskf.ApplyGnss(Eigen::Vector2d(lx[j] + kGnssPos * d_g(rng_gnss), ly[j] + kGnssPos * d_g(rng_gnss)),
                                   Eigen::Vector2d(lvx[j] + kGnssVel * d_g(rng_gnss), lvy[j] + kGnssVel * d_g(rng_gnss)),
                                   kGnssPos, kGnssVel);
                }
            }

            // Barometer at 10 Hz (agl model only; ideal and naive have no baro state).
            const int k10 = static_cast<int>(std::floor(t * 10.0));
            if (model == "agl" && eskf.datum_set() && k10 != last_k10)
                eskf.ApplyBaro(lz[j] + baro_bias_true(t) + d_b(rng2));
            last_k10 = k10;

            const int k = static_cast<int>(std::floor(t * 2.0));   // 0.5 s grid
            if (k == last_k || !yaw_done) { last_k = k; continue; }
            last_k = k;
            const gdn::MapQuery wq = world.Query(lx[j], ly[j]);
            if (wq.ok) {
                const double meas_ideal = wq.h + (diag_mode ? 0.0 : (d_n5(rng) - d_n2(rng)));
                const gdn::MapQuery mq = db.Query(eskf.pos().x(), eskf.pos().y());
                double innov = 0, S = 0;
                const int kc = eskf.updates() + eskf.rejects();   // bench-equivalent k
                gdn::Eskf::UpdResult res;
                double meas = meas_ideal;
                if (model == "ideal") {
                    res = eskf.ApplyTrn(meas, mq, &innov, &S);
                } else {
                    // Measured height above ground (rangefinder / radar altimeter).
                    const double agl = (H_true - lz[j]) - wq.h + d_a5(rng2) - d_a2(rng2);
                    const double baro = lz[j] + baro_bias_true(t) + d_b(rng2);
                    if (t_trn < 0) {
                        // First fix: calibrate on the spot, as a real start-up would.
                        t_trn = t;
                        b_cal = baro;   // naive: baro offset at start
                        // Datum known to about the AGL noise (5.4 m here), baro to 1.5 m.
                        eskf.SetAltitudeDatum(mq.h + agl + eskf.pos().z(), 5.4);
                        eskf.InitBaroBias(baro, 1.5);
                    }
                    if (model == "naive") {
                        // terrain = (datum - (baro - offset)) - agl, no bias state
                        meas = (eskf.altitude_datum() - (baro - b_cal)) - agl;
                        res = eskf.ApplyTrn(meas, mq, &innov, &S);
                    } else {
                        meas = agl;
                        res = eskf.ApplyAgl(agl, mq, &innov, &S);
                    }
                }
            if (res != gdn::Eskf::UpdResult::kSlopeGate && t >= w0 && t <= w1)
                cons.AddInnovation(innov, S);   // NIS, before the gate
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
            if (trace.is_open()) {
                const Eigen::Vector2d ev(eskf.pos().x() - lx[j], eskf.pos().y() - ly[j]);
                const Eigen::Matrix2d Ph = eskf.P_horiz();
                const double nees = Ph.determinant() > 0 ? ev.dot(Ph.ldlt().solve(ev)) : 1e12;
                char row[256];
                std::snprintf(row, sizeof row, "%d,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.4g\n", seed, t,
                              eskf.pos().x(), eskf.pos().y(), lx[j], ly[j], e, eskf.sigma_pos(), nees);
                trace << row;
            }
            if (diag_mode && t >= 25.0 && std::fmod(t, 1.0) < 0.5)
                std::printf("t=%3.0f err=%7.1f pos=(%7.1f,%7.1f) truth=(%7.1f,%7.1f) sp=%5.1f\n",
                            t, e, eskf.pos().x(), eskf.pos().y(), lx[j], ly[j],
                            eskf.sigma_pos());
            // R1.7: acquisition (error 120 s after take-off, and the worst after that),
            // handover, and the GNSS-free windows.
            if (t >= t_takeoff + kAcqTime) {
                if (e_acq < 0) e_acq = e;
                max_after_acq = std::max(max_after_acq, e);
            }
            if (gnss_loss >= 0 && t >= gnss_loss) {
                if (!handed_over) {
                    handed_over = true;
                    handover_sigma.push_back(eskf.sigma_pos()); handover_err.push_back(e);
                }
                for (size_t d = 0; d < gnss_free.size(); ++d)
                    if (t <= gnss_loss + gnss_free[d]) {
                        gf_pool[d].push_back(e); gf_max[d] = std::max(gf_max[d], e);
                        gf_c[d].AddPosition(Eigen::Vector2d(eskf.pos().x() - lx[j], eskf.pos().y() - ly[j]),
                                            eskf.P_horiz(), eskf.sigma_pos());
                    }
            }
            if (t >= w0 && t <= w1) {
                cons.AddPosition(Eigen::Vector2d(eskf.pos().x() - lx[j], eskf.pos().y() - ly[j]),
                                 eskf.P_horiz(), eskf.sigma_pos());
                pool.push_back(e); se += e * e; sn++; if (e > smax) smax = e;
                if (eskf.datum_set())
                    vpool.push_back(std::abs(eskf.altitude_amsl() - (H_true - lz[j])));
            }
        }
        const double rms = sn > 0 ? std::sqrt(se / sn) : 1e9;
        seed_rms.push_back(rms); seed_max.push_back(smax);
        cons_all.Merge(cons);
        (rms < 100.0 ? cons_pass : cons_fail).Merge(cons);
        if (rms < 100.0) ++pass_seeds;
        for (size_t d = 0; d < gnss_free.size(); ++d) { gf_seedmax[d].push_back(gf_max[d]); gf_cons[d].Merge(gf_c[d]); }
        // REQ-ACQ-001: under the limit 120 s after take-off; held: stayed under to the end.
        err_at_acq.push_back(e_acq);
        if (e_acq >= 0 && e_acq < kAcqLimit) ++acquired;
        if (e_acq >= 0 && max_after_acq < kAcqLimit) ++held;
    }

    int in_spec = 0; for (double e : pool) if (e < 100.0) ++in_spec;
    {
        const std::string wn = map_path.substr(map_path.find_last_of('/') + 1);
        const std::string dn = db_path.substr(db_path.find_last_of('/') + 1);
        std::printf("maps: world %s, onboard %s%s\n", wn.c_str(), dn.c_str(),
                    db_path == map_path ? " (same map: no map error)" : "");
    }
    std::printf("MC campaign: %d seeds, model %s, startup %s%s, baro drift %.1f m, window %.0f-%.0f s, pooled n=%zu\n",
                nseeds, model.c_str(), startup.c_str(),
                node_startup ? ((ground_mode == "imu" ? std::string(" (IMU ground, mag ") : std::string(" (GNSS-speed ground, mag "))
                                + mag_mode + ")").c_str() : "",
                baro_drift, w0, w1, pool.size());
    if (node_startup && !latch_t.empty())
        std::printf("airborne latch: %zu/%d seeds, t median %.2f s\n", latch_t.size(), nseeds,
                    Percentile(latch_t, 50));
    std::printf("CEP50: %.1f m  CEP95: %.1f m  MAX: %.1f m  in-spec(<100 m): %.1f%%\n", Percentile(pool, 50),
                Percentile(pool, 95), Percentile(pool, 100),
                pool.empty() ? 0.0 : 100.0 * in_spec / static_cast<double>(pool.size()));
    std::printf("per-seed RMS: median %.1f m  p95 %.1f m  worst %.1f m  passing(<100 m): %d/%d\n",
                Percentile(seed_rms, 50), Percentile(seed_rms, 95),
                Percentile(seed_rms, 100), pass_seeds, nseeds);
    // R1.5 consistency (gdn/consistency.hpp). ci_check_mc.py parses these lines.
    std::printf("consistency: NEES mean %.2f median %.2f (consistent: 2, 1.39), outside 3-sigma ellipse %.1f%% (1.1%%)\n",
                cons_all.nees_mean(), cons_all.nees_median(), cons_all.nees_over9_pct());
    std::printf("consistency: error > 3 x published sigma %.2f%% (REQ-INT-001: at most 1%%)\n",
                cons_all.over3sigma_pct());
    std::printf("consistency: NIS mean %.2f median %.2f (consistent: 1, 0.45), NIS > 9 %.1f%% (0.27%%), n=%zu\n",
                cons_all.nis_mean(), cons_all.nis_median(), cons_all.nis_over9_pct(), cons_all.innovations());
    std::printf("consistency by seed: passing %d: over 3 sigma %.2f%%, NEES median %.2f | failing %d: over 3 sigma %.2f%%, NEES median %.2f\n",
                pass_seeds, cons_pass.over3sigma_pct(), cons_pass.nees_median(),
                nseeds - pass_seeds, cons_fail.over3sigma_pct(), cons_fail.nees_median());
    if (!vpool.empty())
        std::printf("vertical: altitude error p50 %.1f m  p95 %.1f m  max %.1f m\n",
                    Percentile(vpool, 50), Percentile(vpool, 95), Percentile(vpool, 100));
    // R1.7. ci_check_mc.py parses the "GNSS-free" and "acquisition" lines.
    if (gnss_loss >= 0) {
        std::printf("handover at t=%.0f s: filter sigma median %.2f m, error median %.2f m (p95 %.2f m)\n",
                    gnss_loss, Percentile(handover_sigma, 50), Percentile(handover_err, 50),
                    Percentile(handover_err, 95));
        for (size_t d = 0; d < gnss_free.size(); ++d) {
            int under = 0; for (double m : gf_seedmax[d]) if (m < 100.0) ++under;
            std::printf("GNSS-free %.0f s: CEP50 %.1f m  CEP95 %.1f m  max %.1f m  over 3 sigma %.2f%%  "
                        "flights never over 100 m: %d/%d\n",
                        gnss_free[d], Percentile(gf_pool[d], 50), Percentile(gf_pool[d], 95),
                        Percentile(gf_pool[d], 100), gf_cons[d].over3sigma_pct(), under, nseeds);
        }
    }
    if (start_set || start_err > 0)
        std::printf("acquisition (REQ-ACQ-001): start error %.0f m: under %.0f m at %.0f s after take-off: %d/%d flights "
                    "(error there median %.1f m); stayed under to the end (%.0f s): %d/%d\n",
                    start_err, kAcqLimit, kAcqTime, acquired, nseeds, Percentile(err_at_acq, 50), t_end, held, nseeds);
    { std::ofstream o("/tmp/mc_per_seed.csv"); o << "seed,rms,max\n";
      for (size_t s = 0; s < seed_rms.size(); ++s)
          o << s << "," << seed_rms[s] << "," << seed_max[s] << "\n"; }
    return 0;
}
