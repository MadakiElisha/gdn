#pragma once
// IMU error model for simulation (R1.6). ROS-free.
//
// The terms an Allan-variance fit of a static log gives (IEEE Std 952), per sensor:
//   white noise     N, per axis: the angle / velocity random walk density
//   bias drift      first-order Gauss-Markov process: driving density K, correlation
//                   time tau (stationary sigma K * sqrt(tau / 2))
//   turn-on bias    a constant drawn once per power-up, 1 sigma
// The drift is advanced with the exact discrete covariance (Maybeck 4-114), the same
// recursion as PX4's Gazebo IMU plugin, so a model file means the same thing here and
// in Gazebo (R2.4).
//
// Model files live in config/imu/ (key: value, '#' comments, vectors as [x, y, z]).
// Until the H30WP, Mid-360 and OAK-D IMUs have been logged and fitted
// (tools/allan_fit.py writes the file), the Monte Carlo uses the generic MEMS
// placeholder ImuPlaceholder() = config/imu/placeholder_px4_default.yaml, and every
// result carries its name.
#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <random>
#include <set>
#include <sstream>
#include <string>

namespace gdn {

struct ImuNoiseModel {
    std::string name;     // printed with every result that uses the model
    std::string source;   // where the numbers come from
    Eigen::Vector3d accel_noise_density = Eigen::Vector3d::Zero();  // m/s^2/sqrt(Hz)
    Eigen::Vector3d gyro_noise_density = Eigen::Vector3d::Zero();   // rad/s/sqrt(Hz)
    double accel_bias_rw = 0.0;         // m/s^3/sqrt(Hz), Gauss-Markov driving density
    double accel_bias_tau = 300.0;      // s
    double gyro_bias_rw = 0.0;          // rad/s^2/sqrt(Hz)
    double gyro_bias_tau = 1000.0;      // s
    double accel_turn_on_sigma = 0.0;   // m/s^2
    double gyro_turn_on_sigma = 0.0;    // rad/s

    double accel_drift_sigma() const { return accel_bias_rw * std::sqrt(accel_bias_tau / 2.0); }
    double gyro_drift_sigma() const { return gyro_bias_rw * std::sqrt(gyro_bias_tau / 2.0); }

    // Allan deviation of one axis at averaging time t (s): white noise N plus a
    // first-order Gauss-Markov drift (K, tau). Exact for that process.
    static double Adev(double N, double K, double tau, double t) {
        double gm = 0.0;
        if (K > 0.0 && tau > 0.0) {
            const double x = t / tau;
            gm = x < 1e-3 ? K * K * t / 3.0   // small-t limit, avoids cancellation
                          : K * K * tau * tau / t *
                                (1.0 - (3.0 - 4.0 * std::exp(-x) + std::exp(-2.0 * x)) / (2.0 * x));
        }
        return std::sqrt(N * N / t + gm);
    }
};

// Generic MEMS placeholder (R1.6): PX4's default simulated IMU, i.e. the x500 IMU of
// PX4 v1.16 Gazebo (Tools/simulation/gz/models/x500_base/model.sdf, "noise modeled
// after IIM42653"): white noise only, 0.000873 rad/s and 0.00637 / 0.00686 m/s^2 per
// sample at 250 Hz, converted to densities here. PX4 removed its bias drift in Feb 2025
// (gz models 1a73e1f), so the model has none. The turn-on bias is not PX4's: it is the
// per-flight residual the GDN Monte Carlo has always drawn (0.05 m/s^2, 0.02 deg/s).
inline ImuNoiseModel ImuPlaceholder() {
    ImuNoiseModel m;
    m.name = "placeholder-px4-default";
    m.source = "generic MEMS placeholder: PX4 v1.16 Gazebo x500 IMU noise (IIM-42653, white only); "
               "turn-on bias from the GDN MC";
    const double s250 = std::sqrt(250.0);
    m.accel_noise_density = Eigen::Vector3d(0.00637, 0.00637, 0.00686) / s250;
    m.gyro_noise_density = Eigen::Vector3d::Constant(0.0008726646 / s250);
    m.accel_turn_on_sigma = 0.05;
    m.gyro_turn_on_sigma = 0.02 * M_PI / 180.0;
    return m;
}

// Reads a model file. Every model key is required (a typo must not silently become a
// zero); "source" and keys starting with "fit_" are optional / informational.
inline bool LoadImuModel(const std::string& path, ImuNoiseModel* out, std::string* err) {
    std::ifstream f(path);
    if (!f) { *err = "cannot open " + path; return false; }
    ImuNoiseModel m;
    std::set<std::string> seen;
    const std::set<std::string> required = {
        "name", "accel_noise_density", "gyro_noise_density", "accel_bias_rw", "accel_bias_tau",
        "gyro_bias_rw", "gyro_bias_tau", "accel_turn_on_sigma", "gyro_turn_on_sigma"};
    auto trim = [](std::string s) {
        const auto a = s.find_first_not_of(" \t\r\"'"), b = s.find_last_not_of(" \t\r\"'");
        return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
    };
    auto vec = [&](const std::string& v, Eigen::Vector3d* x) {
        std::string s = v;
        for (char& c : s) if (c == '[' || c == ']' || c == ',') c = ' ';
        std::istringstream is(s);
        double a, b, c;
        if (is >> a >> b >> c) { *x = Eigen::Vector3d(a, b, c); return true; }
        std::istringstream is1(s);
        if (is1 >> a) { *x = Eigen::Vector3d::Constant(a); return true; }
        return false;
    };
    std::string line;
    int ln = 0;
    while (std::getline(f, line)) {
        ++ln;
        if (const auto h = line.find('#'); h != std::string::npos) line.erase(h);
        if (trim(line).empty()) continue;
        const auto c = line.find(':');
        if (c == std::string::npos) { *err = path + ":" + std::to_string(ln) + ": expected key: value"; return false; }
        const std::string k = trim(line.substr(0, c)), v = trim(line.substr(c + 1));
        bool ok = true;
        char* end = nullptr;
        auto num = [&](double* x) { *x = std::strtod(v.c_str(), &end); ok = end != v.c_str() && *end == '\0'; };
        if (k == "name") m.name = v;
        else if (k == "source") m.source = v;
        else if (k == "accel_noise_density") ok = vec(v, &m.accel_noise_density);
        else if (k == "gyro_noise_density") ok = vec(v, &m.gyro_noise_density);
        else if (k == "accel_bias_rw") num(&m.accel_bias_rw);
        else if (k == "accel_bias_tau") num(&m.accel_bias_tau);
        else if (k == "gyro_bias_rw") num(&m.gyro_bias_rw);
        else if (k == "gyro_bias_tau") num(&m.gyro_bias_tau);
        else if (k == "accel_turn_on_sigma") num(&m.accel_turn_on_sigma);
        else if (k == "gyro_turn_on_sigma") num(&m.gyro_turn_on_sigma);
        else if (k.rfind("fit_", 0) == 0) continue;
        else { *err = path + ":" + std::to_string(ln) + ": unknown key '" + k + "'"; return false; }
        if (!ok) { *err = path + ":" + std::to_string(ln) + ": bad value for " + k; return false; }
        seen.insert(k);
    }
    for (const auto& k : required)
        if (!seen.count(k)) { *err = path + ": missing " + k; return false; }
    if (m.accel_bias_tau <= 0 || m.gyro_bias_tau <= 0) { *err = path + ": tau must be > 0"; return false; }
    *out = m;
    return true;
}

// Corrupts error-free IMU samples with one model, one power-up.
class ImuErrorSim {
public:
    using V3 = Eigen::Vector3d;
    ImuErrorSim(ImuNoiseModel m, uint64_t seed) : m_(std::move(m)), rng_(seed) {
        ba0_ = m_.accel_turn_on_sigma * V3(n_(rng_), n_(rng_), n_(rng_));
        bg0_ = m_.gyro_turn_on_sigma * V3(n_(rng_), n_(rng_), n_(rng_));
    }
    // Override the turn-on draw (the MC keeps its historical per-flight draws).
    void SetTurnOn(const V3& ba, const V3& bg) { ba0_ = ba; bg0_ = bg; }

    // dt: the sample interval the reading stands for (sets the white-noise sigma).
    // The drift advances over dt first, then white noise of sigma N / sqrt(dt) is added.
    void Corrupt(V3* a, V3* w, double dt) {
        if (dt <= 0.0) return;
        Drift(&da_, m_.accel_bias_rw, m_.accel_bias_tau, dt);
        Drift(&dg_, m_.gyro_bias_rw, m_.gyro_bias_tau, dt);
        const double s = 1.0 / std::sqrt(dt);
        for (int i = 0; i < 3; ++i) {
            (*a)[i] += ba0_[i] + da_[i] + m_.accel_noise_density[i] * s * n_(rng_);
            (*w)[i] += bg0_[i] + dg_[i] + m_.gyro_noise_density[i] * s * n_(rng_);
        }
    }
    V3 accel_bias() const { return ba0_ + da_; }
    V3 gyro_bias() const { return bg0_ + dg_; }
    const ImuNoiseModel& model() const { return m_; }

private:
    // Exact discretisation of the Gauss-Markov drift over dt (Maybeck 4-114).
    void Drift(V3* d, double K, double tau, double dt) {
        if (K <= 0.0) return;
        const double phi = std::exp(-dt / tau);
        const double sd = std::sqrt(-K * K * tau / 2.0 * std::expm1(-2.0 * dt / tau));
        for (int i = 0; i < 3; ++i) (*d)[i] = phi * (*d)[i] + sd * n_(rng_);
    }

    ImuNoiseModel m_;
    std::mt19937_64 rng_;
    std::normal_distribution<double> n_{0.0, 1.0};
    V3 ba0_ = V3::Zero(), bg0_ = V3::Zero(), da_ = V3::Zero(), dg_ = V3::Zero();
};

}  // namespace gdn
