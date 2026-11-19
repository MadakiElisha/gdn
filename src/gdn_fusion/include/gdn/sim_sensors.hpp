#pragma once
// Simulated sensors (roadmap R2.4a), ROS-free: what tools/sim_sensors does between
// Gazebo's error-free sensors and the topics the real drivers publish. Unit-checked by
// gdn_selftest (TEST-035).
//
//   Resampler    error-free IMU samples at the physics rate onto the sensor's own clock:
//                an even rate (H30WP, Mid-360S), or a tick clock where each step skips a
//                tick with some probability (the OAK-D Lite's BMI270). Linear
//                interpolation between the raw samples.
//   TimingModel  a sample time to its message: the stamp (the sample time plus stamp
//                jitter, or the arrival time for a driver that stamps on arrival), the
//                arrival time (plus latency and its jitter, never before the previous
//                message of the same sensor, nor before the simulation can deliver it),
//                and whether it's lost.
//   RangeReading the TFmini-S: +inf beyond max range, -inf under min range, else the
//                true range plus range-dependent noise, rounded to the resolution.
//   SimClock     Gazebo's clock for the stamps and the latencies (R2.4b; R2.4a followed
//                the wall clock, which stretched every interval by 1 / real-time factor
//                when the simulation ran slower than real time: an IMU's 5 ms became
//                5.5 ms at 0.92). Stamps are Gazebo time plus one offset, fixed by the
//                first raw message, so they start near wall time and keep Gazebo's
//                seconds; a message is published once Gazebo's time passes its arrival.
//                All sensors share one clock.
//
// R2.4b, the OAK-D Lite's cameras:
//   PinholeFromHfov  the intrinsics of Gazebo's distortion-free camera: square pixels,
//                    the principal point at the image centre (pixel centres at integers,
//                    so (w - 1) / 2), the focal length from the horizontal field of view.
//   ImageNoise       read noise on an 8-bit image: each pixel plus a normal draw of sigma
//                    (in grey levels), rounded and clipped to 0..255. Draws come from a
//                    table of 2^20, read at a random start and odd stride per image, so a
//                    640 by 480 image never repeats a pattern and costs one add per pixel.
//   AutoExposure     the gain that brings an image's mean linear brightness to a target
//                    (0.18, photographic mid-grey), approached in log gain with a time
//                    constant and kept within the sensor's range, like a camera's AE.
//   ToneMap          16-bit linear light (what Gazebo's grey cameras render: no gamma) times
//                    the gain, clipped at full scale, sRGB-encoded to 8 bits, as a camera's
//                    processing does before it sends the image.
//
// R2.4c, the Mid-360S lidar:
//   LivoxPattern     Livox's measured Mid-360 scan pattern (Livox-SDK/livox_laser_simulation
//                    scan_mode/mid360.csv: 800,000 directions in firing order, four laser
//                    lines in turn, 4 s at 200,000 points/s), as azimuth and elevation in
//                    the sensor frame. The file counts azimuth clockwise; the real unit
//                    sweeps counter-clockwise (TEST-037), so it's mirrored.
//   LidarGrid        Gazebo's gpu_lidar rays: nh by nv, even in azimuth and elevation, row
//                    by row from the lowest elevation; the nearest ray to a direction, and
//                    the four around it with bilinear weights.
//   GridRange        the range along a pattern direction between rendered rays: bilinear
//                    over the four around it where all hit and agree within 5% (a smooth
//                    surface), else the nearest ray's (an edge: no flying points). Points
//                    keep the pattern's own direction, not the grid's (R2.4c: snapped to
//                    0.5 deg rays, Livox's curved bands came out as flat stripes).
//   LivoxTiming      when each point fires: packets of 96 points 4.947 us apart, 208 packets
//                    per 100 ms frame, as the real stream (TEST-037).
//   InterpRange      a ray's range at a point's own time, interpolated between the renders
//                    before and after it, so the cloud's motion matches its offset_time;
//                    past the latest render, extrapolated from the last two (up to one
//                    render period), so a frame needn't wait for the next render.
//   LivoxReturn      a point from a true range: none (published as 0, 0, 0 with
//                    reflectivity 0, as the real driver does) beyond max_range or nearer
//                    than min_range, else along the ray with range and angle noise.
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace gdn {
namespace simsens {

using V3 = Eigen::Vector3d;

class Resampler {
public:
    // rate_hz > 0: even clock. tick_hz > 0: tick clock with skip_prob of skipping one tick.
    Resampler(double rate_hz, double tick_hz, double skip_prob, uint64_t seed)
        : rate_(rate_hz), tick_(tick_hz), skip_(skip_prob), rng_(seed) {}

    // Adds a raw sample at time t; calls deliver(ts, a, w) for every sensor sample in
    // (previous raw time, t], in order.
    template <class F>
    void Add(double t, const V3& a, const V3& w, F deliver) {
        if (!have_) {
            have_ = true;
            t0_ = t;
            a0_ = a;
            w0_ = w;
            start_ = t;
            return;
        }
        if (t <= t0_) return;
        // Sample times from an integer count of periods (or ticks), not a running sum, so
        // rounding doesn't build up over a flight.
        while (true) {
            const double next = Next();
            if (next > t) break;
            const double f = (next - t0_) / (t - t0_);
            deliver(next, V3(a0_ + f * (a - a0_)), V3(w0_ + f * (w - w0_)));
            n_ += tick_ > 0.0 && u_(rng_) < skip_ ? 2 : 1;
        }
        t0_ = t;
        a0_ = a;
        w0_ = w;
    }
    double Next() const { return start_ + static_cast<double>(n_) / (tick_ > 0.0 ? tick_ : rate_); }
    // The nominal spacing between samples (for the first sample's noise interval).
    double NominalDt() const { return tick_ > 0.0 ? (1.0 + skip_) / tick_ : 1.0 / rate_; }

private:
    double rate_, tick_, skip_;
    std::mt19937_64 rng_;
    std::uniform_real_distribution<double> u_{0.0, 1.0};
    bool have_ = false;
    double t0_ = 0.0, start_ = 0.0;
    int64_t n_ = 0;  // periods (or ticks) from start_ to the next sample
    V3 a0_ = V3::Zero(), w0_ = V3::Zero();
};

struct Timing {
    bool arrival_stamp = false;   // the driver stamps on arrival (no sample time from the sensor)
    double latency = 0.0;         // s, sample to arrival
    double latency_jitter = 0.0;  // s, sigma (normal, cut at zero latency)
    double stamp_jitter = 0.0;    // s, sigma of a sensor stamp's error
    double drop_prob = 0.0;       // probability a message is lost
};

struct Message {
    double stamp = 0.0;    // header stamp
    double arrival = 0.0;  // when it's published
    bool lost = false;
};

class TimingModel {
public:
    TimingModel(Timing t, uint64_t seed) : t_(t), rng_(seed) {}
    // not_before: the simulation can't deliver it earlier (an interpolated IMU sample
    // needs the next physics step; R2.4b).
    Message Apply(double sample_t, double not_before = -std::numeric_limits<double>::infinity()) {
        Message m;
        m.lost = u_(rng_) < t_.drop_prob;
        double lat = t_.latency + t_.latency_jitter * n_(rng_);
        if (lat < 0.0) lat = 0.0;
        m.arrival = std::max({sample_t + lat, not_before, last_arrival_});  // a driver delivers in order
        last_arrival_ = m.arrival;
        m.stamp = t_.arrival_stamp ? m.arrival : sample_t + t_.stamp_jitter * n_(rng_);
        return m;
    }

private:
    Timing t_;
    std::mt19937_64 rng_;
    std::normal_distribution<double> n_{0.0, 1.0};
    std::uniform_real_distribution<double> u_{0.0, 1.0};
    double last_arrival_ = -std::numeric_limits<double>::infinity();
};

struct RangeSpec {
    double min_range = 0.1, max_range = 12.0, resolution = 0.01;
    double noise_near = 0.03, near_limit = 6.0, noise_far_frac = 0.005;
};

inline double RangeSigma(const RangeSpec& s, double r) {
    return r < s.near_limit ? s.noise_near : s.noise_far_frac * r;
}

template <class Rng>
double RangeReading(const RangeSpec& s, double r_true, Rng& rng) {
    if (!std::isfinite(r_true) || r_true > s.max_range) return std::numeric_limits<double>::infinity();
    if (r_true < s.min_range) return -std::numeric_limits<double>::infinity();
    std::normal_distribution<double> n(0.0, RangeSigma(s, r_true));
    double r = r_true + n(rng);
    if (s.resolution > 0.0) r = std::round(r / s.resolution) * s.resolution;
    if (r > s.max_range) return std::numeric_limits<double>::infinity();
    if (r < s.min_range) return -std::numeric_limits<double>::infinity();
    return r;
}

class SimClock {
public:
    explicit SimClock(double max_step_s = 0.004) : max_step_(max_step_s) {}
    // A raw message stamped sim_t (Gazebo time) arrived at wall time wall_t. The first
    // one fixes the offset; Gazebo's time is the latest seen.
    void Observe(double sim_t, double wall_t) {
        if (!init_) {
            offset_ = wall_t - sim_t;
            init_ = true;
        }
        if (sim_t > last_sim_) {
            last_sim_ = sim_t;
            last_wall_ = wall_t;
        }
    }
    bool Ready() const { return init_; }
    double Offset() const { return offset_; }
    // Gazebo's time now: the latest seen, plus the wall time since, up to one physics step.
    double Now(double wall_t) const { return last_sim_ + std::clamp(wall_t - last_wall_, 0.0, max_step_); }
    // A time on Gazebo's clock as a stamp: plus the fixed offset, so one second of
    // simulation is one second of stamps.
    double ToStamp(double sim_t) const { return sim_t + offset_; }

private:
    double max_step_;
    bool init_ = false;
    double offset_ = 0.0, last_sim_ = -std::numeric_limits<double>::infinity(), last_wall_ = 0.0;
};

struct Pinhole {
    double fx = 0.0, fy = 0.0, cx = 0.0, cy = 0.0;
};

inline Pinhole PinholeFromHfov(int width, int height, double hfov_rad) {
    Pinhole p;
    p.fx = 0.5 * width / std::tan(0.5 * hfov_rad);
    p.fy = p.fx;
    p.cx = 0.5 * (width - 1);
    p.cy = 0.5 * (height - 1);
    return p;
}

class ImageNoise {
public:
    static constexpr size_t kTable = size_t{1} << 20;
    ImageNoise(double sigma, uint64_t seed) : table_(kTable), rng_(seed) {
        std::normal_distribution<double> n(0.0, sigma);
        for (auto& v : table_) v = static_cast<int16_t>(std::lround(std::clamp(n(rng_), -255.0, 255.0)));
    }
    void Apply(uint8_t* px, size_t n) {
        if (n == 0) return;
        const size_t mask = kTable - 1;
        size_t k = static_cast<size_t>(rng_()) & mask;
        const size_t stride = (static_cast<size_t>(rng_()) & mask) | 1U;
        for (size_t i = 0; i < n; ++i) {
            const int v = px[i] + table_[k];
            px[i] = static_cast<uint8_t>(std::clamp(v, 0, 255));
            k = (k + stride) & mask;
        }
    }

private:
    std::vector<int16_t> table_;
    std::mt19937_64 rng_;
};

class AutoExposure {
public:
    AutoExposure(double target, double tau_s, double gain_min, double gain_max)
        : target_(target), tau_(tau_s), lo_(gain_min), hi_(gain_max) {}
    // mean_linear: the image's mean brightness, 0..1 of full scale, before the gain; dt:
    // seconds since the previous image. Returns the gain for this image.
    double Update(double mean_linear, double dt) {
        const double want = std::clamp(target_ / std::max(mean_linear, 1e-6), lo_, hi_);
        if (gain_ <= 0.0)
            gain_ = want;  // already settled when the first image comes
        else
            gain_ *= std::pow(want / gain_, -std::expm1(-std::max(dt, 0.0) / tau_));
        return gain_;
    }
    double Gain() const { return gain_; }

private:
    double target_, tau_, lo_, hi_, gain_ = 0.0;
};

inline double SrgbEncode(double x) {
    x = std::clamp(x, 0.0, 1.0);
    return x <= 0.0031308 ? 12.92 * x : 1.055 * std::pow(x, 1.0 / 2.4) - 0.055;
}

// 16-bit linear to 8-bit sRGB at a gain, through a table built once per gain.
class ToneMap {
public:
    ToneMap() : lut_(65536) {}
    void SetGain(double gain) {
        if (gain == gain_) return;
        gain_ = gain;
        for (size_t v = 0; v < lut_.size(); ++v)
            lut_[v] = static_cast<uint8_t>(std::lround(255.0 * SrgbEncode(gain * static_cast<double>(v) / 65535.0)));
    }
    void Apply(const uint16_t* in, uint8_t* out, size_t n) const {
        for (size_t i = 0; i < n; ++i) out[i] = lut_[in[i]];
    }

private:
    std::vector<uint8_t> lut_;
    double gain_ = -1.0;
};

struct LivoxPattern {
    std::vector<float> az, el;  // radians, sensor frame (x forward, z up), firing order
};

// Reads Livox's CSV (header, then index, azimuth deg, zenith deg per row). mirror_az: the
// azimuth's sense is the other way round from the sensor frame's.
inline bool LoadLivoxPattern(const std::string& path, bool mirror_az, LivoxPattern* p, std::string* err) {
    std::ifstream f(path);
    if (!f) {
        *err = "can't open " + path + " (tools/fetch_livox_pattern.sh fetches it)";
        return false;
    }
    std::string line;
    std::getline(f, line);  // header
    p->az.clear();
    p->el.clear();
    constexpr double kDeg = M_PI / 180.0;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string idx, az, ze;
        if (!std::getline(ss, idx, ',') || !std::getline(ss, az, ',') || !std::getline(ss, ze, ',')) {
            *err = path;
            *err += ": bad row '";
            *err += line;
            *err += "'";
            return false;
        }
        const double a = std::stod(az) * kDeg, e = (90.0 - std::stod(ze)) * kDeg;
        p->az.push_back(static_cast<float>(std::remainder(mirror_az ? -a : a, 2.0 * M_PI)));
        p->el.push_back(static_cast<float>(e));
    }
    if (p->az.size() < 4 || p->az.size() % 4 != 0) {
        *err = path;
        *err += ": " + std::to_string(p->az.size()) + " directions (want a multiple of 4 lines)";
        return false;
    }
    return true;
}

struct LidarGrid {
    int nh = 720, nv = 120;
    double h_min = -M_PI, h_max = M_PI - M_PI / 360.0;  // radians
    double v_min = -7.2 * M_PI / 180.0, v_max = 52.2 * M_PI / 180.0;
    double Az(int i) const { return h_min + i * (h_max - h_min) / (nh - 1); }
    double El(int j) const { return v_min + j * (v_max - v_min) / (nv - 1); }
    // The nearest ray (row * nh + column), or -1 outside the elevation range by more than
    // half a row. Azimuth wraps.
    int Nearest(double az, double el) const {
        const double dv = (v_max - v_min) / (nv - 1), dh = (h_max - h_min) / (nh - 1);
        const double fj = (el - v_min) / dv;
        if (fj < -0.5 || fj > nv - 0.5) return -1;
        const int j = std::clamp(static_cast<int>(std::lround(fj)), 0, nv - 1);
        double fa = std::remainder(az - h_min, 2.0 * M_PI);
        if (fa < 0) fa += 2.0 * M_PI;
        const int i = static_cast<int>(std::lround(fa / dh)) % nh;
        return j * nh + i;
    }
    // The four rays around a direction and their bilinear weights (azimuth wraps; the
    // elevation is clamped to the grid); false outside the field as Nearest.
    bool Around(double az, double el, int idx[4], double w[4]) const {
        const double dv = (v_max - v_min) / (nv - 1), dh = (h_max - h_min) / (nh - 1);
        double fj = (el - v_min) / dv;
        if (fj < -0.5 || fj > nv - 0.5) return false;
        fj = std::clamp(fj, 0.0, nv - 1.0);
        const int j0 = std::min(static_cast<int>(fj), nv - 2);
        const double tj = fj - j0;
        double fa = std::remainder(az - h_min, 2.0 * M_PI);
        if (fa < 0) fa += 2.0 * M_PI;
        const double fi = fa / dh;
        const double fl = std::floor(fi);
        const int i0 = static_cast<int>(fl) % nh, i1 = (i0 + 1) % nh;
        const double ti = fi - fl;
        idx[0] = j0 * nh + i0;
        idx[1] = j0 * nh + i1;
        idx[2] = (j0 + 1) * nh + i0;
        idx[3] = (j0 + 1) * nh + i1;
        w[0] = (1 - ti) * (1 - tj);
        w[1] = ti * (1 - tj);
        w[2] = (1 - ti) * tj;
        w[3] = ti * tj;
        return true;
    }
};

// The range along a direction from the four rays around it (ranges r, weights w):
// bilinear where all four hit and agree within 5%, else the ray with the most weight.
inline double GridRange(const double r[4], const double w[4]) {
    int best = 0;
    double lo = r[0], hi = r[0];
    for (int k = 1; k < 4; ++k) {
        if (w[k] > w[best]) best = k;
        lo = std::min(lo, r[k]);
        hi = std::max(hi, r[k]);
    }
    if (!std::isfinite(lo) || !std::isfinite(hi) || hi - lo > 0.05 * lo) return r[best];
    return w[0] * r[0] + w[1] * r[1] + w[2] * r[2] + w[3] * r[3];
}

struct LivoxTiming {
    int points_per_packet = 96;
    double point_dt = 4.947e-6;  // s between points in a packet
    double packets_per_frame = 208.0;
    double frame_period = 0.1;  // s
    double PacketPeriod() const { return frame_period / packets_per_frame; }
    // Time of point k (counted from the first point ever) after the first packet at t0.
    double PointTime(double t0, int64_t k) const {
        const int64_t pkt = k / points_per_packet;
        return t0 + static_cast<double>(pkt) * PacketPeriod() + static_cast<double>(k % points_per_packet) * point_dt;
    }
    // The first point of the first packet that starts at or after time t.
    int64_t FirstPointAtOrAfter(double t0, double t) const {
        const double f = (t - t0) / PacketPeriod();
        auto pkt = static_cast<int64_t>(std::ceil(f - 1e-9));
        if (pkt < 0) pkt = 0;
        return pkt * points_per_packet;
    }
};

// A ray's range at time t from renders at t1 (range r1) and t2 (r2), t1 < t2: linear
// between two hits, and on past t2 for up to one more period (beyond, held); where either
// missed (inf), the nearer render's.
inline double InterpRange(double t, double t1, double r1, double t2, double r2) {
    if (t2 <= t1) return r2;
    const double f = std::clamp((t - t1) / (t2 - t1), 0.0, 2.0);
    if (std::isfinite(r1) && std::isfinite(r2)) return r1 + f * (r2 - r1);
    return f < 0.5 ? r1 : r2;
}

struct LivoxSpec {
    double min_range = 0.1, max_range = 40.0;  // m
    double range_sigma = 0.02;                 // m
    double angle_sigma = 0.15 * M_PI / 180.0;  // rad
};

struct LivoxPointOut {
    float x = 0.0F, y = 0.0F, z = 0.0F;
    bool hit = false;
};

// A point along the ray (az, el) whose true range is r_true; no hit beyond max_range,
// under min_range (the measured range too: the real unit reports nothing nearer), or
// not finite.
template <class Rng>
LivoxPointOut LivoxReturn(const LivoxSpec& s, double az, double el, double r_true, Rng& rng) {
    LivoxPointOut p;
    if (!std::isfinite(r_true) || r_true > s.max_range || r_true < s.min_range) return p;
    std::normal_distribution<double> n(0.0, 1.0);
    const double r = r_true + s.range_sigma * n(rng);
    if (r < s.min_range) return p;
    const double a = az + s.angle_sigma * n(rng) / std::max(std::cos(el), 1e-3);
    const double e = el + s.angle_sigma * n(rng);
    p.x = static_cast<float>(r * std::cos(e) * std::cos(a));
    p.y = static_cast<float>(r * std::cos(e) * std::sin(a));
    p.z = static_cast<float>(r * std::sin(e));
    p.hit = true;
    return p;
}

}  // namespace simsens
}  // namespace gdn
