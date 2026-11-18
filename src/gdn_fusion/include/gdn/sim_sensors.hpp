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
//                message of the same sensor), and whether it's lost.
//   RangeReading the TFmini-S: +inf beyond max range, -inf under min range, else the
//                true range plus range-dependent noise, rounded to the resolution.
//   ClockMap     simulation time to wall time: the smallest wall-minus-sim offset seen
//                over the last window (the transport delay's floor), smoothed (a jump in
//                the floor would show as stamp jitter), plus headroom so a message's
//                arrival is normally still ahead when its raw sample comes in (else the
//                bridge's own delay would eat the configured latency). Stamps follow the
//                simulation clock, not the bridge's jitter; all sensors share one map.
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <limits>
#include <random>
#include <utility>

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
    Message Apply(double sample_t) {
        Message m;
        m.lost = u_(rng_) < t_.drop_prob;
        double lat = t_.latency + t_.latency_jitter * n_(rng_);
        if (lat < 0.0) lat = 0.0;
        m.arrival = std::max(sample_t + lat, last_arrival_);  // a driver delivers in order
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

class ClockMap {
public:
    explicit ClockMap(double window_s = 2.0, double tau_s = 1.0, double headroom_s = 0.005)
        : window_(window_s), tau_(tau_s), headroom_(headroom_s) {}
    // A raw message stamped sim_t arrived at wall time wall_t.
    void Observe(double sim_t, double wall_t) {
        const double d = wall_t - sim_t;
        while (!q_.empty() && q_.back().second >= d) q_.pop_back();  // monotonic min-queue
        q_.emplace_back(wall_t, d);
        while (q_.front().first < wall_t - window_) q_.pop_front();
        const double floor = q_.front().second;
        if (!init_) {
            off_ = floor;
            init_ = true;
        } else if (sim_t > last_sim_) {
            off_ += -std::expm1(-(sim_t - last_sim_) / tau_) * (floor - off_);
        }
        last_sim_ = std::max(last_sim_, sim_t);
    }
    bool Ready() const { return init_; }
    double ToWall(double sim_t) const { return sim_t + off_ + headroom_; }

private:
    double window_, tau_, headroom_;
    std::deque<std::pair<double, double>> q_;  // (wall time, offset), offsets increasing
    bool init_ = false;
    double off_ = 0.0, last_sim_ = -std::numeric_limits<double>::infinity();
};

}  // namespace simsens
}  // namespace gdn
