#pragma once
// R1.2: on-ground detection from the IMU alone, for ZUPT and the airborne latch.
// ROS-free, shared by eskf_node and mc_campaign so both run the same start-up path.
//
// Before R1.2 the node ran ZUPT until GNSS speed passed 2 m/s. That needs GNSS (so
// it can't exist in a GNSS-denied system), and it kept telling the filter "velocity
// is zero" for the first 2 m/s of the take-off roll (TEST-018 control: 0.75 m/s left
// over). Here "still" means the IMU reads what it read when the filter aligned:
//
//   | mean(a_m) - f_rest | < accel_tol   and   | mean(w_m) - w_rest | < gyro_tol
//
// with both means over the last window_s. f_rest = R^T(-g) + b_a and w_rest = b_g,
// taken at alignment. The window mean averages out sensor noise and vibration
// (0.15 m/s^2 per sample becomes about 0.03 over 20 samples), while any real
// acceleration or turn moves the mean. A constant acceleration is caught too,
// which a variance-only test would miss.
//
// Motion that lasts latch_s sets the airborne latch, and ZUPT never runs again.
// Shorter motion (a bump, a gust) only suspends ZUPT. A missed ZUPT costs a little
// drift on the ground; a false one corrupts velocity, so every doubt goes towards
// "moving".
//
// Limits: the reference is the alignment pose, so a vehicle re-parked at a different
// tilt reads as moving and latches (safe direction). Creeping at under about
// accel_tol (0.3 m/s^2 in config/filter/default.yaml) is not seen. Landing and re-take-off need a regime input
// from the autopilot (R2.5); until then the latch is one-way.
#include <Eigen/Dense>
#include <deque>
#include "gdn/unset.hpp"

namespace gdn {

// Settings from the config file (ground_* keys, R1.10); no values in the code.
struct GroundConfig {
    double window_s = kUnset;       // averaging window, s
    double accel_tol = kUnset;      // m/s^2 on the window-mean specific force
    double gyro_tol = kUnset;       // rad/s on the window-mean rate
    double latch_s = kUnset;        // continuous motion that sets the airborne latch, s
    double zupt_period_s = kUnset;  // ZUPT interval while still, s
};

class GroundMonitor {
public:
    using V3 = Eigen::Vector3d;
    explicit GroundMonitor(const GroundConfig& cfg) : cfg_(cfg) {}

    // Call once, right after static alignment, with Eskf::RestSpecificForce() and
    // Eskf::gyro_bias().
    void Start(const V3& f_rest, const V3& w_rest) {
        f_ref_ = f_rest; w_ref_ = w_rest; started_ = true;
        buf_.clear(); t_ = span_ = moving_for_ = since_zupt_ = 0.0;
        airborne_ = still_ = false; latch_t_ = -1.0;
    }

    // Feed every IMU sample (dt in seconds). Returns true when a ZUPT is due now.
    bool Update(const V3& a_m, const V3& w_m, double dt) {
        if (!started_ || airborne_ || !(dt > 0.0)) return false;
        t_ += dt;
        buf_.push_back({a_m, w_m, dt});
        span_ += dt;
        while (buf_.size() > 1 && span_ - buf_.front().dt >= cfg_.window_s) {
            span_ -= buf_.front().dt; buf_.pop_front();
        }
        V3 sa = V3::Zero(), sw = V3::Zero();
        for (const Sample& s : buf_) { sa += s.a * s.dt; sw += s.w * s.dt; }
        const bool full = span_ >= 0.99 * cfg_.window_s;
        still_ = full && (sa / span_ - f_ref_).norm() < cfg_.accel_tol &&
                 (sw / span_ - w_ref_).norm() < cfg_.gyro_tol;
        if (!still_ && full) {
            moving_for_ += dt;
            if (moving_for_ >= cfg_.latch_s) { airborne_ = true; latch_t_ = t_; }
            since_zupt_ = 0.0;
            return false;
        }
        if (!still_) return false;                  // window still filling
        moving_for_ = 0.0;
        since_zupt_ += dt;
        if (since_zupt_ + 1e-9 < cfg_.zupt_period_s) return false;
        since_zupt_ = 0.0;
        return true;
    }

    bool started() const { return started_; }
    bool airborne() const { return airborne_; }
    bool still() const { return still_; }
    // Seconds since Start() at which the latch was set; -1 while on the ground.
    double latch_time() const { return latch_t_; }

private:
    struct Sample { V3 a, w; double dt; };
    GroundConfig cfg_;
    V3 f_ref_ = V3::Zero(), w_ref_ = V3::Zero();
    std::deque<Sample> buf_;
    double t_ = 0.0, span_ = 0.0, moving_for_ = 0.0, since_zupt_ = 0.0, latch_t_ = -1.0;
    bool started_ = false, airborne_ = false, still_ = false;
};

}  // namespace gdn
