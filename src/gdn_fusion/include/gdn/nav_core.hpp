#pragma once
// The node's navigation logic, ROS-free (R1.11). ROS-free.
//
// eskf_node used to make its decisions inside the ROS callbacks: when to align, when the
// vehicle is still (ZUPT), when to set the altitude datum, which magnetometer update to
// run, what to do with a dropped or missing IMU sample. They live here now, so the node
// and the offline replay (gdn_replay) run the same code: fed the inputs a run logged, in
// the logged order, NavCore reproduces that run's states bit for bit. Nothing here reads
// a clock; time is the IMU timestamps.
//
// Every input is reported to a NavSink: one State after each input, one Update record
// for each measurement (skipped ones too, with the reason) and each ZUPT, and Events for
// start-up steps, IMU holes and dropped samples. The node's sink writes them to the run
// log (MCAP); the replay's sink compares them with the log.
#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include "gdn/eskf.hpp"
#include "gdn/filter_config.hpp"
#include "gdn/ground_monitor.hpp"
#include "gdn/map_db.hpp"

namespace gdn {

enum class TrnModel : std::uint8_t { kAgl, kIdeal };  // node parameter trn_model
enum class EventLevel : std::uint8_t { kInfo, kWarn, kError };

// The state as logged: t, position (3), velocity (3), attitude quaternion w x y z (4),
// accel bias (3), gyro bias (3), baro bias, mag bias (3), covariance diagonal (19), the
// north-east covariance P(0,1), and flags (1 aligned, 2 heading aligned, 4 altitude
// datum set, 8 airborne).
inline constexpr int kStateSize = 42;
using StateVec = std::array<double, kStateSize>;

class NavCore;
struct NavSink {
    virtual ~NavSink() = default;
    // seq: the input's number (0, 1, ...); every record carries the input that caused it.
    virtual void State(std::uint64_t seq, const NavCore& core) {
        (void)seq;
        (void)core;
    }
    virtual void Update(std::uint64_t seq, double t, const Eskf::UpdateInfo& u) {
        (void)seq;
        (void)t;
        (void)u;
    }
    // console: whether the node prints it too (repeated warnings are thinned on the
    // console; the log keeps every one).
    virtual void Event(std::uint64_t seq, double t, EventLevel level, const std::string& text, bool console) {
        (void)seq;
        (void)t;
        (void)level;
        (void)text;
        (void)console;
    }
};

class NavCore {
public:
    using V3 = Eigen::Vector3d;
    using Sensor = Eskf::Sensor;
    using Reason = Eskf::Reason;

    NavCore(const FilterConfig& fc, const MapDb& map, TrnModel model, NavSink* sink = nullptr)
        : fc_(fc), map_(map), model_(model), sink_(sink), eskf_(fc.eskf), ground_(fc.ground) {}

    // IMU sample: PX4 timestamp (us), specific force (m/s^2), rate (rad/s), and the time
    // the sample's mean covers (sensor_combined gyro_integral_dt, us). Returns true when
    // the filter was propagated (the node publishes odometry then).
    bool Imu(std::uint64_t t_us, const V3& a_m, const V3& w_m, std::uint32_t integral_us) {
        bool propagated = false;
        if (!eskf_.aligned()) {
            if (eskf_.FeedStatic(a_m, w_m)) {
                char b[160];
                std::snprintf(b, sizeof b, "static alignment: ba=%.4f m/s2, bg=%.4f deg/s", eskf_.accel_bias().norm(),
                              eskf_.gyro_bias().norm() / Eskf::kDeg);
                Emit(EventLevel::kInfo, b, true);
                // R1.2: on-ground detection is referenced to the aligned rest state.
                ground_.Start(eskf_.RestSpecificForce(), eskf_.gyro_bias());
            }
            return Done(propagated);
        }
        t_cur_ = static_cast<double>(t_us) * 1e-6;
        if (last_t_ == 0) {
            last_t_ = t_us;
            return Done(propagated);
        }
        // Signed difference: a timestamp going backwards gives dt < 0 (dropped), not an
        // unsigned wrap to about 1.8e13 s (R1.9).
        double dt = static_cast<double>(static_cast<std::int64_t>(t_us - last_t_)) * 1e-6;
        last_t_ = t_us;
        if (dt <= 0.0 || dt > 0.25) {  // wrap/stall: skip, never fabricate
            n_drop_++;
            char b[120];
            std::snprintf(b, sizeof b, "IMU sample dropped: dt %.3f s (count %d)", dt, n_drop_);
            Emit(EventLevel::kWarn, b, false);
            return Done(propagated);
        }
        if (dt > 0.1) {
            n_stall_++;
            char b[120];
            std::snprintf(b, sizeof b, "IMU gap absorbed %.3f s (count %d)", dt, n_stall_);
            Emit(EventLevel::kWarn, b, n_stall_ % 10 == 1);
        }
        // Each sensor_combined message holds the mean rate over gyro_integral_dt. If that's
        // much shorter than the spacing between messages, samples are being dropped on the
        // way (e.g. uXRCE-DDS's 100 Hz cap with PX4 integrating at 250 Hz), and most of the
        // motion never reaches the filter. Say so loudly; the fix is IMU_INTEG_RATE.
        if (integral_us > 0 && dt * 1e6 > 1.5 * integral_us) {
            if (++n_undercover_ == 200) {
                char b[300];
                std::snprintf(b, sizeof b,
                              "IMU messages cover only %.0f%% of the time between them "
                              "(integral %.1f ms, spacing %.1f ms): samples are dropped upstream. "
                              "On PX4 set IMU_INTEG_RATE to the DDS rate (100).",
                              100.0 * integral_us * 1e-6 / dt, integral_us * 1e-3, dt * 1e3);
                Emit(EventLevel::kError, b, true);
            }
        }
        eskf_.Propagate(a_m, w_m, dt);
        propagated = true;
        // Time not covered by this message's integral means IMU data was lost. The
        // propagation above guessed the motion in the hole from this one sample, so widen
        // the attitude and velocity uncertainty by what could have happened in it
        // (config gap_*: by default twice the current rate times the hole, plus 1 degree;
        // 2 m/s^2 for velocity).
        const double covered = integral_us > 0 ? integral_us * 1e-6 : dt;
        const double hole = dt - covered;
        if (hole > 0.02) {
            eskf_.InflateForGap(
                fc_.gap_att_rate_scale * (w_m - eskf_.gyro_bias()).norm() * hole + fc_.gap_att_floor_deg * Eskf::kDeg,
                fc_.gap_vel_accel * hole);
            char b[160];
            std::snprintf(b, sizeof b, "IMU hole of %.0f ms (rate %.0f dps): attitude uncertainty widened (count %d)",
                          hole * 1e3, w_m.norm() / Eskf::kDeg, eskf_.gaps());
            Emit(EventLevel::kWarn, b, eskf_.gaps() % 20 == 1);
        }
        // R1.2: ZUPT while the IMU says the vehicle is still; sustained motion sets the
        // airborne latch. No GNSS input (gdn/ground_monitor.hpp, TEST-018).
        const bool was_airborne = ground_.airborne();
        if (ground_.Update(a_m, w_m, dt)) {
            eskf_.ApplyZupt();
            Record(eskf_.last_update());
        }
        if (ground_.airborne() && !was_airborne) {
            char b[120];
            std::snprintf(b, sizeof b, "airborne latch set at t=%.1f (IMU motion for %.1f s): ZUPT off", t_cur_,
                          fc_.ground.latch_s);
            Emit(EventLevel::kInfo, b, true);
        }
        return Done(propagated);
    }

    // Magnetometer, body frame, uT.
    void Mag(const V3& z) {
        const Sensor s = fc_.mag_fusion == "heading" ? Sensor::kMagHeading : Sensor::kMag3;
        if (!eskf_.aligned()) {
            Skip(s, Reason::kNotAligned);
        } else if (!eskf_.yaw_aligned()) {
            eskf_.AlignYawFromMag(z);
            Skip(s, Reason::kInit);
            char b[80];
            std::snprintf(b, sizeof b, "heading from magnetometer at t=%.1f", t_cur_);
            Emit(EventLevel::kInfo, b, true);
        } else if (fc_.mag_fusion == "auto") {
            eskf_.ApplyMagAuto(z);
            Record(eskf_.last_update());
        } else if (fc_.mag_fusion == "heading") {
            eskf_.ApplyMagHeading(z);
            Record(eskf_.last_update());
        } else if (fc_.mag_fusion == "full") {
            eskf_.ApplyMag(z);
            Record(eskf_.last_update());
        } else {
            Skip(s, Reason::kNotUsed);  // "align": initial heading only
        }
        Done(false);
    }

    // Terrain height directly ("ideal" model only), m.
    void Trn(double h) {
        if (model_ != TrnModel::kIdeal) {
            Skip(Sensor::kTrn, Reason::kNotUsed);
        } else {
            eskf_.ApplyTrn(h, map_.Query(eskf_.pos().x(), eskf_.pos().y()));
            Record(eskf_.last_update());
        }
        Done(false);
    }

    // R1.1: height above ground (tilt-corrected, metres). The first reading after heading
    // alignment fixes the altitude datum: AMSL height of p_z = 0 = map height under the
    // estimate + AGL + current p_z. The baro bias is calibrated at the same moment.
    void Agl(double agl) {
        if (model_ != TrnModel::kAgl) {
            Skip(Sensor::kAgl, Reason::kNotUsed);
        } else if (!eskf_.aligned() || !eskf_.yaw_aligned()) {
            Skip(Sensor::kAgl, !eskf_.aligned() ? Reason::kNotAligned : Reason::kNoHeading);
        } else {
            const MapQuery mq = map_.Query(eskf_.pos().x(), eskf_.pos().y());
            if (!mq.ok) {
                Skip(Sensor::kAgl, Reason::kOffMap);
            } else if (!eskf_.datum_set()) {
                eskf_.SetAltitudeDatum(mq.h + agl + eskf_.pos().z(), fc_.datum_sigma_m);
                if (have_baro_) eskf_.InitBaroBias(last_baro_, fc_.baro_init_sigma_m);
                z_initialized_ = true;
                Skip(Sensor::kAgl, Reason::kInit);
                char b[160];
                std::snprintf(b, sizeof b, "altitude datum %.1f m AMSL (map %.1f + AGL %.1f), baro bias %.2f m",
                              eskf_.altitude_datum(), mq.h, agl, eskf_.baro_bias());
                Emit(EventLevel::kInfo, b, true);
            } else {
                eskf_.ApplyAgl(agl, mq);
                Record(eskf_.last_update());
            }
        }
        Done(false);
    }

    // Baro altitude, NED z (m): z_true + bias + noise.
    void Baro(double z) {
        last_baro_ = z;
        have_baro_ = true;
        // R1.2, "ideal" mode: the local frame's origin is the start point, so the vehicle
        // starts at z = 0 and the first baro reading after alignment sets the baro bias.
        // In "agl" mode Agl() sets the altitude datum and the bias instead.
        if (model_ == TrnModel::kIdeal && !z_initialized_ && eskf_.aligned()) {
            eskf_.InitBaroBias(z, fc_.baro_init_sigma_m);
            z_initialized_ = true;
            Skip(Sensor::kBaro, Reason::kInit);
            char b[120];
            std::snprintf(b, sizeof b, "start altitude z=%.2f m, baro bias %.2f m from first baro", eskf_.pos().z(),
                          eskf_.baro_bias());
            Emit(EventLevel::kInfo, b, true);
        } else if (z_initialized_ && eskf_.aligned()) {
            eskf_.ApplyBaro(z);
            Record(eskf_.last_update());
        } else {
            Skip(Sensor::kBaro, !eskf_.aligned() ? Reason::kNotAligned : Reason::kNoDatum);
        }
        Done(false);
    }

    const Eskf& eskf() const { return eskf_; }
    const GroundMonitor& ground() const { return ground_; }
    const FilterConfig& config() const { return fc_; }
    TrnModel model() const { return model_; }
    std::uint64_t inputs() const { return seq_; }  // inputs processed so far
    double t() const { return t_cur_; }            // last IMU time, s
    bool have_baro() const { return have_baro_; }
    int drops() const { return n_drop_; }
    int stalls() const { return n_stall_; }

    StateVec State() const {
        StateVec v{};
        int k = 0;
        v[k++] = t_cur_;
        for (int i = 0; i < 3; ++i) v[k++] = eskf_.pos()(i);
        for (int i = 0; i < 3; ++i) v[k++] = eskf_.vel()(i);
        v[k++] = eskf_.q().w();
        v[k++] = eskf_.q().x();
        v[k++] = eskf_.q().y();
        v[k++] = eskf_.q().z();
        for (int i = 0; i < 3; ++i) v[k++] = eskf_.accel_bias()(i);
        for (int i = 0; i < 3; ++i) v[k++] = eskf_.gyro_bias()(i);
        v[k++] = eskf_.baro_bias();
        for (int i = 0; i < 3; ++i) v[k++] = eskf_.mag_bias()(i);
        for (int i = 0; i < 19; ++i) v[k++] = eskf_.P()(i, i);
        v[k++] = eskf_.P()(0, 1);
        v[k++] = (eskf_.aligned() ? 1 : 0) + (eskf_.yaw_aligned() ? 2 : 0) + (eskf_.datum_set() ? 4 : 0) +
                 (ground_.airborne() ? 8 : 0);
        return v;
    }

private:
    bool Done(bool propagated) {
        if (sink_) sink_->State(seq_, *this);
        ++seq_;
        return propagated;
    }
    void Record(const Eskf::UpdateInfo& u) {
        if (sink_) sink_->Update(seq_, t_cur_, u);
    }
    void Skip(Sensor s, Reason r) {
        Eskf::UpdateInfo u;
        u.sensor = s;
        u.reason = r;
        Record(u);
    }
    void Emit(EventLevel level, const std::string& text, bool console) {
        if (sink_) sink_->Event(seq_, t_cur_, level, text, console);
    }

    FilterConfig fc_;
    const MapDb& map_;
    TrnModel model_;
    NavSink* sink_;
    Eskf eskf_;
    GroundMonitor ground_;
    std::uint64_t seq_ = 0;
    std::uint64_t last_t_ = 0;
    double t_cur_ = 0.0;
    int n_drop_ = 0, n_stall_ = 0, n_undercover_ = 0;
    bool have_baro_ = false, z_initialized_ = false;
    double last_baro_ = 0.0;
};

}  // namespace gdn
