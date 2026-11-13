#pragma once
// The GDN run log (R1.11): an MCAP file per run, written by eskf_node and by gdn_replay
// --out, read by gdn_replay and tools/plot_run_log.py. ROS (rosbag2), so it lives here
// and not in the ROS-free include/gdn/.
//
// Topics, in the order they are written (log time strictly increasing, so readers see
// exactly the order the node processed things):
//   /gdn/log/run_info   gdn_msgs/RunInfo    once, first: config text and fingerprint, map,
//                                           origin, version
//   input topics        as received         /fmu/out/sensor_combined, /gdn/mag_meas,
//                                           /gdn/agl_meas, /gdn/baro_meas, /gdn/trn_meas,
//                                           and the PX4 reference (local position, attitude)
//   /gdn/log/state      gdn_msgs/NavState   after every input (seq = input number)
//   /gdn/log/update     gdn_msgs/NavUpdate  every measurement and ZUPT, skipped ones too
//   /gdn/log/event      gdn_msgs/NavEvent   start-up steps, IMU holes, dropped samples
#include <gdn_msgs/msg/nav_event.hpp>
#include <gdn_msgs/msg/nav_state.hpp>
#include <gdn_msgs/msg/nav_update.hpp>
#include <gdn_msgs/msg/run_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <functional>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include "gdn/nav_core.hpp"

namespace gdn {

inline constexpr const char* kTopicRunInfo = "/gdn/log/run_info";
inline constexpr const char* kTopicState = "/gdn/log/state";
inline constexpr const char* kTopicUpdate = "/gdn/log/update";
inline constexpr const char* kTopicEvent = "/gdn/log/event";

// The message constants and the core's enums must agree.
static_assert(gdn_msgs::msg::NavUpdate::SENSOR_TRN == static_cast<std::uint8_t>(Eskf::Sensor::kTrn));
static_assert(gdn_msgs::msg::NavUpdate::SENSOR_GNSS == static_cast<std::uint8_t>(Eskf::Sensor::kGnss));
static_assert(gdn_msgs::msg::NavUpdate::APPLIED == static_cast<std::uint8_t>(Eskf::Reason::kApplied));
static_assert(gdn_msgs::msg::NavUpdate::INNOV_GATE == static_cast<std::uint8_t>(Eskf::Reason::kInnovGate));
static_assert(gdn_msgs::msg::NavUpdate::NOT_USED == static_cast<std::uint8_t>(Eskf::Reason::kNotUsed));
static_assert(gdn_msgs::msg::NavEvent::ERROR == static_cast<std::uint8_t>(EventLevel::kError));

// 64-bit FNV-1a of a file's bytes, 16 hex digits ("" if it can't be read).
inline std::string FileFingerprint(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return "";
    std::uint64_t h = 1469598103934665603ULL;
    char buf[65536];
    while (f.read(buf, sizeof buf) || f.gcount() > 0) {
        for (std::streamsize i = 0; i < f.gcount(); ++i) {
            h ^= static_cast<unsigned char>(buf[i]);
            h *= 1099511628211ULL;
        }
    }
    char out[20];
    std::snprintf(out, sizeof out, "%016llx", static_cast<unsigned long long>(h));
    return out;
}

inline gdn_msgs::msg::NavState ToMsg(std::uint64_t seq, const StateVec& v) {
    gdn_msgs::msg::NavState m;
    m.seq = seq;
    int k = 0;
    m.t = v[k++];
    for (double& x : m.position) x = v[k++];
    for (double& x : m.velocity) x = v[k++];
    for (double& x : m.orientation) x = v[k++];
    for (double& x : m.accel_bias) x = v[k++];
    for (double& x : m.gyro_bias) x = v[k++];
    m.baro_bias = v[k++];
    for (double& x : m.mag_bias) x = v[k++];
    for (double& x : m.p_diag) x = v[k++];
    m.p_ne = v[k++];
    m.flags = static_cast<std::uint8_t>(v[k++]);
    return m;
}

inline StateVec FromMsg(const gdn_msgs::msg::NavState& m) {
    StateVec v{};
    int k = 0;
    v[k++] = m.t;
    for (double x : m.position) v[k++] = x;
    for (double x : m.velocity) v[k++] = x;
    for (double x : m.orientation) v[k++] = x;
    for (double x : m.accel_bias) v[k++] = x;
    for (double x : m.gyro_bias) v[k++] = x;
    v[k++] = m.baro_bias;
    for (double x : m.mag_bias) v[k++] = x;
    for (double x : m.p_diag) v[k++] = x;
    v[k++] = m.p_ne;
    v[k++] = m.flags;
    return v;
}

// Name of state element k, for reports of a mismatch.
inline std::string StateName(int k) {
    static const char* const names[] = {"t",    "pn",   "pe",   "pd",        "vn",   "ve",   "vd",
                                        "qw",   "qx",   "qy",   "qz",        "ba_x", "ba_y", "ba_z",
                                        "bg_x", "bg_y", "bg_z", "baro_bias", "bm_x", "bm_y", "bm_z"};
    if (k < 21) return names[k];
    if (k < 40) return "p_diag[" + std::to_string(k - 21) + "]";
    return k == 40 ? "p_ne" : "flags";
}

inline gdn_msgs::msg::NavUpdate ToMsg(std::uint64_t seq, double t, const Eskf::UpdateInfo& u) {
    gdn_msgs::msg::NavUpdate m;
    m.seq = seq;
    m.t = t;
    m.sensor = static_cast<std::uint8_t>(u.sensor);
    m.reason = static_cast<std::uint8_t>(u.reason);
    m.innovation.assign(u.y, u.y + u.dim);
    m.innovation_var.assign(u.s, u.s + u.dim);
    m.nis = u.nis;
    m.gain_limited = u.limited;
    return m;
}

// Writes one run log. Log times are strictly increasing (the clock, or one nanosecond
// after the previous message), so a reader gets the messages in the order written.
class RunLogWriter {
public:
    // dir: the bag directory to create (must not exist).
    bool Open(const std::string& dir, std::string* err) {
        try {
            rosbag2_storage::StorageOptions so;
            so.uri = dir;
            so.storage_id = "mcap";
            so.storage_preset_profile = "zstd_fast";
            writer_ = std::make_unique<rosbag2_cpp::Writer>();
            writer_->open(so);
        } catch (const std::exception& e) {
            *err = e.what();
            writer_.reset();
            return false;
        }
        dir_ = dir;
        return true;
    }
    bool is_open() const { return writer_ != nullptr; }
    const std::string& dir() const { return dir_; }
    std::uint64_t written() const { return n_; }

    template <typename MsgT>
    void Write(const MsgT& msg, const std::string& topic, std::int64_t clock_ns) {
        if (!writer_) return;
        last_ns_ = clock_ns > last_ns_ ? clock_ns : last_ns_ + 1;
        writer_->write(msg, topic, rclcpp::Time(last_ns_, RCL_SYSTEM_TIME));
        ++n_;
    }
    void Close() { writer_.reset(); }

private:
    std::unique_ptr<rosbag2_cpp::Writer> writer_;
    std::string dir_;
    std::int64_t last_ns_ = 0;
    std::uint64_t n_ = 0;
};

// NavSink that writes states, updates and events to a run log, and passes events and
// updates on to an optional callback (the node prints them).
class RunLogSink : public NavSink {
public:
    using Clock = std::function<std::int64_t()>;
    RunLogSink(RunLogWriter* w, Clock clock) : w_(w), clock_(std::move(clock)) {}
    std::function<void(EventLevel, const std::string&, bool)> on_event;
    std::function<void(const Eskf::UpdateInfo&)> on_update;

    void State(std::uint64_t seq, const NavCore& c) override {
        if (w_->is_open()) w_->Write(ToMsg(seq, c.State()), kTopicState, clock_());
    }
    void Update(std::uint64_t seq, double t, const Eskf::UpdateInfo& u) override {
        if (w_->is_open()) w_->Write(ToMsg(seq, t, u), kTopicUpdate, clock_());
        if (on_update) on_update(u);
    }
    void Event(std::uint64_t seq, double t, EventLevel level, const std::string& text, bool console) override {
        if (w_->is_open()) {
            gdn_msgs::msg::NavEvent m;
            m.seq = seq;
            m.t = t;
            m.level = static_cast<std::uint8_t>(level);
            m.text = text;
            w_->Write(m, kTopicEvent, clock_());
        }
        if (on_event) on_event(level, text, console);
    }

private:
    RunLogWriter* w_;
    Clock clock_;
};

}  // namespace gdn
