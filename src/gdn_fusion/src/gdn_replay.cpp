// gdn_replay (R1.11): replay a GDN run log offline through the same navigation core
// (gdn::NavCore) the node runs.
//
//   gdn_replay LOG                     replay with the logged configuration and check every
//                                      state, update record and event against the log, bit
//                                      for bit (exit 0 identical, 1 different, 2 error)
//   gdn_replay LOG --set=r_agl=200 --out=DIR
//                                      "what if": replay with other settings (--config=FILE,
//                                      --set=KEY=VALUE, repeatable) and write the result as a
//                                      new run log, to compare with tools/plot_run_log.py
//   --map=PATH                         the map, if it isn't at the logged path (its
//                                      fingerprint must match the logged one)
//
// LOG is a run log directory (log/gdn_runs/gdn_...) or its .mcap file.
#include <geometry_msgs/msg/vector3.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <std_msgs/msg/float64.hpp>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <deque>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>
#include "gdn/filter_config.hpp"
#include "gdn/map_db.hpp"
#include "gdn/nav_core.hpp"
#include "gdn/paths.hpp"
#include "run_log.hpp"

namespace {

template <typename MsgT>
MsgT Decode(const rosbag2_storage::SerializedBagMessage& m) {
    static rclcpp::Serialization<MsgT> ser;
    rclcpp::SerializedMessage s(*m.serialized_data);
    MsgT out;
    ser.deserialize_message(&s, &out);
    return out;
}

std::uint64_t Bits(double x) {
    std::uint64_t b = 0;
    std::memcpy(&b, &x, sizeof b);
    return b;
}

struct Replayed {
    std::uint64_t seq;
    gdn::StateVec state;
};
struct ReplayedUpdate {
    std::uint64_t seq;
    gdn::Eskf::UpdateInfo u;
};

// Collects what the replay produces, to compare with the log as the log is read, and
// optionally writes it as a new run log.
class ReplaySink : public gdn::NavSink {
public:
    explicit ReplaySink(gdn::RunLogSink* out) : out_(out) {}
    std::deque<Replayed> states;
    std::deque<ReplayedUpdate> updates;
    std::deque<std::string> events;
    void State(std::uint64_t seq, const gdn::NavCore& c) override {
        states.push_back({seq, c.State()});
        if (out_) out_->State(seq, c);
    }
    void Update(std::uint64_t seq, double t, const gdn::Eskf::UpdateInfo& u) override {
        updates.push_back({seq, u});
        if (out_) out_->Update(seq, t, u);
    }
    void Event(std::uint64_t seq, double t, gdn::EventLevel level, const std::string& text, bool console) override {
        events.push_back(text);
        if (out_) out_->Event(seq, t, level, text, console);
    }

private:
    gdn::RunLogSink* out_;
};

int Usage() {
    std::printf(
        "usage: gdn_replay LOG [--map=PATH] [--config=FILE] [--set=KEY=VALUE]... [--out=DIR]\n"
        "  Replays a GDN run log through the navigation core. Without --config/--set it checks\n"
        "  every state, update and event against the log, bit for bit.\n");
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    std::string log_path, map_arg, config_arg, out_dir;
    std::vector<std::string> sets;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a.rfind("--map=", 0) == 0)
            map_arg = a.substr(6);
        else if (a.rfind("--config=", 0) == 0)
            config_arg = a.substr(9);
        else if (a.rfind("--set=", 0) == 0)
            sets.push_back(a.substr(6));
        else if (a.rfind("--out=", 0) == 0)
            out_dir = a.substr(6);
        else if (a.rfind("--", 0) != 0 && log_path.empty())
            log_path = a;
        else
            return Usage();  // an unknown option, or a second LOG
    }
    if (log_path.empty()) return Usage();
    const bool what_if = !config_arg.empty() || !sets.empty();

    rosbag2_cpp::Reader reader;
    try {
        rosbag2_storage::StorageOptions so;
        so.uri = log_path;
        so.storage_id = "mcap";
        reader.open(so);
    } catch (const std::exception& e) {
        std::printf("ERROR: cannot open %s: %s\n", log_path.c_str(), e.what());
        if (std::filesystem::is_directory(log_path) && !std::filesystem::exists(log_path + "/metadata.yaml"))
            std::printf(
                "  No metadata.yaml: the node was probably killed. Rebuild the index with\n"
                "  ros2 bag reindex %s -s mcap\n",
                log_path.c_str());
        return 2;
    }
    if (!reader.has_next()) {
        std::printf("ERROR: %s is empty\n", log_path.c_str());
        return 2;
    }
    auto first = reader.read_next();
    if (first->topic_name != gdn::kTopicRunInfo) {
        std::printf("ERROR: %s doesn't start with %s: not a GDN run log\n", log_path.c_str(), gdn::kTopicRunInfo);
        return 2;
    }
    const auto info = Decode<gdn_msgs::msg::RunInfo>(*first);
    std::printf("log: %s\n  version %s, filter %s (fingerprint %s%s%s), trn_model %s\n", log_path.c_str(),
                info.gdn_version.c_str(), info.config_name.c_str(), info.config_fingerprint.c_str(),
                info.config_overrides.empty() ? "" : "; overrides ", info.config_overrides.c_str(),
                info.trn_model.c_str());
    if (info.state_size != static_cast<std::uint32_t>(gdn::kStateSize)) {
        std::printf("ERROR: the log's state has %u values, this build's %d\n", info.state_size, gdn::kStateSize);
        return 2;
    }

    // Configuration: the logged one, or a what-if.
    gdn::FilterConfig logged, fc;
    std::string err;
    if (!gdn::ParseFilterConfig(info.config_text, "the log's config", &logged, &err)) {
        std::printf("ERROR: %s\n", err.c_str());
        return 2;
    }
    if (gdn::FilterFingerprint(logged) != info.config_fingerprint) {
        std::printf("ERROR: the logged config's fingerprint is %s, the log says %s\n",
                    gdn::FilterFingerprint(logged).c_str(), info.config_fingerprint.c_str());
        return 2;
    }
    fc = logged;
    if (!config_arg.empty() && !gdn::LoadFilterConfig(gdn::DataPath(config_arg), &fc, &err)) {
        std::printf("ERROR: --config: %s\n", err.c_str());
        return 2;
    }
    for (const std::string& kv : sets) {
        const auto eq = kv.find('=');
        if (eq == std::string::npos || !gdn::SetFilterValue(&fc, kv.substr(0, eq), kv.substr(eq + 1), &err)) {
            std::printf("ERROR: --set=%s: %s\n", kv.c_str(),
                        eq == std::string::npos ? "expected KEY=VALUE" : err.c_str());
            return 2;
        }
    }
    if (!gdn::ValidateFilterConfig(fc, &err)) {
        std::printf("ERROR: %s\n", err.c_str());
        return 2;
    }
    if (what_if)
        std::printf("  what if: %s\n",
                    gdn::DescribeFilterConfig(fc, config_arg.empty() ? "the log's" : config_arg, &logged).c_str());

    // Map and origin.
    const std::string map_path = map_arg.empty() ? info.map_path : gdn::DataPath(map_arg);
    gdn::MapDb map;
    if (!map.Load(map_path)) {
        std::printf("ERROR: cannot load the map %s (pass --map=PATH)\n", map_path.c_str());
        return 2;
    }
    const std::string map_fp = gdn::FileFingerprint(map_path);
    if (map_fp != info.map_fingerprint) {
        std::printf("ERROR: %s has fingerprint %s, the logged map %s: not the map this run used\n", map_path.c_str(),
                    map_fp.c_str(), info.map_fingerprint.c_str());
        return 2;
    }
    if (std::isfinite(info.origin_lat) && std::isfinite(info.origin_lon))
        map.SetOrigin(info.origin_lat, info.origin_lon);

    gdn::RunLogWriter writer;
    std::int64_t clock_ns = 0;
    std::unique_ptr<gdn::RunLogSink> out_sink;
    if (!out_dir.empty()) {
        if (!writer.Open(out_dir, &err)) {
            std::printf("ERROR: --out: %s\n", err.c_str());
            return 2;
        }
        gdn_msgs::msg::RunInfo oi = info;
        oi.config_name = fc.name;
        oi.config_fingerprint = gdn::FilterFingerprint(fc);
        oi.config_overrides = gdn::FilterOverrides(logged, fc);
        oi.config_text = gdn::DumpFilterConfig(fc, "gdn_replay of " + log_path);
        writer.Write(oi, gdn::kTopicRunInfo, first->recv_timestamp);
        out_sink = std::make_unique<gdn::RunLogSink>(&writer, [&clock_ns] { return clock_ns; });
    }
    ReplaySink sink(out_sink.get());
    gdn::NavCore core(fc, map, info.trn_model == "ideal" ? gdn::TrnModel::kIdeal : gdn::TrnModel::kAgl, &sink);

    std::uint64_t n_in = 0, n_state = 0, n_upd = 0, n_ev = 0;
    std::string diff;  // the first difference
    auto note = [&diff](const std::string& d) {
        if (diff.empty()) diff = d;
    };
    while (reader.has_next()) {
        const auto m = reader.read_next();
        const std::string& topic = m->topic_name;
        clock_ns = m->recv_timestamp;
        if (topic == "/fmu/out/sensor_combined") {
            const auto s = Decode<px4_msgs::msg::SensorCombined>(*m);
            if (out_sink) writer.Write(s, topic, clock_ns);
            core.Imu(s.timestamp,
                     Eigen::Vector3d(s.accelerometer_m_s2[0], s.accelerometer_m_s2[1], s.accelerometer_m_s2[2]),
                     Eigen::Vector3d(s.gyro_rad[0], s.gyro_rad[1], s.gyro_rad[2]), s.gyro_integral_dt);
            ++n_in;
        } else if (topic == "/gdn/mag_meas") {
            const auto v = Decode<geometry_msgs::msg::Vector3>(*m);
            if (out_sink) writer.Write(v, topic, clock_ns);
            core.Mag(Eigen::Vector3d(v.x, v.y, v.z));
            ++n_in;
        } else if (topic == "/gdn/agl_meas" || topic == "/gdn/baro_meas" || topic == "/gdn/trn_meas") {
            const auto v = Decode<std_msgs::msg::Float64>(*m);
            if (out_sink) writer.Write(v, topic, clock_ns);
            if (topic == "/gdn/agl_meas")
                core.Agl(v.data);
            else if (topic == "/gdn/baro_meas")
                core.Baro(v.data);
            else
                core.Trn(v.data);
            ++n_in;
        } else if (topic == "/fmu/out/vehicle_local_position") {
            if (out_sink) writer.Write(Decode<px4_msgs::msg::VehicleLocalPosition>(*m), topic, clock_ns);
        } else if (topic == "/fmu/out/vehicle_attitude") {
            if (out_sink) writer.Write(Decode<px4_msgs::msg::VehicleAttitude>(*m), topic, clock_ns);
        } else if (topic == gdn::kTopicState && !what_if) {
            const auto ls = Decode<gdn_msgs::msg::NavState>(*m);
            if (sink.states.empty() || sink.states.front().seq != ls.seq) {
                note("logged state for input " + std::to_string(ls.seq) + " has no replayed state");
                continue;
            }
            const gdn::StateVec a = gdn::FromMsg(ls), b = sink.states.front().state;
            sink.states.pop_front();
            ++n_state;
            for (int k = 0; k < gdn::kStateSize; ++k)
                if (Bits(a[k]) != Bits(b[k])) {
                    char d[200];
                    std::snprintf(d, sizeof d, "state after input %llu (t=%.3f s): %s logged %.17g, replayed %.17g",
                                  static_cast<unsigned long long>(ls.seq), a[0], gdn::StateName(k).c_str(), a[k], b[k]);
                    note(d);
                    break;
                }
        } else if (topic == gdn::kTopicUpdate && !what_if) {
            const auto lu = Decode<gdn_msgs::msg::NavUpdate>(*m);
            if (sink.updates.empty()) {
                note("logged update for input " + std::to_string(lu.seq) + " has no replayed update");
                continue;
            }
            const ReplayedUpdate r = sink.updates.front();
            sink.updates.pop_front();
            ++n_upd;
            bool same = r.seq == lu.seq && static_cast<std::uint8_t>(r.u.sensor) == lu.sensor &&
                        static_cast<std::uint8_t>(r.u.reason) == lu.reason && Bits(r.u.nis) == Bits(lu.nis) &&
                        lu.innovation.size() == static_cast<size_t>(r.u.dim) && r.u.limited == lu.gain_limited;
            for (int i = 0; same && i < r.u.dim; ++i)
                same = Bits(r.u.y[i]) == Bits(lu.innovation[i]) && Bits(r.u.s[i]) == Bits(lu.innovation_var[i]);
            if (!same) note("update record for input " + std::to_string(lu.seq) + " differs");
        } else if (topic == gdn::kTopicEvent && !what_if) {
            const auto le = Decode<gdn_msgs::msg::NavEvent>(*m);
            if (sink.events.empty() || sink.events.front() != le.text) {
                note("event '" + le.text + "' at input " + std::to_string(le.seq) + " not replayed");
                continue;
            }
            sink.events.pop_front();
            ++n_ev;
        }
    }
    writer.Close();

    const gdn::Eskf& e = core.eskf();
    std::printf("replayed %llu inputs: final t=%.1f s, position (%.1f, %.1f, %.1f) m, sigma %.1f m\n",
                static_cast<unsigned long long>(n_in), core.t(), e.pos().x(), e.pos().y(), e.pos().z(), e.sigma_pos());
    if (!out_dir.empty()) std::printf("wrote %s\n", out_dir.c_str());
    if (what_if) {
        std::printf("what-if replay: nothing compared\n");
        return 0;
    }
    // Records the replay made after the log ended (the node stopped mid-input) aren't
    // differences; anything missing or different is.
    if (!diff.empty()) {
        std::printf("DIFFERENT: %s\n  (compared %llu states, %llu updates, %llu events before it)\n", diff.c_str(),
                    static_cast<unsigned long long>(n_state), static_cast<unsigned long long>(n_upd),
                    static_cast<unsigned long long>(n_ev));
        return 1;
    }
    if (n_state == 0) {
        std::printf("ERROR: the log has no states to compare\n");
        return 2;
    }
    std::printf("IDENTICAL: %llu states, %llu update records and %llu events match the log bit for bit\n",
                static_cast<unsigned long long>(n_state), static_cast<unsigned long long>(n_upd),
                static_cast<unsigned long long>(n_ev));
    return 0;
}
