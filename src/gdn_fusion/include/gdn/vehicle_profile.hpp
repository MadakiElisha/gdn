#pragma once
// Vehicle profile (roadmap R2.5a). ROS-free.
//
// One file per airframe, config/vehicles/<name>.yaml: the filter settings it runs with
// (a filter config file plus overrides), which aiding GDN uses, and every sensor with its
// driver topic, frame, noise model and mount (extrinsics). Flat "key: value" like the
// filter configuration (gdn/kv_file.hpp); every key and rule is described in
// tools/vehicle_profile.py, which reads it the same way and gives the same fingerprint.
// tools/check_profiles.py also checks the files it names exist and the mounts against
// the simulation.
//
// Mounts are the pose of a sensor's frame_id in the airframe's base_link, ROS convention
// (x forward, y left, z up; roll, pitch, yaw in degrees about the fixed axes). GDN doesn't
// use them yet (Phase 3's LIO and VIO will); the node checks them and logs them.
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <initializer_list>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include "gdn/filter_config.hpp"
#include "gdn/kv_file.hpp"

namespace gdn {

inline constexpr int kVehicleProfileSchema = 1;

// Which aiding GDN uses (gdn::NavCore). All on is how GDN ran before profiles.
//   agl  : height-above-ground updates (terrain). Off: no TRN; the altitude datum (set by
//          the first AGL reading) is never set, so the barometer isn't applied either.
//   baro : barometer updates.
//   mag  : magnetometer updates in flight. Off: the magnetometer still gives the initial
//          heading (GDN has no other source of it yet), like mag_fusion "align".
//   zupt : zero-velocity updates while the IMU says the vehicle is still.
struct Aiding {
    bool agl = true, baro = true, mag = true, zupt = true;
};

struct VehicleSensor {
    std::string id, kind, topic, frame_id, mount_source, gdn_input, noise, sim, note;
    double xyz[3] = {0, 0, 0};      // m, in base_link
    double rpy_deg[3] = {0, 0, 0};  // deg, roll pitch yaw about fixed x, y, z
};

struct VehicleProfile {
    std::string name, kind, px4_airframe, sim_model, filter_config, description, note;
    std::vector<std::pair<std::string, std::string>> filter_overrides;  // in file order
    Aiding aiding;
    std::vector<VehicleSensor> sensors;
    std::string fingerprint;  // 12 hex digits, ProfileFingerprint
    std::string text;         // the file as read, for the run log
};

namespace profile_detail {

// a + b + c ... without a temporary per '+' (clang-tidy performance-inefficient-string-concatenation).
template <typename... A>
std::string Cat(const A&... a) {
    std::string s;
    ((s += a), ...);
    return s;
}

}  // namespace profile_detail

// 64-bit FNV-1a of "key=value\n" lines sorted by key, as 12 hex digits: comments, order
// and spacing don't count, any value does (tools/vehicle_profile.py: fingerprint).
inline std::string ProfileFingerprint(const std::vector<KeyValue>& kvs) {
    std::vector<std::pair<std::string, std::string>> s;
    s.reserve(kvs.size());
    for (const KeyValue& kv : kvs) s.emplace_back(kv.key, kv.value);
    std::sort(s.begin(), s.end());
    std::uint64_t h = 1469598103934665603ULL;
    for (const auto& [k, v] : s) {
        const std::string line = profile_detail::Cat(k, "=", v, "\n");
        for (const unsigned char ch : line) {
            h ^= ch;
            h *= 1099511628211ULL;
        }
    }
    char buf[20];
    std::snprintf(buf, sizeof buf, "%012llx", static_cast<unsigned long long>(h >> 16));
    return buf;
}

namespace profile_detail {

inline bool OneOf(const std::string& v, std::initializer_list<const char*> options) {
    for (const char* o : options)
        if (v == o) return true;
    return false;
}

inline bool ParseVec3(const std::string& v, double out[3]) {
    std::istringstream is(v);
    for (int i = 0; i < 3; ++i) {
        std::string tok;
        if (!(is >> tok)) return false;
        char* end = nullptr;
        out[i] = std::strtod(tok.c_str(), &end);
        if (*end != '\0' || !std::isfinite(out[i])) return false;
    }
    std::string extra;
    return !(is >> extra);
}

}  // namespace profile_detail

inline bool VehicleProfileFromKeyValues(const std::vector<KeyValue>& kvs, const std::string& label,
                                        const std::string& text, VehicleProfile* out, std::string* err) {
    using profile_detail::Cat;
    using profile_detail::OneOf;
    VehicleProfile p;
    std::map<std::string, int> line;
    std::map<std::string, std::map<std::string, std::string>> sensors;  // id -> field -> value
    std::vector<std::string> sensor_order;
    std::set<std::string> aid_seen;
    for (const KeyValue& kv : kvs) {
        const std::string where = label + ":" + std::to_string(kv.line) + ": " + kv.key;
        line[kv.key] = kv.line;
        const std::string& k = kv.key;
        const std::string& v = kv.value;
        if (k == "schema") {
            if (v != std::to_string(kVehicleProfileSchema)) {
                *err = Cat(where, ": schema ", v, ", this build reads schema ", std::to_string(kVehicleProfileSchema));
                return false;
            }
        } else if (k == "name") {
            p.name = v;
        } else if (k == "kind") {
            p.kind = v;
        } else if (k == "px4_airframe") {
            p.px4_airframe = v;
        } else if (k == "sim_model") {
            p.sim_model = v;
        } else if (k == "filter_config") {
            p.filter_config = v;
        } else if (k == "description") {
            p.description = v;
        } else if (k == "note") {
            p.note = v;
        } else if (k.rfind("filter.", 0) == 0) {
            p.filter_overrides.emplace_back(k.substr(7), v);
        } else if (k.rfind("aiding.", 0) == 0) {
            const std::string a = k.substr(7);
            if (v != "true" && v != "false") {
                *err = Cat(where, ": must be true or false (is '", v, "')");
                return false;
            }
            const bool on = v == "true";
            if (a == "agl")
                p.aiding.agl = on;
            else if (a == "baro")
                p.aiding.baro = on;
            else if (a == "mag")
                p.aiding.mag = on;
            else if (a == "zupt")
                p.aiding.zupt = on;
            else {
                *err = where + ": unknown aiding (known: agl, baro, mag, zupt)";
                return false;
            }
            aid_seen.insert(a);
        } else if (k.rfind("sensor.", 0) == 0) {
            const auto d1 = k.find('.', 7);
            if (d1 == std::string::npos || d1 == 7 || k.find('.', d1 + 1) != std::string::npos) {
                *err = where + ": expected sensor.<id>.<field>";
                return false;
            }
            const std::string id = k.substr(7, d1 - 7), field = k.substr(d1 + 1);
            if (!OneOf(field, {"kind", "topic", "frame_id", "mount_xyz_m", "mount_rpy_deg", "mount_source", "gdn_input",
                               "noise", "sim", "note"})) {
                *err = Cat(where, ": unknown sensor field '", field, "'");
                return false;
            }
            if (!sensors.count(id)) sensor_order.push_back(id);
            sensors[id][field] = v;
        } else {
            *err = where + ": unknown key";
            return false;
        }
    }
    for (const char* k : {"schema", "name", "kind", "px4_airframe", "sim_model", "filter_config"})
        if (!line.count(k)) {
            *err = label + ": missing " + k;
            return false;
        }
    if (!OneOf(p.kind, {"multirotor", "vtol", "fixedwing"})) {
        *err = label + ":" + std::to_string(line["kind"]) + ": kind must be one of multirotor, vtol, fixedwing";
        return false;
    }
    for (const char* a : {"agl", "baro", "mag", "zupt"})
        if (!aid_seen.count(a)) {
            *err = label + ": missing aiding." + a;
            return false;
        }
    if (sensors.empty()) {
        *err = label + ": no sensors";
        return false;
    }
    std::map<std::string, std::vector<std::string>> by_input;
    for (const std::string& id : sensor_order) {
        auto& f = sensors[id];
        const std::string w = Cat(label, ": sensor.", id);
        for (const char* req :
             {"kind", "topic", "frame_id", "mount_xyz_m", "mount_rpy_deg", "mount_source", "gdn_input"})
            if (!f.count(req)) {
                *err = w + ": missing " + req;
                return false;
            }
        VehicleSensor s;
        s.id = id;
        s.kind = f["kind"];
        s.topic = f["topic"];
        s.frame_id = f["frame_id"];
        s.mount_source = f["mount_source"];
        s.gdn_input = f["gdn_input"];
        s.noise = f.count("noise") ? f["noise"] : "";
        s.sim = f.count("sim") ? f["sim"] : "";
        s.note = f.count("note") ? f["note"] : "";
        const auto at = [&](const char* field) {
            return Cat(label, ":", std::to_string(line[Cat("sensor.", id, ".", field)]), ": sensor.", id, ".", field);
        };
        if (!OneOf(s.kind, {"imu", "lidar", "camera", "range", "radar_altimeter", "baro", "mag"})) {
            *err = at("kind") + ": kind must be one of imu, lidar, camera, range, radar_altimeter, baro, mag";
            return false;
        }
        if (!OneOf(s.gdn_input, {"imu", "agl", "baro", "mag", "none"})) {
            *err = at("gdn_input") + ": gdn_input must be one of imu, agl, baro, mag, none";
            return false;
        }
        if (!OneOf(s.mount_source, {"placeholder", "measured"})) {
            *err = at("mount_source") + ": mount_source must be placeholder or measured";
            return false;
        }
        if (!profile_detail::ParseVec3(f["mount_xyz_m"], s.xyz)) {
            *err = at("mount_xyz_m") + ": three numbers expected (is '" + f["mount_xyz_m"] + "')";
            return false;
        }
        if (!profile_detail::ParseVec3(f["mount_rpy_deg"], s.rpy_deg)) {
            *err = at("mount_rpy_deg") + ": three numbers expected (is '" + f["mount_rpy_deg"] + "')";
            return false;
        }
        by_input[s.gdn_input].push_back(id);
        p.sensors.push_back(s);
    }
    // Each GDN input from exactly one sensor; an aiding switched on needs its sensor.
    const std::pair<const char*, bool> inputs[] = {
        {"imu", true}, {"agl", p.aiding.agl}, {"baro", p.aiding.baro}, {"mag", p.aiding.mag}};
    for (const auto& [inp, needed] : inputs) {
        const auto& ids = by_input[inp];
        if (ids.size() > 1) {
            std::string list;
            for (const std::string& id : ids) list += (list.empty() ? "" : ", ") + id;
            *err = Cat(label, ": gdn_input ", inp, " from more than one sensor (", list, ")");
            return false;
        }
        if (ids.empty() && needed) {
            *err = label + ": no sensor has gdn_input " + inp +
                   (std::string(inp) == "imu" ? "" : std::string(", but aiding.") + inp + " is true");
            return false;
        }
    }
    p.fingerprint = ProfileFingerprint(kvs);
    p.text = text;
    *out = p;
    return true;
}

// From the text of a profile (a file's, or the one a run log carries).
inline bool ParseVehicleProfile(const std::string& text, const std::string& label, VehicleProfile* out,
                                std::string* err) {
    std::istringstream is(text);
    std::vector<KeyValue> kvs;
    return ReadKeyValuesFrom(is, label, &kvs, err) && VehicleProfileFromKeyValues(kvs, label, text, out, err);
}

inline bool LoadVehicleProfile(const std::string& path, VehicleProfile* out, std::string* err) {
    std::ifstream f(path);
    if (!f) {
        *err = "cannot open " + path;
        return false;
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return ParseVehicleProfile(ss.str(), path, out, err);
}

// The profile's filter overrides on top of a loaded filter config. Unknown keys and bad
// values are refused (SetFilterValue), then the result is validated.
inline bool ApplyProfileFilter(const VehicleProfile& p, FilterConfig* fc, std::string* err) {
    for (const auto& [k, v] : p.filter_overrides)
        if (!SetFilterValue(fc, k, v, err)) {
            *err = "vehicle profile " + p.name + ": filter." + k + ": " + *err;
            return false;
        }
    if (!ValidateFilterConfig(*fc, err)) {
        *err = "vehicle profile " + p.name + ": " + *err;
        return false;
    }
    return true;
}

inline std::string DescribeAiding(const Aiding& a) {
    std::string s;
    const std::pair<const char*, bool> all[] = {{"agl", a.agl}, {"baro", a.baro}, {"mag", a.mag}, {"zupt", a.zupt}};
    for (const auto& [n, on] : all) s += std::string(s.empty() ? "" : ", ") + n + (on ? " on" : " off");
    return s;
}

// One line for logs: name, kind, fingerprint, aiding, sensors.
inline std::string DescribeVehicleProfile(const VehicleProfile& p, const std::string& path) {
    int placeholders = 0;
    for (const VehicleSensor& s : p.sensors) placeholders += s.mount_source == "placeholder";
    return p.name + " (" + p.kind + ", " + path + "), fingerprint " + p.fingerprint + "; aiding " +
           DescribeAiding(p.aiding) + "; " + std::to_string(p.sensors.size()) + " sensors, " +
           std::to_string(placeholders) + " with placeholder mounts";
}

}  // namespace gdn
