#pragma once
// Filter configuration file (R1.10). ROS-free.
//
// Every tuning value of the filter lives in a config file (config/filter/default.yaml);
// the code has none (EskfConfig and GroundConfig start out unset, NaN). The node and the
// MC load a file, may override single values (node parameters, mc_campaign --set), and
// print the file's name with a fingerprint of the values they ran with, so a result can
// be traced to its exact configuration.
//
// The file is flat "key: value" (gdn/kv_file.hpp). Every key in ConfigFields() is
// required, plus "schema" and "name"; "source" is optional. Anything else is an error,
// so a typo can't fall back to a value nobody chose.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <string>
#include <variant>
#include <vector>
#include "gdn/eskf.hpp"
#include "gdn/ground_monitor.hpp"
#include "gdn/kv_file.hpp"

namespace gdn {

inline constexpr const char* kDefaultFilterConfig = "config/filter/default.yaml";
inline constexpr int kFilterConfigSchema = 1;

struct FilterConfig {
    std::string name, source;
    EskfConfig eskf;
    GroundConfig ground;
    // Start-up and node settings outside the filter core.
    double datum_sigma_m = kUnset;       // altitude datum from the first AGL reading, 1 sigma
    double baro_init_sigma_m = kUnset;   // baro bias at its first reading, 1 sigma
    double gap_att_rate_scale = kUnset;  // IMU hole: attitude widened by scale x rate x hole
    double gap_att_floor_deg = kUnset;   //   ... plus this
    double gap_vel_accel = kUnset;       // IMU hole: velocity widened by this x hole
    std::string mag_fusion;              // auto | heading | full | align
};

struct ConfigField {
    enum Rule : std::uint8_t { kPositive, kNonNegative, kAny };
    const char* key;
    std::variant<double*, int*, std::string*> ptr;
    Rule rule;
};

// The table every reader and writer goes through, in file order.
inline std::vector<ConfigField> ConfigFields(FilterConfig* c) {
    using F = ConfigField;
    EskfConfig& e = c->eskf;
    GroundConfig& g = c->ground;
    return {
        {"r_trn", &e.r_trn, F::kPositive},
        {"slope_min", &e.slope_min, F::kNonNegative},
        {"gate_sigma", &e.gate_sigma, F::kPositive},
        {"gate_abs_m", &e.gate_abs_m, F::kPositive},
        {"r_agl", &e.r_agl, F::kPositive},
        {"gate_agl_sigma", &e.gate_agl_sigma, F::kPositive},
        {"gate_agl_abs_m", &e.gate_agl_abs_m, F::kPositive},
        {"datum_sigma_m", &c->datum_sigma_m, F::kPositive},
        {"q_baro_bias", &e.q_baro_bias, F::kNonNegative},
        {"p0_baro_bias", &e.p0_baro_bias, F::kPositive},
        {"r_baro", &e.r_baro, F::kPositive},
        {"gate_baro_sigma", &e.gate_baro_sigma, F::kPositive},
        {"baro_init_sigma_m", &c->baro_init_sigma_m, F::kPositive},
        {"q_accel_nd", &e.q_accel_nd, F::kPositive},
        {"q_gyro_nd", &e.q_gyro_nd, F::kPositive},
        {"q_accel_bias_rw", &e.q_accel_bias_rw, F::kNonNegative},
        {"q_gyro_bias_rw", &e.q_gyro_bias_rw, F::kNonNegative},
        {"still_rate", &e.still_rate, F::kPositive},
        {"static_n", &e.static_n, F::kPositive},
        {"max_gyro_mean", &e.max_gyro_mean, F::kPositive},
        {"max_grav_dev", &e.max_grav_dev, F::kPositive},
        {"max_accel_var", &e.max_accel_var, F::kPositive},
        {"p0_pos", &e.p0_pos, F::kPositive},
        {"p0_vel", &e.p0_vel, F::kPositive},
        {"p0_tilt_sigma_deg", &e.p0_tilt_sigma_deg, F::kPositive},
        {"p0_accel_bias", &e.p0_accel_bias, F::kPositive},
        {"p0_gyro_bias_sigma_deg", &e.p0_gyro_bias_sigma_deg, F::kPositive},
        {"yaw_sigma_transfer_deg", &e.yaw_sigma_transfer_deg, F::kPositive},
        {"yaw_sigma_mag_align_deg", &e.yaw_sigma_mag_align_deg, F::kPositive},
        {"ground_window_s", &g.window_s, F::kPositive},
        {"ground_accel_tol", &g.accel_tol, F::kPositive},
        {"ground_gyro_tol", &g.gyro_tol, F::kPositive},
        {"ground_latch_s", &g.latch_s, F::kPositive},
        {"ground_zupt_period_s", &g.zupt_period_s, F::kPositive},
        {"zupt_sigma_v", &e.zupt_sigma_v, F::kPositive},
        {"gate_zupt_chi2", &e.gate_zupt_chi2, F::kPositive},
        {"mag_fusion", &c->mag_fusion, F::kAny},
        {"q_mag_bias", &e.q_mag_bias, F::kNonNegative},
        {"p0_mag_bias", &e.p0_mag_bias, F::kPositive},
        {"r_mag", &e.r_mag, F::kPositive},
        {"gate_mag_sigma", &e.gate_mag_sigma, F::kPositive},
        {"mag_heading_cal_sigma", &e.mag_heading_cal_sigma, F::kPositive},
        {"mag_auto_norm_tol", &e.mag_auto_norm_tol, F::kPositive},
        {"mag_auto_dip_tol_deg", &e.mag_auto_dip_tol_deg, F::kPositive},
        {"mag_auto_tau", &e.mag_auto_tau, F::kPositive},
        {"mag_auto_hold", &e.mag_auto_hold, F::kNonNegative},
        {"mag_ref_north", &e.mag_ref_north, F::kAny},
        {"mag_ref_east", &e.mag_ref_east, F::kAny},
        {"mag_ref_down", &e.mag_ref_down, F::kAny},
        {"clamp_dv", &e.clamp_dv, F::kPositive},
        {"clamp_dtheta_deg", &e.clamp_dtheta_deg, F::kPositive},
        {"clamp_dva", &e.clamp_dva, F::kPositive},
        {"clamp_dvg_deg", &e.clamp_dvg_deg, F::kPositive},
        {"clamp_dvm", &e.clamp_dvm, F::kPositive},
        {"gap_att_rate_scale", &c->gap_att_rate_scale, F::kNonNegative},
        {"gap_att_floor_deg", &c->gap_att_floor_deg, F::kNonNegative},
        {"gap_vel_accel", &c->gap_vel_accel, F::kNonNegative},
    };
}
inline std::vector<ConfigField> ConfigFields(const FilterConfig& c) {
    return ConfigFields(const_cast<FilterConfig*>(&c));  // read-only use
}

// Shortest text that reads back as exactly the same double.
inline std::string FormatExact(double v) {
    char buf[40];
    for (int prec = 15; prec <= 17; ++prec) {
        std::snprintf(buf, sizeof buf, "%.*g", prec, v);
        if (std::strtod(buf, nullptr) == v) break;
    }
    return buf;
}

inline std::string FieldText(const ConfigField& f) {
    if (const auto* p = std::get_if<double*>(&f.ptr)) return FormatExact(**p);
    if (const auto* n = std::get_if<int*>(&f.ptr)) return std::to_string(**n);
    return *std::get<std::string*>(f.ptr);
}

inline bool ValidateFilterConfig(const FilterConfig& c, std::string* err) {
    for (const ConfigField& f : ConfigFields(c)) {
        const std::string k = f.key;
        double v = 0.0;
        if (const auto* p = std::get_if<double*>(&f.ptr)) {
            v = **p;
            if (!std::isfinite(v)) {
                *err = k + " is not set";
                return false;
            }
        } else if (const auto* n = std::get_if<int*>(&f.ptr)) {
            v = **n;
        } else {
            continue;
        }
        if (f.rule == ConfigField::kPositive && !(v > 0.0)) {
            *err = k + " must be > 0 (is " + FieldText(f) + ")";
            return false;
        }
        if (f.rule == ConfigField::kNonNegative && !(v >= 0.0)) {
            *err = k + " must be >= 0 (is " + FieldText(f) + ")";
            return false;
        }
    }
    if (c.mag_fusion != "auto" && c.mag_fusion != "heading" && c.mag_fusion != "full" && c.mag_fusion != "align") {
        *err = "mag_fusion must be auto, heading, full or align (is '" + c.mag_fusion + "')";
        return false;
    }
    return true;
}

// Sets one value from text. Unknown keys and values that don't parse are refused.
inline bool SetFilterValue(FilterConfig* c, const std::string& key, const std::string& value, std::string* err) {
    for (const ConfigField& f : ConfigFields(c)) {
        if (key != f.key) continue;
        char* end = nullptr;
        if (auto* p = std::get_if<double*>(&f.ptr)) {
            const double v = std::strtod(value.c_str(), &end);
            if (value.empty() || *end != '\0') break;
            **p = v;
        } else if (auto* n = std::get_if<int*>(&f.ptr)) {
            const long v = std::strtol(value.c_str(), &end, 10);
            if (value.empty() || *end != '\0' || v != static_cast<int>(v)) break;
            **n = static_cast<int>(v);
        } else {
            *std::get<std::string*>(f.ptr) = value;
        }
        return true;
    }
    bool known = false;
    for (const ConfigField& f : ConfigFields(c)) known = known || key == f.key;
    *err = known ? "bad value for " + key + ": '" + value + "'" : "unknown key '" + key + "'";
    return false;
}

inline bool LoadFilterConfig(const std::string& path, FilterConfig* out, std::string* err) {
    std::vector<KeyValue> kvs;
    if (!ReadKeyValues(path, &kvs, err)) return false;
    FilterConfig c;
    std::set<std::string> seen;
    for (const KeyValue& kv : kvs) {
        const std::string where = path + ":" + std::to_string(kv.line) + ": ";
        seen.insert(kv.key);
        if (kv.key == "schema") {
            if (kv.value != std::to_string(kFilterConfigSchema)) {
                *err =
                    where + "schema " + kv.value + ", this build reads schema " + std::to_string(kFilterConfigSchema);
                return false;
            }
        } else if (kv.key == "name") {
            c.name = kv.value;
        } else if (kv.key == "source") {
            c.source = kv.value;
        } else if (!SetFilterValue(&c, kv.key, kv.value, err)) {
            *err = where + *err;
            return false;
        }
    }
    std::vector<std::string> need = {"schema", "name"};
    for (const ConfigField& f : ConfigFields(c)) need.emplace_back(f.key);
    for (const std::string& k : need)
        if (!seen.count(k)) {
            *err = path;
            *err += ": missing " + k;
            return false;
        }
    if (!ValidateFilterConfig(c, err)) {
        *err = path + ": " + *err;
        return false;
    }
    *out = c;
    return true;
}

// The values only, one "key=value" per line in table order: what the fingerprint hashes.
inline std::string CanonicalFilterText(const FilterConfig& c) {
    std::string s;
    for (const ConfigField& f : ConfigFields(c)) {
        s += f.key;
        s += "=";
        s += FieldText(f);
        s += "\n";
    }
    return s;
}

// 64-bit FNV-1a of the values, as 12 hex digits. Names, comments and key order don't
// enter it; any change of a value does.
inline std::string FilterFingerprint(const FilterConfig& c) {
    std::uint64_t h = 1469598103934665603ULL;
    for (const unsigned char ch : CanonicalFilterText(c)) {
        h ^= ch;
        h *= 1099511628211ULL;
    }
    char buf[20];
    std::snprintf(buf, sizeof buf, "%012llx", static_cast<unsigned long long>(h >> 16));
    return buf;
}

// "key=value, ..." for every value of `now` that differs from `base`.
inline std::string FilterOverrides(const FilterConfig& base, const FilterConfig& now) {
    const auto fb = ConfigFields(base), fn = ConfigFields(now);
    std::string s;
    for (size_t i = 0; i < fn.size(); ++i) {
        const std::string v = FieldText(fn[i]);
        if (v == FieldText(fb[i])) continue;
        if (!s.empty()) s += ", ";
        s += std::string(fn[i].key) + "=" + v;
    }
    return s;
}

// A loadable file of exactly these values (mc_campaign --dump-config).
inline std::string DumpFilterConfig(const FilterConfig& c, const std::string& note = "") {
    std::string s = "# Filter configuration written by GDN";
    s += note.empty() ? "" : " (" + note + ")";
    s += "; fingerprint " + FilterFingerprint(c) + ".\n";
    s += "# Every value exactly as run. Comments and units: config/filter/default.yaml.\n";
    s += "schema: " + std::to_string(kFilterConfigSchema) + "\n";
    s += "name: " + c.name + "\n";
    if (!c.source.empty()) s += "source: " + c.source + "\n";
    for (const ConfigField& f : ConfigFields(c)) s += std::string(f.key) + ": " + FieldText(f) + "\n";
    return s;
}

// One line for logs and reports.
inline std::string DescribeFilterConfig(const FilterConfig& c, const std::string& path, const FilterConfig* base) {
    std::string s = c.name + " (" + path + "), fingerprint " + FilterFingerprint(c);
    const std::string o = base ? FilterOverrides(*base, c) : "";
    if (!o.empty()) s += "; overrides " + o;
    return s;
}

}  // namespace gdn
