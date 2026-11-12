#pragma once
// Reader for the flat "key: value" files in config/ (IMU models, filter configuration).
// ROS-free. '#' starts a comment, blank lines are skipped, values may be quoted.
// A key given twice is an error: which of the two counts would be a guess.
#include <fstream>
#include <set>
#include <string>
#include <vector>

namespace gdn {

struct KeyValue {
    std::string key, value;
    int line = 0;
};

inline std::string TrimKv(const std::string& s) {
    const auto a = s.find_first_not_of(" \t\r\"'"), b = s.find_last_not_of(" \t\r\"'");
    return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
}

inline bool ReadKeyValues(const std::string& path, std::vector<KeyValue>* out, std::string* err) {
    std::ifstream f(path);
    if (!f) {
        *err = "cannot open " + path;
        return false;
    }
    out->clear();
    std::set<std::string> seen;
    std::string line;
    int ln = 0;
    while (std::getline(f, line)) {
        ++ln;
        if (const auto h = line.find('#'); h != std::string::npos) line.erase(h);
        if (TrimKv(line).empty()) continue;
        const auto c = line.find(':');
        const std::string where = path + ":" + std::to_string(ln) + ": ";
        if (c == std::string::npos) {
            *err = where + "expected key: value";
            return false;
        }
        KeyValue kv{TrimKv(line.substr(0, c)), TrimKv(line.substr(c + 1)), ln};
        if (!seen.insert(kv.key).second) {
            *err = where + "key '" + kv.key + "' given twice";
            return false;
        }
        out->push_back(kv);
    }
    return true;
}

}  // namespace gdn
