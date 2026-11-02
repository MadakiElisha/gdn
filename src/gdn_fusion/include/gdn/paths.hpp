#pragma once
// Resolve repo-relative data paths without hard-coding a home directory.
// GDN_ROOT is exported by env.sh; an absolute path is returned unchanged.
#include <cstdlib>
#include <string>

namespace gdn {

inline std::string DataPath(const std::string& rel) {
    if (!rel.empty() && rel.front() == '/') return rel;
    const char* root = std::getenv("GDN_ROOT");
    return std::string(root && *root ? root : ".") + "/" + rel;
}

inline constexpr const char* kDefaultMap = "data/maps/terrain_db_sigma4.bin";

}  // namespace gdn
