#pragma once
// R1.10: the filter's settings have no values in the code; they come from a config file
// (gdn/filter_config.hpp). An unset setting is NaN, so a filter built from a config that
// wasn't loaded goes visibly wrong instead of quietly running on a forgotten number.
#include <limits>

namespace gdn {
inline constexpr double kUnset = std::numeric_limits<double>::quiet_NaN();
inline constexpr int kUnsetInt = -1;
}  // namespace gdn
