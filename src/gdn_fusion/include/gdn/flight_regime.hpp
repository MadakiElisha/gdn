#pragma once
// Flight regime from the autopilot (roadmap R2.5b). ROS-free.
//
// PX4 knows whether it's on the ground and how it's flying; an IMU alone can't tell a
// steady hover from being parked (TEST-032). So GDN takes the regime from PX4's
// vehicle_land_detected and vehicle_status:
//
//   ground      landed (the land detector's last stage), or disarmed
//   transition  a VTOL changing between rotary and fixed wing
//   cruise      airborne, flying as a fixed wing
//   hover       airborne, flying on rotors (a multirotor's whole flight)
//   unknown     before both messages have arrived, or with no autopilot (the bench):
//               GDN then decides on the ground from the IMU, as before (R1.2)
//
// The vehicle profile's rules (gdn/vehicle_profile.hpp: regime.<regime>.<aiding>) say
// which aiding is used in each regime; zero-velocity updates only ever on the ground.
#include <cstdint>
#include <string>

namespace gdn {

enum class Regime : std::uint8_t { kUnknown = 0, kGround = 1, kHover = 2, kTransition = 3, kCruise = 4 };

inline const char* RegimeName(Regime r) {
    switch (r) {
        case Regime::kGround:
            return "ground";
        case Regime::kHover:
            return "hover";
        case Regime::kTransition:
            return "transition";
        case Regime::kCruise:
            return "cruise";
        case Regime::kUnknown:
            break;
    }
    return "unknown";
}

// PX4 1.16 values (px4_msgs VehicleStatus).
inline constexpr std::uint8_t kPx4ArmingArmed = 2;
inline constexpr std::uint8_t kPx4VehicleFixedWing = 2;

inline Regime RegimeFromPx4(std::uint8_t arming_state, std::uint8_t vehicle_type, bool in_transition, bool landed) {
    if (landed || arming_state != kPx4ArmingArmed) return Regime::kGround;
    if (in_transition) return Regime::kTransition;
    return vehicle_type == kPx4VehicleFixedWing ? Regime::kCruise : Regime::kHover;
}

// Which aiding a regime allows. ZUPT is the ground's alone: velocity is never zero in
// flight, so the airborne regimes have no switch for it.
struct RegimeAid {
    bool agl = true, baro = true, mag = true;
};

struct RegimeRules {
    RegimeAid ground{false, true, true};  // the altimeter reads under its minimum range when parked
    bool ground_zupt = true;
    RegimeAid hover, transition, cruise;

    const RegimeAid* For(Regime r) const {  // nullptr: unknown, no rule
        switch (r) {
            case Regime::kGround:
                return &ground;
            case Regime::kHover:
                return &hover;
            case Regime::kTransition:
                return &transition;
            case Regime::kCruise:
                return &cruise;
            case Regime::kUnknown:
                break;
        }
        return nullptr;
    }
};

}  // namespace gdn
