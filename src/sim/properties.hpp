#pragma once

#include "constants.hpp"
#include <array>

// number of stages on the rocket
inline constexpr int ROCKET_NUM_STAGES = 3;

struct Stage {
    double id;
    double m_dry;                   // dry mass
    double m_fuel;                  // fuel mass
    double isp;                     // specific impulse (s)
    double tip_to_end_length;       // m
    double CoM_dist;                // dist of center of mass from front edge of the stage
    double max_thrust;              // rated (max) motor thrust
    double thrust;                  // current commanded thrust
    double engine_distance;         // distance of engine from leading edge
    double engine_gimball_range;    // rad
    Vec3 rcs_max_capable_moment;    // n-m torque that RCS system for that stage can apply about axes along CoM (set 0 if no rcs)

    double exhaust_velocity() const { return isp * g0; }
    double mass_flow_rate() const { return isp > 0 ? thrust / exhaust_velocity() : 0.0; }
    double max_mass_flow_rate() const { return isp > 0 ? max_thrust / exhaust_velocity() : 0.0; }
};

// config and geometry of rocket whao
struct RocketProps {
    double radius = 0;  // hull radius for the solid-cylinder inertia model (m)
    double Cd = 0.0;    // drag coefficient
    std::array<Stage, ROCKET_NUM_STAGES> stages = {{
        { .id = 1 },
        { .id = 2 },
        { .id = 3 },
    }};
};
