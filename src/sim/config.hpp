#pragma once
#include <string>
#include <vector>
#include "sim/properties.hpp"

// one rocket: where it starts and where it's aimed
struct LaunchTarget {
    double origin_lat = 0.0, origin_lon = 0.0;
    double target_lat = 0.0, target_lon = 0.0;
};

struct SimConfig {
    std::vector<LaunchTarget> launches;
    RocketProps rocket_props;
    double time_step = 0.01; // seconds
    double step_delay = 0.001;
};

// reads the sims config file
SimConfig load_sim_config(const std::string& path);
