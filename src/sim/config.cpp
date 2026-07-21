#include "sim/config.hpp"
#include "tomlplusplus/toml.hpp"
#include <iostream>
#include <stdexcept>

SimConfig load_sim_config(const std::string& path) {
    // parse the toml file
    toml::table tbl;
    try {
        tbl = toml::parse_file(path);
    } catch (const toml::parse_error& err) {
        std::cerr << "config error: could not parse '" << path << "': " << err.description() << "\n";
    }

    SimConfig cfg;

    cfg.time_step = tbl["time_step"].value_or(cfg.time_step);

    // one launch entry per rocket, each corresponding to its own origin and target
    auto launches = tbl["launch"].as_array();
    if (launches) {
        for (size_t i = 0; i < launches->size(); i++) {
            const toml::table& l = *(*launches)[i].as_table();
            LaunchTarget launch;

            launch.origin_lat = l["origin_lat"].value_or(0.0);
            launch.origin_lon = l["origin_lon"].value_or(0.0);
            launch.target_lat = l["target_lat"].value_or(0.0);
            launch.target_lon = l["target_lon"].value_or(0.0);
            cfg.launches.push_back(launch);
        }
    }

    if (cfg.launches.empty()) {
        std::cerr << "config error: '" << path << "' must define at least one [[launch]] entry\n";
    }

    // rocket body properties
    auto rocket = tbl["rocket"];
    cfg.rocket_props.radius     = rocket["radius"].value_or(cfg.rocket_props.radius);
    cfg.rocket_props.Cd         = rocket["drag_coefficient"].value_or(cfg.rocket_props.Cd);

    // must have exactly ROCKET_NUM_STAGES entries
    auto stages = rocket["stage"].as_array();
    if (!stages || stages->size() != ROCKET_NUM_STAGES) {
        std::cerr << "config error: '" << path << "' must define exactly "
                  << ROCKET_NUM_STAGES << " [[rocket.stage]] entries\n";
    }

    for (size_t i = 0; i < stages->size(); i++) {
        const toml::table& st = *(*stages)[i].as_table();
        Stage& s = cfg.rocket_props.stages[i];

        s.id                    = st["id"].value_or(0.0);
        s.m_dry                 = st["dry_mass"].value_or(0.0);
        s.m_fuel                = st["fuel_mass"].value_or(0.0);
        s.isp                   = st["isp"].value_or(0.0);
        s.tip_to_end_length     = st["length"].value_or(0.0);
        s.CoM_dist              = st["com_distance"].value_or(0.0);
        s.max_thrust            = st["max_thrust"].value_or(0.0);
        s.engine_distance       = st["engine_distance"].value_or(0.0);
        s.engine_gimball_range  = st["gimbal_range_deg"].value_or(0.0);

        if (auto rcs = st["rcs_max_moment"].as_array(); rcs && rcs->size() == 3) {
            s.rcs_max_capable_moment = {
                (*rcs)[0].value_or(0.0),
                (*rcs)[1].value_or(0.0),
                (*rcs)[2].value_or(0.0),
            };
        } else if (st.contains("rcs_max_moment")) {
            std::cerr << "config error: stage " << i
                      << " rcs_max_moment must be a 3-element array, ignoring\n";
        }
    }

    return cfg;
}
