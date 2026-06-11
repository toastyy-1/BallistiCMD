// pre launch targeting system
#include "types.hpp"
#include "sim/sim.hpp"
#include "constants.hpp"

struct Initial_States {
    Vec3 r_origin, r_target;

    Vec3 v_s1_bo_T;
    Vec3 r_s1_bo_T;
    
    double target_tof;
};

Initial_States create_target_trajectory(double lat_target, double long_target, sim::Sim& sim) {
    Initial_States out;

    // convert to radians
    lat_target = lat_target * M_PI / 180.0;
    long_target = long_target * M_PI / 180.0;

    // derive lat and longitude from starting position of rocket on planet
    Vec3 p = sim.get_rocket_pos();
    double r = p.norm();
    double lat_origin  = asin(p.z / r);
    double long_origin = atan2(p.y, p.x);

    // get ECF coordinates from lat and long
    out.r_origin = {
        EARTH_RADIUS_M * cos(lat_origin) * cos(long_origin),
        EARTH_RADIUS_M * cos(lat_origin) * sin(long_origin),
        EARTH_RADIUS_M * sin(lat_origin)
    };
    out.r_target = {
        EARTH_RADIUS_M * cos(lat_target) * cos(long_target),
        EARTH_RADIUS_M * cos(lat_target) * sin(long_target),
        EARTH_RADIUS_M * sin(lat_target)
    };

    // determine range angle
    double range_angle = acos(
                            sin(lat_origin) * sin(lat_target) +
                            cos(lat_origin) * cos(lat_target) * cos(long_target - long_origin)
                        );
    
    // set target time of flight (in seconds)
    out.target_tof = 900;

    // return all our calculated stuff yay!
    return out;
}