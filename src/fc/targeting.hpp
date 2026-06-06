// pre launch targeting system
#include "types.hpp"
#include "sim/sim.hpp"
#include "constants.hpp"

struct TargetingOutput {
    // NOTE: all angles in radians
    double lat_target, long_target; // latitude and longitude of target
    double lat_origin, long_origin; // latitude and longitude of origin
    Vec3 r_origin, r_target;
    double range_angle; // range angle
    double launch_asimuth; // launch asimuth
    double Q_bo; // trajectory parameter
    double phi_bo_high, phi_bo_low; // path angle
    double TOF;

    Vec3 V_burnout;
    Vec3 r_burnout;
};

TargetingOutput create_target_trajectory(double lat_target, double long_target, sim::Sim& sim) {
    TargetingOutput out;

    // convert to radians
    out.lat_target = lat_target * M_PI / 180.0;
    out.long_target = long_target * M_PI / 180.0;

    // derive lat and longitude from position of rocket on planet
    Vec3 p = sim.get_rocket_pos();
    double r = p.norm();
    out.lat_origin  = asin(p.z / r);
    out.long_origin = atan2(p.y, p.x);

    // get ECF coordinates from lat and long
    out.r_origin = {
        EARTH_RADIUS_M * cos(out.lat_origin) * cos(out.long_origin),
        EARTH_RADIUS_M * cos(out.lat_origin) * sin(out.long_origin),
        EARTH_RADIUS_M * sin(out.lat_origin)
    };
    out.r_target = {
        EARTH_RADIUS_M * cos(out.lat_target) * cos(out.long_target),
        EARTH_RADIUS_M * cos(out.lat_target) * sin(out.long_target),
        EARTH_RADIUS_M * sin(out.lat_target)
    };
}