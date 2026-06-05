// pre launch targeting system
#include "types.hpp"
#include "sim.hpp"

struct TargetingOutput {
    // NOTE: all angles in radians
    double lat_target, long_target; // latitude and longitude of target
    double lat_origin, long_origin; // latitude and longitude of origin
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

    // using range angle equation
    out.range_angle = acos( sin(out.lat_origin) * sin(out.lat_target) + cos(out.lat_origin) * cos(out.lat_target) * cos(out.long_target - out.long_origin) );

    // launch asimuth equation
    out.launch_asimuth = acos( (sin(out.lat_target) - sin(out.lat_origin) * cos(out.range_angle)) / (cos(out.lat_origin) * sin(out.range_angle)) );

}