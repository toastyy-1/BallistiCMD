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
    sim::State st = sim.get_state();
    Vec3 p = st.r;
    double r = p.norm();
    double lat_origin  = asin(p.z / r);
    double long_origin = atan2(p.y, p.x);

    // get ECI coordinates from lat and long
    out.r_origin = {
        EARTH_RADIUS * cos(lat_origin) * cos(long_origin),
        EARTH_RADIUS * cos(lat_origin) * sin(long_origin),
        EARTH_RADIUS * sin(lat_origin)
    };
    out.r_target = {
        EARTH_RADIUS * cos(lat_target) * cos(long_target),
        EARTH_RADIUS * cos(lat_target) * sin(long_target),
        EARTH_RADIUS * sin(lat_target)
    };

    // determine trajectory parameters
    // https://cmp.felk.cvut.cz/~kukelova/pajdla/Bate,%20Mueller,%20and%20White%20-%20Fundamentals%20of%20Astrodynamics.pdf
    // page 277
    double range_angle = acos(out.r_origin.dot(out.r_target) / (EARTH_RADIUS * EARTH_RADIUS));

    double h_burnout = 80e3;
    double r_burnout = EARTH_RADIUS + h_burnout;

    double sin_half = sin(range_angle / 2.0);
    double v_burnout = sqrt(2.0 * GM_EARTH * sin_half / (r_burnout * (1.0 + sin_half)));

    double fpa = 0.25 * (M_PI - range_angle); // flight path angle at burnout
    double sma = 0.5 * r_burnout * (1.0 + sin_half);
    double ecc = cos(range_angle / 2.0) / (1.0 + sin_half);
    double E_bo = acos(-ecc); // eccentric anomaly at burnout
    out.target_tof = 2.0 * sqrt(sma * sma * sma / GM_EARTH) * (M_PI - (E_bo - ecc * sin(E_bo)));

    // stage 1 burnout
    double m0 = 0;
    for (const Stage& stg : st.stages) {
        m0 += stg.m_dry + stg.m_fuel;
    }
    const Stage& s1 = st.stages[0];
    double t_burn = s1.m_fuel * s1.exhaust_velocity() / s1.max_thrust;
    double v_s1_bo = s1.exhaust_velocity() * log(m0 / (m0 - s1.m_fuel)) - g0 * t_burn;
    double h_s1_bo = 0.5 * v_s1_bo * t_burn;

    Vec3 up = out.r_origin / EARTH_RADIUS;
    Vec3 downrange = (out.r_target - up * out.r_target.dot(up)).normalized();
    out.r_s1_bo_T = up * (EARTH_RADIUS + h_s1_bo);
    out.v_s1_bo_T = (downrange * cos(fpa) + up * sin(fpa)) * v_s1_bo;

    // return all our calculated stuff yay!
    return out;
}