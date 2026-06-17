#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <random>
#include <vector>
#include <thread>
#include <chrono>
#include "fc.hpp"
#include "imu.hpp"
#include "types.hpp"
#include "sim/rocket.hpp"

Vec3 INS::read_INS_acc(const Rocket& r) {
    Vec3 specific_force_eci = r.a - gravity_eci(r.r);
    return add_noise(specific_force_eci, acc_stddev);
}

Vec3 INS::read_INS_gyr(const Rocket& r) {
    return add_noise(r.w, gyr_stddev);
}


///////////////////////////////////////////////////////////////////////////////////////////////
// startup                                                                                   //
///////////////////////////////////////////////////////////////////////////////////////////////
FCInitState FlightController::create_target_trajectory(double lat_target, double long_target, Rocket& r) {
    FCInitState out;

    // convert to radians
    lat_target = lat_target * M_PI / 180.0;
    long_target = long_target * M_PI / 180.0;

    // derive lat and longitude from starting position of rocket on planet
    Vec3 p = r.r;
    double radius = p.norm();
    double lat_origin  = asin(p.z / radius);
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
    for (const Stage& stg : r.props.stages) {
        m0 += stg.m_dry + stg.m_fuel;
    }
    const Stage s1 = r.props.stages[0];
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

// init
void FlightController::init(Rocket& r, double current_time) {
    cs = {};
    cs.stage = ARMED;
    cs.time = current_time;
    cs.is = create_target_trajectory(38.9072, -77.0369, r);
    cs.r = cs.is.r_origin; // set initial r to starting r
    countdown_start = current_time;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// helper functions                                                                          //
///////////////////////////////////////////////////////////////////////////////////////////////

// aquires new data from the sim
void FlightController::pull_new_data(const Rocket& r, double current_time) {
    cs.a_inertial = ins.read_INS_acc(r);
    cs.w = ins.read_INS_gyr(r);
    cs.dt = current_time - cs.time;
    cs.time = current_time; // this must happen after cs.dt or else cs.dt will be 0 :)
}

// estimate state of the rocket in flight at the current moment
void FlightController::estimate_state() {
    // determine true acceleration from gravity based on past position state
    double r_norm = cs.r.norm();
    Vec3 g = cs.r * (-1.0 * GM_EARTH / (r_norm * r_norm * r_norm));
    cs.a = g + cs.a_inertial;

    // add delta v based on a to current v
    cs.v = cs.v + cs.a * cs.dt;

    // add delta r based on v to current r
    cs.r = cs.r + cs.v * cs.dt;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// flight controller loop                                                                    //
///////////////////////////////////////////////////////////////////////////////////////////////

// runs on its own thread alongside the sim
void FlightController::flight_controller_process(Rocket& r, double current_time) {

    ////////////////////////////
    // control inside         //
    ////////////////////////////

    // get latest data from the INS and advance the clock
    pull_new_data(r, current_time);

    // estimate state based on pulled data
    estimate_state();

    switch (cs.stage) {
    case STANDBY:
        break;
    case ARMED:
        if (cs.time - countdown_start >= HOLD_DURATION) cs.stage = STAGE_1_POWERED;
        break;
    case STAGE_1_POWERED: {
        static bool engine_lit = false;
        if (!engine_lit) {
            r.light_engine();
            engine_lit = true;
        }

        static double start = cs.time;
        static bool is_separated = false;
        if (cs.time - start > 130 && !is_separated) {
            r.advance_stage();
            is_separated = true;
            r.light_engine();
        }

        const double turn_time = 3.0;
        static double turn_time_start = cs.time;
        if ((cs.time - turn_time_start) < turn_time) {
            double half = (0.3 * M_PI / 180.0) / 2.0;
            r.set_engine_orientation( {std::cos(half), 0.0, std::sin(half), 0.0} );
        }
        else if ((cs.time - turn_time_start) < 2.0 * turn_time) {
            double half = (-0.3 * M_PI / 180.0) / 2.0;
            r.set_engine_orientation( {std::cos(half), 0.0, std::sin(half), 0.0} );
        }
        else r.set_engine_orientation({1, 0, 0, 0});
        break;
    }
    case STAGE_2_UNPOWERED:
        break;
    case STAGE_2_POWERED:
        break;
    case PAYLOAD_DEPLOY:
        break;
    }

}
