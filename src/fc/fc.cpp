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
#include "fc/stages.hpp"

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

    Vec3 unit_vec_from_center = {
        .x = cos(lat_origin) * cos(long_origin),
        .y = cos(lat_origin) * sin(long_origin),
        .z = sin(lat_origin)
    };
    Vec3 up = {0, 0, 1};
    Vec3 rotation_axis = up.cross(unit_vec_from_center);
    Vec3 rot_axis_u = rotation_axis / rotation_axis.norm();
    double half_theta = acos(sin(lat_origin)) / 2;
    out.q_origin = {
        .w = cos(half_theta),
        .x = sin(half_theta) * rot_axis_u.x,
        .y = sin(half_theta) * rot_axis_u.y,
        .z = sin(half_theta) * rot_axis_u.z
    };

    // determine rocket dependent quantities and things
    double delta_v_first_stage = r.props.stages[0].isp * g0 * log((r.props.stages[0].m_dry + r.props.stages[0].m_fuel) / r.props.stages[0].m_dry ); // rocket equation yay

    // launch azimuth
    double delta_long = long_target - long_origin;
    double launch_azimuth = atan2(
        sin(delta_long) * cos(lat_target),
        cos(lat_origin) * sin(lat_target) - sin(lat_origin) * cos(lat_target) * cos(delta_long)
    );
    if (launch_azimuth < 0) launch_azimuth += 2.0 * M_PI;
    out.launch_asimuth = launch_azimuth;

    // stage 1 burn time
    out.stage_burn_time[0] = r.props.stages[0].m_fuel / r.props.stages[0].max_mass_flow_rate();

    // stage 2 burn time
    out.stage_burn_time[1] = r.props.stages[1].m_fuel / r.props.stages[1].max_mass_flow_rate();

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
    cs.att = cs.is.q_origin; // set initial orientation
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
    // position and velocity 

    // determine true acceleration from gravity based on past position state
    double r_norm = cs.r.norm();
    Vec3 g = cs.r * (-1.0 * GM_EARTH / (r_norm * r_norm * r_norm));
    cs.a = g + cs.a_inertial;

    // add delta v based on a to current v
    cs.v = cs.v + cs.a * cs.dt;

    // add delta r based on v to current r
    cs.r = cs.r + cs.v * cs.dt;

    // now for attitude estimation

    // q_dot = 0.5 * Ω(w) * q
    Mat4 omega_w = {{
        {     0.0, -cs.w.x, -cs.w.y, -cs.w.z },
        {  cs.w.x,     0.0,  cs.w.z, -cs.w.y },
        {  cs.w.y, -cs.w.z,     0.0,  cs.w.x },
        {  cs.w.z,  cs.w.y, -cs.w.x,     0.0 },
    }};

    // integrate quaternion rate
    Quat q_dot = omega_w * cs.att;
    cs.att.w += 0.5 * q_dot.w * cs.dt;
    cs.att.x += 0.5 * q_dot.x * cs.dt;
    cs.att.y += 0.5 * q_dot.y * cs.dt;
    cs.att.z += 0.5 * q_dot.z * cs.dt;

    // normalize quaternion bc computer shit
    cs.att = cs.att.normalize();
}

Quat FlightController::quat_from_target_pos(Vec3 position, Vec3 target) {
    // unit vector to taget point
    Vec3 u = (target - position).normalized();

    
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
        if (cs.time - countdown_start >= HOLD_DURATION) {
            cs.stage = STAGE_1_POWERED;
            r.light_engine();
        }
        break;
    case STAGE_1_POWERED: {
        s1_powered(r, cs);
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
