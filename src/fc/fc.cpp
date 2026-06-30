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
    return add_noise(r.a - gravity_eci(r.r), acc_stddev);
}

Vec3 INS::read_INS_gyr(const Rocket& r) {
    return add_noise(r.w, gyr_stddev);
}

Vec3 INS::read_INS_grav(const Rocket& r) {
    return gravity_eci(r.r);
}


///////////////////////////////////////////////////////////////////////////////////////////////
// startup                                                                                   //
///////////////////////////////////////////////////////////////////////////////////////////////

FlightController::FlightController(Rocket& r, double current_time) : props(r.props) {
    cs.stage = STANDBY;
}

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

    // launch azimuth
    double delta_long = long_target - long_origin;
    double launch_azimuth = atan2(
        sin(delta_long) * cos(lat_target),
        cos(lat_origin) * sin(lat_target) - sin(lat_origin) * cos(lat_target) * cos(delta_long)
    );
    if (launch_azimuth < 0) launch_azimuth += 2.0 * M_PI;
    out.launch_asimuth = launch_azimuth;

    // stage 1 burn time
    out.stage_burn_time[0] = props.stages[0].m_fuel / props.stages[0].max_mass_flow_rate();

    // stage 2 burn time
    out.stage_burn_time[1] = props.stages[1].m_fuel / props.stages[1].max_mass_flow_rate();

    // stage 3 burn time
    out.stage_burn_time[2] = props.stages[2].m_fuel / props.stages[2].max_mass_flow_rate();

    // return all our calculated stuff yay!
    return out;
}

// init
void FlightController::init(Rocket& r, double current_time) {
    cs = {};
    cs.stage = ARMED;
    cs.time = current_time;

    double tgt_lat = asin(r.start_state.target_r_eci.z / r.start_state.target_r_eci.norm()) * RAD_TO_DEG;
    double tgt_long = atan2(r.start_state.target_r_eci.y, r.start_state.target_r_eci.x) * RAD_TO_DEG;
    cs.is = create_target_trajectory(tgt_lat, tgt_long, r);

    cs.r = cs.is.r_origin; // set initial r to starting r
    cs.att = cs.is.q_origin; // set initial orientation
    cs.target_att = cs.is.q_origin;
    countdown_start = current_time;
}

///////////////////////////////////////////////////////////////////////////////////////////////
// helper functions                                                                          //
///////////////////////////////////////////////////////////////////////////////////////////////

// aquires new data from the sim
void FlightController::pull_new_data(const Rocket& r, double current_time) {
    cs.a_inertial = ins.read_INS_acc(r);
    cs.g = ins.read_INS_grav(r);
    cs.a = cs.a_inertial + cs.g;
    cs.w = ins.read_INS_gyr(r);
    cs.dt = current_time - cs.time;
    cs.time = current_time; // this must happen after cs.dt or else cs.dt will be 0 :)
}

// estimate state of the rocket in flight at the current moment
void FlightController::estimate_state() {
    // position and velocity 

    // acceleration is assumed to have been determined before this by reading from the INS

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

// returns quaternion that points in direction of
Quat FlightController::quat_from_vec(Vec3 u) {
    u = u.normalized();

    // quaternion of the shortest arc from nose to u
    Quat q = {
        .w = 1.0 + u.z,
        .x = -u.y,
        .y =  u.x,
        .z =  0.0,
    };
    return q.normalize();
}

// sets the engine gimbal based on target orientation
Quat FlightController::set_new_engine_gimbal_quat() {
    if (cs.stage < STAGE_1) return {1, 0, 0, 0};

    Quat target = cs.target_att;
    Quat current = cs.att;

    // calculate error between the two quaternions
    Quat q_err = current.inverse() * target;
    if (q_err.w < 0) q_err = {-q_err.w, -q_err.x, -q_err.y, -q_err.z};

    // for small angles the vector part of the error quaternion is proportional 
    // to the rotational error about body axis. n is the roll, pitch, yaw error in radians
    Vec3 n = { .x = 2 * q_err.x, .y = 2 * q_err.y, .z = 2 * q_err.z };

    // calculate required torque using PD
    double K_p = 0.5;
    double K_d = 0.5;
    Vec3 tau_req = (n * K_p) - (cs.w * K_d);

    // map torque to gimball command angles

    // active stage index for the current mission stage
    const Stage& s = props.stages[cs.stage];

    // calculate the CoM of the rocket
    double M = 0.0, m_CoM = 0.0, base = 0.0;
    for (int i = cs.stage; i < ROCKET_NUM_STAGES; i++) {
        const Stage& st = props.stages[i];
        double m = st.m_dry + st.m_fuel;
        M += m;
        m_CoM += m * (base + st.tip_to_end_length - st.CoM_dist);
        base += st.tip_to_end_length;
    }
    double z_cm = m_CoM / M;

    // engine gimbal point
    double s_engine = s.tip_to_end_length - s.engine_distance;

    // moment arm from the engine gimbal point to the rocket CoM
    double moment_arm = z_cm - s_engine;

    double R2 = props.radius * props.radius;
    double I = 0.0;
    base = 0.0;
    for (int i = cs.stage; i < ROCKET_NUM_STAGES; i++) {
        const Stage& st = props.stages[i];
        double m = st.m_dry + st.m_fuel, L = st.tip_to_end_length;
        double d = (base + L - st.CoM_dist) - z_cm;
        I += (1.0 / 12.0) * m * (3.0 * R2 + L * L) + m * d * d;
        base += L;
    }

    // calculate required pitch and yaw for engine
    double pitch = -I * tau_req.x / (s.max_thrust * moment_arm);
    double yaw = -I * tau_req.y / (s.max_thrust * moment_arm);

    // determine nozzzle deflection from body in quaternion orientation from pitch and yaw commands
    Quat q_pitch = { cos(pitch / 2), sin(pitch / 2), 0.0, 0.0 };
    Quat q_yaw = { cos(yaw / 2), 0.0, sin(yaw / 2), 0.0 };
    return (q_pitch * q_yaw).normalize();
}

// figures out what moment needs to be applied by the RCS system to achieve the target orientation 
Vec3 FlightController::calculate_rcs_moments_to_achieve_target_orientation() {
    Quat target = cs.target_att;
    Quat current = cs.att;

    // calculate error between the two quaternions
    Quat q_err = current.inverse() * target;
    if (q_err.w < 0) q_err = {-q_err.w, -q_err.x, -q_err.y, -q_err.z};

    // for small angles the vector part of the error quaternion is proportional 
    // to the rotational error about body axis. n is the roll, pitch, yaw error in radians
    Vec3 n = { .x = 2 * q_err.x, .y = 2 * q_err.y, .z = 2 * q_err.z };

    // calculate moments about payload (assume solid cone)
    Stage s = props.stages[cs.stage];
    double R2 = props.radius * props.radius;
    Vec3 I = {
        .x = 0.15 * s.m_dry * R2 + 0.0375 * s.m_dry * pow(s.tip_to_end_length, 2),
        .y = 0.15 * s.m_dry * R2 + 0.0375 * s.m_dry * pow(s.tip_to_end_length, 2),
        .z = 0.3 * s.m_dry * R2
    };
    double w_nat = 0.15; // random placeholder
    Vec3 K_p = I * w_nat * w_nat;
    Vec3 K_d = I * 0.7 * w_nat;
    Vec3 tau_req = n.vector_individual_multiply(K_p) - cs.w.vector_individual_multiply(K_d);
    std::cout << "PD applying moment of " << tau_req.x << ", " << tau_req.y << ", " << tau_req.z << " n-m\n";
    return { tau_req.x, tau_req.y, tau_req.z };
}

///////////////////////////////////////////////////////////////////////////////////////////////
// flight controller loop                                                                    //
///////////////////////////////////////////////////////////////////////////////////////////////

// runs on its own thread alongside the sim
void FlightController::flight_controller_process(Rocket& r, double current_time) {

    ////////////////////////////
    // control inside         //
    ////////////////////////////

    // NOTE!! I have chosen to pass the rocket struct into as few functions as possible unless absolutely necessary.
    // this means that anything staging function inside the switch statement shouldnt execute functions on the rocket
    // directly so the behavior isnt hidden and we dont accidentally do shit we dont want to

    if (cs.stage == STANDBY) init(r, current_time);

    // get latest data from the INS and advance the clock
    pull_new_data(r, current_time);

    // estimate state based on pulled data
    estimate_state();

    // manages setting a target attitude for the engine gimballing stuff depending on what the stage is
    switch (cs.stage) {
    case STANDBY:
        break;
    case ARMED:
        if (cs.time - countdown_start >= HOLD_DURATION) {
            cs.light_engine_flag = true;
            cs.stage_burn_time_start = cs.time;
            cs.stage = STAGE_1;
        }
        
        cs.v = {0, 0, 0}; // do this because the integrator doesnt account for the normal force so it thinks the rocket moves when sitting on pad
        break;
    case STAGE_1: {
        s1_powered();
        break;
    }
    case STAGE_2:
        s2_powered();
        break;
    case PAYLOAD_DEPLOY:
        payload_deploy();
        break;
    }

    // check if the engine was supposed to be cut off
    if (cs.cutoff_engine_flag) {
        cs.cutoff_engine_flag = false;
        r.cutoff_engine();
    }

    // check if the stage was supposed to be separated (clears the engine lock for the fresh stage)
    if (cs.separate_stage_flag) {
        cs.separate_stage_flag = false;
        r.advance_stage();
    }

    // check if engine was supposed to be lit
    if (cs.light_engine_flag) {
        cs.light_engine_flag = false;
        r.light_engine();
    }

    // send targeting commands to the engine gimbal system based on target attitude in cs
    r.set_engine_orientation(set_new_engine_gimbal_quat());

    // send orientation change commands to the rcs thruster system if it is active
    if (cs.rcs_activated_flag) {
        r.rcs_on();
        r.rcs_apply_const_moment(calculate_rcs_moments_to_achieve_target_orientation());
    }
    else {
        r.rcs_off();
    }
}
