#include "rocket.hpp"
#include "constants.hpp"
#include <cmath>
#include <iostream>

Rocket::Rocket(double origin_latitude, double origin_longitude, double target_latitude, double target_longitude) {
    set_start(origin_latitude, origin_longitude, target_latitude, target_longitude);

    // stage 1
    props.stages[0] = {
        .id = 1,
        .m_dry = 4000.0,
        .m_fuel = 117910.0,
        .isp = 296.0,
        .tip_to_end_length = 21.4,
        .CoM_dist = 11.0,
        .max_thrust = 2200000.0,
        .engine_distance = 21.4,
        .engine_gimball_range = 5.0
    };

    // stage 2
    props.stages[1] = {
        .id = 2,
        .m_dry = 2800.0,
        .m_fuel = 27200.0,
        .isp = 316.0,
        .tip_to_end_length = 9.4,
        .CoM_dist = 4.7,
        .max_thrust = 445000.0,
        .engine_distance = 9.4,
        .engine_gimball_range = 5.0
    };

    // payload
    props.stages[2] = {
        .id = 3,
        .m_dry = 3700.0,
        .m_fuel = 0.0,
        .isp = 0.0,
        .tip_to_end_length = 3.1,
        .CoM_dist = 1.5,
        .max_thrust = 0.0,
        .engine_distance = 3.1,
        .engine_gimball_range = 0.0
    };

    props.radius = 1.524;
}

Rocket::~Rocket() {
    // default destructor
}

/**
 * advances the stage of the rocket to the next one
 */
RocketState Rocket::get_state() const {
    double length = 0;
    for (int i = active_idx; i < NUM_STAGES; i++) length += props.stages[i].tip_to_end_length;
    double s_engine = active().tip_to_end_length - active().engine_distance;

    RocketState s;
    s.r           = r;
    s.v           = v;
    s.a           = a;
    s.w           = w;
    s.q_rocket    = q_rocket;
    s.q_engine    = q_engine;
    s.mass        = m_current;
    s.fuel        = m_fuel_current;
    s.length      = length;
    s.cm_dist     = length - z_cm;
    s.engine_dist = length - s_engine;
    s.radius      = props.radius;
    s.init        = start_state;
    return s;
}

bool Rocket::advance_stage() {
    if (active_idx + 1 < NUM_STAGES) {
        active_idx++;
        engine_locked = false; // fresh stage
        return true;
    }
    return false;
}

/**
 * control lighting the engine on the current active stage
 */
void Rocket::light_engine() {
    if (engine_locked) return; // motor was cut off and cannot be relit on this stage
    if (active().m_fuel > 0) {
        active().thrust = active().max_thrust;
    }
}

/**
 * permanently terminates thrust on the active stage, kills motor real dead
 */
void Rocket::cutoff_engine() {
    active().thrust = 0;
    engine_locked = true;
}

/**
 * calculates the component of the engine's thrust that contributes to direct forward motion
 * based on its gimbal angle
 * @return forward thrust in newtons relative to body frame
 */
double Rocket::calculate_engine_thrust_component() {
    double gimbal_angle = 2.0 * std::acos(q_engine.w);
    return active().thrust * std::cos(gimbal_angle);
}

/**
 * calculates the component of the engines thrust that is applied normal to the orientation
 * of the rocket, creating a moment about the center of mass
 * @return lateral thrust in newtons relative to body frame
 */
double Rocket::calculate_engine_rotational_component() {
    double gimbal_angle = 2.0 * std::acos(q_engine.w);
    return active().thrust * std::sin(gimbal_angle);
}

/**
 * @return the rocket's nose direction in ECI frame coordinates
 */
Vec3 Rocket::nose_direction_eci() {
    double w = q_rocket.w, x = q_rocket.x, y = q_rocket.y, z = q_rocket.z;
    return {
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y)
    };
}

// gravitational acceleration in the ECI frame
static Vec3 calc_gravity_accel(const Vec3& r) {
    double r2       = r.dot(r);
    double r_norm   = std::sqrt(r2);
    double pm       = -GM_EARTH / (r2 * r_norm);

    double zr2      = (r.z * r.z) / r2;
    double k        = 1.5 * J2 * (EARTH_RADIUS * EARTH_RADIUS) / r2;

    return {
        .x = pm * r.x * (1.0 - k * (5.0 * zr2 - 1.0)),
        .y = pm * r.y * (1.0 - k * (5.0 * zr2 - 1.0)),
        .z = pm * r.z * (1.0 - k * (5.0 * zr2 - 3.0)),
    };
}

// acceleration due to drag in the ECI frame
Vec3 Rocket::calc_drag_accel() {
    double altitude = r.norm() - EARTH_RADIUS; // altitude in meters
    double air_density;

    // power relationship density equation
    auto pow_dens = [&altitude](double rho_b, double T_b, double L, double layer_base_alt) {
        return rho_b * pow(( (T_b + L * (altitude - layer_base_alt)) / T_b ), (-1.0 * g0 / (R_d * L)) - 1);
    };

    // exponential relationship density equation
    auto exp_dens = [&altitude](double rho_b, double T_b, double layer_base_alt) {
        return rho_b * exp(-1.0 * (g0 * (altitude - layer_base_alt)) / (R_d * T_b));
    };

    // troposphere
    if (altitude < 11000) {
        air_density = pow_dens(1.2250, 288.15, -0.0065, 0);
    }
    // lower stratosphere
    else if (altitude < 20000) {
        air_density = exp_dens(0.36391, 216.65, 11000);
    }
    // middle stratosphere
    else if (altitude < 32000) {
        air_density = pow_dens(0.088035, 216.65, 0.001, 20000);
    }
    // upper stratosphere
    else if (altitude < 47000) {
        air_density = pow_dens(0.013225, 228.65, 0.0028, 32000);
    }
    // lower mesosphere
    else if (altitude < 51000) {
        air_density = exp_dens(0.0014275, 270.65, 47000);
    }
    // middle mesosphere
    else if (altitude < 71000) {
        air_density = pow_dens(0.00086160, 270.65, -0.0028, 51000);
    }
    // upper mesosphere
    else if (altitude < 86000) {
        air_density = pow_dens(0.000064211, 214.65, -0.0020, 71000);
    }
    // thermosphere
    else {
        air_density = exp_dens(0.000006958, 186.87, 86000);
    }

    // wind of earth spinning
    Vec3 w_earth = {0, 0, EARTH_ROTATION_RATE};
    Vec3 v_air = w_earth.cross(r);
    Vec3 v_relative = v - v_air;

    double craft_speed = v_relative.norm();

    // calcualte the aoa entering into the atmosphere
    //Vec3 nose_direction = nose_direction_eci();
    //double AoA = std::acos(std::max(-1.0, std::min(1.0, v_relative.dot(nose_direction) / craft_speed)));
    //std::cout << AoA * RAD_TO_DEG << std::endl;

    // @todo apply drag (shid rn add a real drag model idoit) ((idfk how im going to do that simply))
    double area = M_PI * props.radius * props.radius;
    double drag_mag = 0.5 * air_density * craft_speed * craft_speed * props.Cd * area;
    return v_relative * (-drag_mag / (m_current * craft_speed));
}

void Rocket::update_dynamics() {
    // rocket mass
    double m = m_current;

    // time step for the simulation
    double dt = TIME_STEP;

    // distance from center of earth
    double r_norm = r.norm();

    // thrust acceleration
    Vec3 a_thrust = nose_direction_eci() * (calculate_engine_thrust_component() / m);

    // drag acceleration
    Vec3 a_drag = calc_drag_accel();

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // RK4 integration                                                                           //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////
    // k1 terms                       //
    ////////////////////////////////////
    // gravitional acceleration
    Vec3 k1_r = v;
    Vec3 k1_rv = calc_gravity_accel(r);

    // add engine thrust and drag
    k1_rv += a_thrust + a_drag;

    ////////////////////////////////////
    // k2 terms                       //
    ////////////////////////////////////
    // gravitional acceleration
    Vec3 k2_r = v + k1_rv * (dt / 2);
    Vec3 r2 = r + k1_r * (dt / 2);
    Vec3 k2_rv = calc_gravity_accel(r2);

    // add engine thrust and drag
    k2_rv += a_thrust + a_drag;

    ////////////////////////////////////
    // k3 terms                       //
    ////////////////////////////////////
    // gravitional acceleration
    Vec3 k3_r = v + k2_rv * (dt / 2);
    Vec3 r3 = r + k2_r * (dt / 2);
    Vec3 k3_rv = calc_gravity_accel(r3);

    // add engine thrust and drag
    k3_rv += a_thrust + a_drag;

    ////////////////////////////////////
    // k4 terms                       //
    ////////////////////////////////////
    // gravitional acceleration
    Vec3 k4_r = v + k3_rv * dt;
    Vec3 r4 = r + k3_r * dt;
    Vec3 k4_rv = calc_gravity_accel(r4);

    // add engine thrust and drag
    k4_rv += a_thrust + a_drag;

    ////////////////////////////////////
    // Rk4 formula
    ////////////////////////////////////
    // calculate change in r, v values
    Vec3 delta_r = (k1_r + k2_r*2 + k3_r*2 + k4_r) * (dt / 6);
    Vec3 delta_v = (k1_rv + k2_rv*2 + k3_rv*2 + k4_rv) * (dt / 6);

    // apply changes to rocket
    r += delta_r;
    v += delta_v;
    a = delta_v / dt; // for INS
    altitude = r_norm - EARTH_RADIUS;

    // keep rocket from falling through the earth
    if (r.norm() < EARTH_RADIUS) {
        Vec3 r_hat = r.normalized();
        r = r_hat * EARTH_RADIUS;
        v = r_hat * std::max(v.dot(r_hat), 0.0);

    }

}

void Rocket::update_rotation() {
    double dt = TIME_STEP;

    // net torque
    Vec3 net_torque = {0, 0, 0};
    
    ////////////////////////////////////
    // engine thrust                  //
    ////////////////////////////////////
    // thrust direction in body frame
    Vec3 nose_body = {0, 0, 1};
    Vec3 q_vec = {q_engine.x, q_engine.y, q_engine.z};
    Vec3 t = q_vec.cross(nose_body);
    Vec3 thrust_dir_body = nose_body + t * (2.0 * q_engine.w) + q_vec.cross(t) * 2.0;

    // lever arm from the combined CM to the engine, along the body axis
    double s_engine = active().tip_to_end_length - active().engine_distance;
    Vec3 r_engine = {0, 0, s_engine - z_cm};

    // torque in body frame
    net_torque += r_engine.cross(thrust_dir_body * active().thrust);


    ////////////////////////////////////
    // apply torque to orientation
    ////////////////////////////////////
    Vec3 Iw   = {I_body.x * w.x, I_body.y * w.y, I_body.z * w.z};
    Vec3 gyro = w.cross(Iw);
    Vec3 ang_a = {
        (net_torque.x - gyro.x) / I_body.x,
        (net_torque.y - gyro.y) / I_body.y,
        (net_torque.z - gyro.z) / I_body.z,
    };
    w += ang_a * dt;

    // modify quaternion orientation from change
    Quat omega_q = {0.0, w.x * 0.5 * dt, w.y * 0.5 * dt, w.z * 0.5 * dt};
    double nw = q_rocket.w*omega_q.w - q_rocket.x*omega_q.x - q_rocket.y*omega_q.y - q_rocket.z*omega_q.z;
    double nx = q_rocket.w*omega_q.x + q_rocket.x*omega_q.w + q_rocket.y*omega_q.z - q_rocket.z*omega_q.y;
    double ny = q_rocket.w*omega_q.y - q_rocket.x*omega_q.z + q_rocket.y*omega_q.w + q_rocket.z*omega_q.x;
    double nz = q_rocket.w*omega_q.z + q_rocket.x*omega_q.y - q_rocket.y*omega_q.x + q_rocket.z*omega_q.w;
    q_rocket.w += nw;
    q_rocket.x += nx;
    q_rocket.y += ny;
    q_rocket.z += nz;

    // renormalize
    double qnorm = std::sqrt(q_rocket.w*q_rocket.w + q_rocket.x*q_rocket.x + q_rocket.y*q_rocket.y + q_rocket.z*q_rocket.z);
    q_rocket.w /= qnorm;
    q_rocket.x /= qnorm;
    q_rocket.y /= qnorm;
    q_rocket.z /= qnorm;

}

// updates the fuel mass based on the current throttle and mass flow rate
void Rocket::update_mass() {
    // update per-stage mass
    Stage& s = active();
    if (s.m_fuel > 0 && s.thrust > 0) {
        double sub = s.mass_flow_rate() * TIME_STEP;
        if (sub > s.m_fuel) {
            s.m_fuel = 0;
            s.thrust = 0;
        }
        else s.m_fuel -= sub;
    }

    double M = 0, M_f = 0, m_cm = 0, base = 0;
    for (int i = active_idx; i < NUM_STAGES; i++) {
        const Stage& st = props.stages[i];
        double m = st.m_dry + st.m_fuel;
        M += m;
        M_f += st.m_fuel;
        m_cm += m * (base + st.tip_to_end_length - st.CoM_dist);
        base += st.tip_to_end_length;
    }
    m_current = M;
    m_fuel_current = M_f;
    z_cm = m_cm / M;

    // also adjust moment using assumption that each stage is uniform cylinder
    double R2 = props.radius * props.radius, I_trans = 0;
    base = 0;
    for (int i = active_idx; i < NUM_STAGES; i++) {
        const Stage& st = props.stages[i];
        double m = st.m_dry + st.m_fuel, L = st.tip_to_end_length;
        double d = (base + L - st.CoM_dist) - z_cm;
        I_trans += (1.0 / 12.0) * m * (3.0 * R2 + L * L) + m * d * d;
        base += L;
    }
    I_body = { I_trans, I_trans, 0.5 * R2 * M };
}

void Rocket::set_engine_orientation(Quat orientation) {
    // normalize input
    double norm = std::sqrt(orientation.w*orientation.w + orientation.x*orientation.x +
                            orientation.y*orientation.y + orientation.z*orientation.z);
    orientation.w /= norm;
    orientation.x /= norm;
    orientation.y /= norm;
    orientation.z /= norm;

    double angle = 2.0 * std::acos(std::max(-1.0, std::min(1.0, orientation.w)));
    double max_angle = active().engine_gimball_range * M_PI / 180.0;

    if (angle <= max_angle) {
        q_engine = orientation;
        return;
    }

    // clamp to max gimbal angle
    double sin_half = std::sin(angle / 2.0);
    if (sin_half < 1e-9) {
        q_engine = {1, 0, 0, 0};
        return;
    }
    double half_max = max_angle / 2.0;
    double s = std::sin(half_max) / sin_half;
    q_engine = {std::cos(half_max), orientation.x * s, orientation.y * s, orientation.z * s};
}

static Vec3 lat_lon_to_eci(double latitude_deg, double longitude_deg) {
    double lat = latitude_deg * M_PI / 180.0;
    double lon = longitude_deg * M_PI / 180.0;
    return {
        .x = EARTH_RADIUS * cos(lat) * cos(lon),
        .y = EARTH_RADIUS * cos(lat) * sin(lon),
        .z = EARTH_RADIUS * sin(lat)
    };
}

void Rocket::set_start(double origin_latitude, double origin_longitude, double target_latitude, double target_longitude) {
    Vec3 origin_pos = lat_lon_to_eci(origin_latitude, origin_longitude);

    // set the position of the rocket to that asolute position
    start_state.origin_r_eci = origin_pos;
    set_pos(origin_pos);

    // set target position
    start_state.target_r_eci = lat_lon_to_eci(target_latitude, target_longitude);

    // determine the necessary orientation to achive normal "up" position from surface
    double lat = origin_latitude * M_PI / 180.0;
    double lon = origin_longitude * M_PI / 180.0;
    Vec3 unit_vec_from_center = {
        .x = cos(lat) * cos(lon),
        .y = cos(lat) * sin(lon),
        .z = sin(lat)
    };
    Vec3 up = {0, 0, 1}; // since +z is up
    Vec3 rotation_axis = up.cross(unit_vec_from_center);
    Vec3 rot_axis_u = rotation_axis / rotation_axis.norm();

    double half_theta = acos(sin(lat)) / 2;
    Quat q = {
        .w = cos(half_theta),
        .x = sin(half_theta) * rot_axis_u.x,
        .y = sin(half_theta) * rot_axis_u.y,
        .z = sin(half_theta) * rot_axis_u.z
    };
    
    // set the oritnetaion of the rocket to normal the surface
    set_orientation(q);
    start_state.origin_q_eci = q;
}