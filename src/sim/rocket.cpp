#include "rocket.hpp"
#include "constants.hpp"
#include <cmath>

Rocket::Rocket() {
    // default constructor
}

Rocket::~Rocket() {
    // default destructor
}

/**
 * calculates the component of the engine's thrust that contributes to direct forward motion
 * based on its gimbal angle
 * @return forward thrust in newtons relative to body frame
 */
double Rocket::calculate_engine_thrust_component() {
    double gimbal_angle = 2.0 * std::acos(q_engine.w);
    return current_thrust * std::cos(gimbal_angle);
}

/**
 * calculates the component of the engines thrust that is applied normal to the orientation
 * of the rocket, creating a moment about the center of mass
 * @return lateral thrust in newtons relative to body frame
 */
double Rocket::calculate_engine_rotational_component() {
    double gimbal_angle = 2.0 * std::acos(q_engine.w);
    return current_thrust * std::sin(gimbal_angle);
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

void Rocket::update_dynamics() {

    // time step for the simulation
    double dt = TIME_STEP;

    // distance from center of earth
    double r_norm = r.norm();

    // thrust acceleration
    Vec3 a_thrust = nose_direction_eci() * (calculate_engine_thrust_component() / m_dry);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // RK4 integration                                                                           //
    ///////////////////////////////////////////////////////////////////////////////////////////////

    ////////////////////////////////////
    // k1 terms                       //
    ////////////////////////////////////
    // gravitional acceleration
    Vec3 k1_r = v;
    Vec3 k1_rv = r * ((-1) * (GM_EARTH / (r_norm * r_norm * r_norm)));

    // add engine thrust
    k1_rv += a_thrust;

    ////////////////////////////////////
    // k2 terms                       //
    ////////////////////////////////////
    // gravitional acceleration
    Vec3 k2_r = v + k1_rv * (dt / 2);
    Vec3 r2 = r + k1_r * (dt / 2);
    double r2_norm = r2.norm();
    Vec3 k2_rv = r2 * ((-1) * (GM_EARTH / (r2_norm * r2_norm * r2_norm)));

    // add engine thrust
    k2_rv += a_thrust;

    ////////////////////////////////////
    // k3 terms                       //
    ////////////////////////////////////
    // gravitional acceleration
    Vec3 k3_r = v + k2_rv * (dt / 2);
    Vec3 r3 = r + k2_r * (dt / 2);
    double r3_norm = r3.norm();
    Vec3 k3_rv = r3 * ((-1) * (GM_EARTH / (r3_norm * r3_norm * r3_norm)));

    // add engine thrust
    k3_rv += a_thrust;

    ////////////////////////////////////
    // k4 terms                       //
    ////////////////////////////////////
    // gravitional acceleration
    Vec3 k4_r = v + k3_rv * dt;
    Vec3 r4 = r + k3_r * dt;
    double r4_norm = r4.norm();
    Vec3 k4_rv = r4 * ((-1) * (GM_EARTH / (r4_norm * r4_norm * r4_norm))); 

    // add engine thrust
    k4_rv += a_thrust;

    ////////////////////////////////////
    // Rk4 formula
    ////////////////////////////////////
    // calculate change in r, v values
    Vec3 delta_r = (k1_r + k2_r*2 + k3_r*2 + k4_r) * (dt / 6);
    Vec3 delta_v = (k1_rv + k2_rv*2 + k3_rv*2 + k4_rv) * (dt / 6);

    // apply changes to rocket
    r += delta_r;
    v += delta_v;
    a = delta_v / dt; // for imu
    altitude = r_norm - EARTH_RADIUS_M;

    // keep rocket from falling through the earth
    if (r.norm() < EARTH_RADIUS_M) {
        Vec3 r_hat = r.normalized();
        r = r_hat * EARTH_RADIUS_M;
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

    // level arm
    Vec3 r_engine = {0, 0, -(engine_distance - CM_dist)};

    // torque in body frame
    net_torque += r_engine.cross(thrust_dir_body * current_thrust);


    ////////////////////////////////////
    // apply torque to orientation
    ////////////////////////////////////
    // angular acceleration and velocity (body frame)
    Vec3 ang_a = net_torque / I;
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
    double per = current_thrust / max_thrust;
    if (m_fuel > 0) {
        double sub = per * m_flow_rate;
        if (sub > m_fuel) {
            m_fuel = 0;
        }
        else m_fuel -= sub;
    }
}

void Rocket::activate_engine(double throttle_percent) {
    double throttle = throttle_percent / 100.0;
    current_thrust = max_thrust * throttle;
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
    double max_angle = engine_gimball_range * M_PI / 180.0;

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