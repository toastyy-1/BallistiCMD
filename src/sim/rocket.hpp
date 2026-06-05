#pragma once
#include "types.hpp"

class Rocket {
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // public                                                                                    //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    public:

    // setup
    Rocket();
    ~Rocket();

    // getters
    Vec3 get_pos() const { return r; }
    Vec3 get_vel() const { return v; }
    Vec3 get_acc() const { return a; }
    Vec3 get_ang_vel() const { return w; }
    Quat get_orientation() const { return q_rocket; }
    Quat get_engine_orientation() const { return q_engine; }
    double get_mass() const { return m_dry + m_fuel; }
    double get_inertia() const { return I; }
    double get_drag_coeff() const { return Cd; }

    // setters (should only be used on setup)
    void set_pos(const Vec3& pos) { r = pos; }
    void set_orientation(const Quat& orient) { q_rocket = orient; }
    void set_dry_mass(double mass) { m_dry = mass; }
    void set_fuel_mass(double mass) { m_fuel = mass; }
    void set_mass_flow_rate(double mfr) { m_flow_rate = mfr; }
    void set_drag_coeff(double drag_coeff) { Cd = drag_coeff; }
    void set_nose_to_engine_length(double nel) { nose_to_engine_length = nel; }
    void set_CM_dist(double cm_dist) { CM_dist = cm_dist; }
    void set_moment_of_inertia(double i) { I = i; }
    void set_max_thrust(double max_t) { max_thrust = max_t; }
    void set_engine_dist(double ed) {engine_distance = ed;}
    void set_engine_gimball_range(double egr) {engine_gimball_range = egr;}


    // simulation things
    void update_dynamics();
    void update_rotation();
    void update_mass();
    void activate_engine(double throttle_percent);
    void set_engine_orientation(Quat orientation);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // private                                                                                    //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    private:
    // rocket parameters
    double m_dry = 0.0; // dry mass
    double m_fuel = 0.0; // fuel mass
    double nose_to_engine_length = 0.0; // m
    double CM_dist = 0.0; // distance of center of mass from the nose
    double I = 0.0; // moment of inertia
    double Cd = 0.0; // drag coefficient

    // engine properties
    double current_thrust = 0.0; // N
    double max_thrust = 0.0;
    double engine_distance = 0.0; // distance of engine from nose
    double m_flow_rate = 0.0; // flow rate of mass out of the engine - kg/s
    double engine_gimball_range = 0.0; // degrees

    // sim states
    // all in ECI frame and in base SI units
    Vec3 r = {0, 0, 0}; // position (m)
    Vec3 v = {0, 0, 0}; // velocity (m/s)
    Vec3 a = {0, 0, 0}; // acceleration (m/s^2)
    Vec3 w = {0, 0, 0}; // angular velocity (rad/s)

    Quat q_rocket = {1, 0, 0, 0}; // orientation of rocket nose relative to ECI (+z is nose)
    Quat q_engine = {1, 0, 0, 0}; // orientation of engine relative to rocket body

    double altitude = 0;

    // helper functions
    double calculate_engine_thrust_component();
    double calculate_engine_rotational_component();
    Vec3 nose_direction_eci();

};