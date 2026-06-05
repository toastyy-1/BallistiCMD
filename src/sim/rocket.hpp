#pragma once
#include "types.hpp"
#include <array>

struct Stage {
    double id;
    double m_dry; // dry mass
    double m_fuel; // fuel mass
    double m_flow_rate; // mass flow rate from engine (kg/s)
    double tip_to_end_length; // m
    double CM_dist; // dist of center of mass from front edge of the stage
    double max_thrust; // rated (max) motor thrust
    double thrust; // current commanded thrust
    double engine_distance; // distance of engine from leading edge
    double engine_gimball_range; // rad
};

class Rocket {
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // public                                                                                    //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    public:

    // setup
    Rocket();
    ~Rocket();

    // read-only accessors (let other threads peek at the current state)
    Vec3 get_pos() const { return r; }
    Vec3 get_vel() const { return v; }
    Vec3 get_acc() const { return a; }
    Vec3 get_ang_vel() const { return w; }
    Quat get_orientation() const { return q_rocket; }
    Quat get_engine_orientation() const { return q_engine; }
    double get_mass() const { return m_current; }
    double get_fuel_mass() const { return m_fuel_current; }
    double get_active_fuel() const { return active().m_fuel; }
    bool active_is_powered() const { return active().max_thrust > 0.0; }

    // setters (should only be used on setup)
    void set_pos(const Vec3& pos) { r = pos; } // set absolute position
    void set_orientation(const Quat& orient) { q_rocket = orient; } // set absolute orientation
    void set_start(double latitude, double longitude); // used to set the starting position of the rocket (should over over setting absolute position and orientation)
    void set_drag_coeff(double drag_coeff) { Cd = drag_coeff; }
    void set_stage(int stage_num, const Stage& cfg);
    void set_radius(double r) { radius = r; }


    // simulation things
    void light_engine(); // should be used once per stage
    void update_dynamics();
    void update_rotation();
    void update_mass();
    bool advance_stage();
    void set_engine_orientation(Quat orientation);

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // private                                                                                    //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    private:

    // rocket parameters
    static constexpr int NUM_STAGES = 4; // stage_1, stage_2, stage_3, payload
    std::array<Stage, NUM_STAGES> stages = {{
        { .id = 1 },
        { .id = 2 },
        { .id = 3 },
        { .id = 4 },
    }};

    int active_idx = 0; // index of the currently active stage
    double Cd = 0.0; // drag coefficient
    double m_current = 0; // current total mass (kg)
    double m_fuel_current = 0; // current total fuel mass (kg)
    Vec3 I_body = {0, 0, 0}; // moments of inertia about the combined CM, body frame
    double z_cm = 0; // combined CM along body +z, measured from the active stage's aft edge (m)
    double radius = 0; // hull radius for the solid-cylinder inertia model (m)

    // accessors for the currently active stage
    Stage& active() { return stages[active_idx]; }
    const Stage& active() const { return stages[active_idx]; }

    // sim states
    // all in ECI frame and in base SI units
    // this is all relative to the rocket's center of mass
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