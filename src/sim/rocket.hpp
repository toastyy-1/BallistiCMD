#pragma once
#include "types.hpp"
#include "constants.hpp"
#include "fc/fc.hpp"
#include <array>

struct Stage {
    double id;
    double m_dry;                   // dry mass
    double m_fuel;                  // fuel mass
    double isp;                     // specific impulse (s)
    double tip_to_end_length;       // m
    double CM_dist;                 // dist of center of mass from front edge of the stage
    double max_thrust;              // rated (max) motor thrust
    double thrust;                  // current commanded thrust
    double engine_distance;         // distance of engine from leading edge
    double engine_gimball_range;    // rad

    double exhaust_velocity() const { return isp * g0; }
    double mass_flow_rate() const { return isp > 0 ? thrust / exhaust_velocity() : 0.0; }
    double max_mass_flow_rate() const { return isp > 0 ? max_thrust / exhaust_velocity() : 0.0; }
};

struct RocketStartState {
    Vec3 origin_r_eci;
    Quat origin_q_eci; // origin attitude
    Vec3 target_r_eci;
};

// snapshot of rocket state at any given moment
struct RocketState {
    double t = 0;
    Vec3 r{}, v{}, a{}, w{};
    Quat q_rocket{1, 0, 0, 0};
    Quat q_engine{1, 0, 0, 0};
    double mass = 0, fuel = 0;
    double length = 0, cm_dist = 0, engine_dist = 0, radius = 0;   // dims from nose
    RocketStartState init{};
};

class Rocket {
    friend class INS;
    friend class FlightController;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // public                                                                                    //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    public:

    static constexpr int NUM_STAGES = 3; // stage_1, stage_2, payload

    // setup
    Rocket();
    ~Rocket();

    // getters:
    RocketState get_state() const;

    // setters (should only be used on setup)
    void set_pos(const Vec3& pos) { r = pos; } // set absolute position
    void set_orientation(const Quat& orient) { q_rocket = orient; } // set absolute orientation
    void set_start(double origin_latitude, double origin_longitude, double target_latitude, double target_longitude); // used to set the starting and target position of the rocket (should use over over setting absolute position and orientation)
    void set_drag_coeff(double drag_coeff) { props.Cd = drag_coeff; }
    void set_stage(int stage_num, const Stage& cfg);
    void set_radius(double r) { props.radius = r; }

    // simulation things
    void update_dynamics();
    void update_rotation();
    void update_mass();
    void update_flight_controller(double current_time) {
        if (!fc_initialized) { fc.init(*this, current_time); fc_initialized = true; }
        fc.flight_controller_process(*this, current_time);
    }

    // used by flight controller
    void light_engine(); // should be used once per stage
    bool advance_stage();
    void set_engine_orientation(Quat orientation);


    ///////////////////////////////////////////////////////////////////////////////////////////////
    // private                                                                                    //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    private:
    ///////////////////////////////////////////////////////////////////////////////////////////////
    // rocket static configuration                                                               //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    FlightController fc;
    bool fc_initialized = false;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // rocket static configuration                                                               //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    struct Properties {
        double radius = 0;  // hull radius for the solid-cylinder inertia model (m)
        double Cd = 0.0;    // drag coefficient
        std::array<Stage, NUM_STAGES> stages = {{
            { .id = 1 },
            { .id = 2 },
            { .id = 3 },
        }};
    };
    Properties props;

    // initial launch geometry (origin, target, launch attitude, such things)
    RocketStartState start_state;

    ///////////////////////////////////////////////////////////////////////////////////////////////
    // dynamic state                                                                             //
    ///////////////////////////////////////////////////////////////////////////////////////////////
    int active_idx = 0;         // index of the currently active stage

    // mass properties
    double m_current = 0;       // current total mass (kg)
    double m_fuel_current = 0;  // current total fuel mass (kg)
    Vec3 I_body = {0, 0, 0};    // moments of inertia about the combined CM, body frame
    double z_cm = 0;            // combined CM along body +z, from the active stage's aft edge (m)

    // kinematic state
    Vec3 r = {0, 0, 0};             // position (m)
    Vec3 v = {0, 0, 0};             // velocity (m/s)
    Vec3 a = {0, 0, 0};             // acceleration (m/s^2)
    Vec3 w = {0, 0, 0};             // angular velocity (rad/s)
    Quat q_rocket = {1, 0, 0, 0};   // orientation of rocket nose relative to ECI (+z is nose)
    Quat q_engine = {1, 0, 0, 0};   // orientation of engine relative to rocket body
    double altitude = 0;

    // accessors for the currently active stage
    Stage& active() { return props.stages[active_idx]; }
    const Stage& active() const { return props.stages[active_idx]; }

    // helper functions
    double calculate_engine_thrust_component();
    double calculate_engine_rotational_component();
    Vec3 calc_drag_accel();
    Vec3 nose_direction_eci();

};