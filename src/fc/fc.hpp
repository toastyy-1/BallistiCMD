#pragma once

#include "types.hpp"
#include "imu.hpp"
#include "sim/properties.hpp"

class Rocket;

// the intitial states that the rocket starts at
struct FCInitState {
    Vec3 r_origin, r_target;
    Quat q_origin;

    double launch_asimuth;

    Vec3 v_s1_bo_T;
    Vec3 r_s1_bo_T;

    double stage_burn_time[2];

    double target_tof;
};

// holds the rockets staging patterns
enum MissionStage {
    STANDBY = -2,
    ARMED = -1,
    STAGE_1 = 0,
    STAGE_2 = 1,
    PAYLOAD_DEPLOY = 2
};

// holds the states of controls for the rocket
struct ControlStates {
    MissionStage stage;

    // states
    Vec3 g; // gravity vector
    Vec3 a_inertial; // inertial acceleration
    Vec3 a; // g + a_inertial
    Vec3 v;
    Vec3 r;
    Vec3 w; // angualr velocity
    Quat att; // attitude
    Quat target_att; // the current iterations target attitude

    double dt; // time from last time measurement
    double time;
    double stage_burn_time_start; // mission time the current stage's engine was lit

    bool light_engine_flag = false; // set to true if you want to light the engine on that step
    bool separate_stage_flag = false; // set to true if you want to separate stage on that step
    bool cutoff_engine_flag = false; // set to true if you want to permanently cut off the engine on that step

    // initial states
    FCInitState is;
};

class FlightController {
    public:
    // load a fully set up rocket config/geometry into the FC and set up initial state
    FlightController(Rocket& r, double current_time);

    // perform flight controller operations (should be called every time we want to update the FC)
    void flight_controller_process(Rocket& r, double current_time);

    // command the active stage's motor to shut off permanently. thrust stops on the next FC step
    // and the motor cannot be relit until the rocket stages away from it
    void command_engine_cutoff() { cs.cutoff_engine_flag = true; }

    private:
    static constexpr double HOLD_DURATION = 40.0;

    // rocket geometry copied from the rocket part of sim once at startup
    const RocketProps props;

    INS ins;
    ControlStates cs;
    double countdown_start = 0;

    // helpers
    void init(Rocket& r, double current_time);
    FCInitState create_target_trajectory(double lat_target, double long_target, Rocket& r);
    void estimate_state();
    void pull_new_data(const Rocket& r, double current_time);
    Quat quat_from_vec(Vec3 u);
    Quat set_new_engine_gimbal_quat();

    // stages
    void s1_powered();
    void s2_powered();

};