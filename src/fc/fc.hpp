#pragma once

#include "types.hpp"
#include "imu.hpp"

class Rocket;

// the intitial states that the rocket starts at
struct FCInitState {
    Vec3 r_origin, r_target;

    double launch_asimuth;

    Vec3 v_s1_bo_T;
    Vec3 r_s1_bo_T;

    double target_tof;
};

// holds the rockets staging patterns
enum MissionStage {
    STANDBY,
    ARMED,
    STAGE_1_POWERED,
    STAGE_2_UNPOWERED,
    STAGE_2_POWERED,
    PAYLOAD_DEPLOY
};

// holds the states of controls for the rocket
struct ControlStates {
    MissionStage stage;

    // states
    Vec3 a_inertial; // specific force from the INS in ECI frame
    Vec3 a;
    Vec3 v;
    Vec3 r;
    Vec3 w;
    Quat att; // attitude

    double dt; // time from last time measurement
    double time;

    // initial states
    FCInitState is;
};

class FlightController {
    public:
    FlightController() = default;

    // configure against a fully set up rocket
    void init(Rocket& r, double current_time);

    // perform flight controller operations (should be called every time we want to update the FC)
    void flight_controller_process(Rocket& r, double current_time);

    private:
    static constexpr double HOLD_DURATION = 80.0;

    INS ins;
    ControlStates cs;
    double countdown_start = 0;

    FCInitState create_target_trajectory(double lat_target, double long_target, Rocket& r);
    void estimate_state();
    void pull_new_data(const Rocket& r, double current_time);

};