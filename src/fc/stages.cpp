#include "fc.hpp"

void FlightController::s1_powered() {
    static double start_init_turn = cs.time;
    const double init_turn_time = 15.0; // seconds
    const double tilt_angle = 45.0 * DEG_TO_RAD;

    // do initial turn towards target before gravity turn starts
    if (cs.time - start_init_turn < init_turn_time) {
        Vec3 up = cs.r.normalized();

        // direction towards the target in the plane perpendicular to the rockets starting up point
        Vec3 downrange = cs.is.r_target - cs.is.r_origin;
        downrange = (downrange - up * downrange.dot(up)).normalized();

        // turned slightly
        Vec3 biased_up = up * cos(tilt_angle) + downrange * sin(tilt_angle);
        cs.target_att = quat_from_vec(biased_up);
    }

    // separate once the stage has burned out
    if (cs.time - cs.stage_burn_time_start > cs.is.stage_burn_time[0] + 10) {
        cs.separate_stage_flag = true;
        cs.stage++;
    }
}

void FlightController::s2_powered() {
    cs.light_engine_flag = true;

    static double start_init_turn = cs.time;
    const double init_turn_time = 15.0; // seconds
    const double tilt_angle = 60.0 * DEG_TO_RAD;

    // do initial turn towards target before gravity turn starts
    if (cs.time - start_init_turn < init_turn_time) {
        Vec3 up = cs.r.normalized();

        // direction towards the target in the plane perpendicular to the rockets starting up point
        Vec3 downrange = cs.is.r_target - cs.is.r_origin;
        downrange = (downrange - up * downrange.dot(up)).normalized();

        // turned slightly
        Vec3 biased_up = up * cos(tilt_angle) + downrange * sin(tilt_angle);
        cs.target_att = quat_from_vec(biased_up);
    }
}