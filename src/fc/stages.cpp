#include "fc.hpp"

void FlightController::s1_powered() {
    static double s1_start_time = cs.time;
    double dt = cs.time - s1_start_time;

    ///////////////////////////////////////////////////////////////////////////
    // do initial turn towards target before gravity turn starts
    ///////////////////////////////////////////////////////////////////////////
    constexpr double init_turn_len = 20.0; // seconds
    constexpr double init_tilt_angle = 2.0 * DEG_TO_RAD;

    if (dt < init_turn_len) {
        Vec3 up = cs.r.normalized();

        // direction towards the target in the plane perpendicular to the rockets starting up point
        Vec3 downrange_from_origin = cs.is.r_target - cs.is.r_origin;
        downrange_from_origin = (downrange_from_origin - up * downrange_from_origin.dot(up)).normalized();

        // turned slightly
        Vec3 tilt = up * cos(init_tilt_angle) + downrange_from_origin * sin(init_tilt_angle);
        cs.target_att = quat_from_vec(tilt);
    }

    ///////////////////////////////////////////////////////////////////////////
    // after initial turn, start gravity turn (follow v vec)
    ///////////////////////////////////////////////////////////////////////////
    else {
        // align with plane connecting to target and blend with current velocity vector
        Vec3 n = cs.is.r_origin.cross(cs.is.r_target).normalized();
        Vec3 v_in_target_plane = cs.v - n * cs.v.dot(n);
        cs.target_att = quat_from_vec(v_in_target_plane.normalized());
    }

    ///////////////////////////////////////////////////////////////////////////
    // separate once the stage has burned out
    ///////////////////////////////////////////////////////////////////////////
    if (cs.time - cs.stage_burn_time_start > cs.is.stage_burn_time[0] + 10) {
        cs.separate_stage_flag = true;
        cs.light_engine_flag = true;
        cs.stage = STAGE_2;
    }
}

void FlightController::s2_powered() {
    // intial start of s2 period
    static double s2_start = cs.time;
    const double tof_total = 1500;
    double dt = s2_start - cs.time;
    double t_ff = tof_total - dt; // remaining time of free flight


    Vec3 r = cs.r; // current position
    Vec3 v = cs.v; // current velocity
    Vec3 r_t = cs.is.r_target; // target position
    Vec3 g = cs.g;

    Vec3 v_req = ((r_t - r) / t_ff) - (g * t_ff * 0.5);

    Vec3 v_gain = v_req - v;

    cs.target_att = quat_from_vec(v_gain.normalized());
}