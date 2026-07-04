#include "fc.hpp"
#include <iostream>
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// S1 GUIDANCE
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void FlightController::s1_powered() {
    static double s1_start_time = cs.time;
    double dt = cs.time - s1_start_time;

    ///////////////////////////////////////////////////////////////////////////
    // do initial turn towards target before gravity turn starts
    ///////////////////////////////////////////////////////////////////////////
    constexpr double init_turn_len = 20.0; // seconds
    constexpr double init_tilt_angle = 50.0 * DEG_TO_RAD;

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
    if (cs.time - cs.stage_burn_time_start > cs.is.stage_burn_time[0] + 1) {
        cs.separate_stage_flag = true;
        cs.light_engine_flag = true;
        cs.stage = STAGE_2;
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// S2 GUIDANCE
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// stumpff functions helper
static double Cz(double z) {
    if (z >  1e-6) return (1.0 - cos(sqrt(z))) / z;
    if (z < -1e-6) return (cosh(sqrt(-z)) - 1.0) / (-z);
    return 0.5;
}
static double Sz(double z) {
    if (z >  1e-6) { 
        double s = sqrt(z);
        return (s - sin(s)) / (s*s*s);
    }
    if (z < -1e-6) { 
        double s = sqrt(-z);
        return (sinh(s) - s) / (s*s*s);
    }
    return 1.0 / 6.0;
}

// lambert problem (determine velocity vector that reaches target in time tff given current position)
static Vec3 lambert(Vec3 r, Vec3 r_t, double tff) {
    double r_norm = r.norm();
    double r_t_norm = r_t.norm();

    // transfer angle
    double d_theta = acos(r.dot(r_t) / (r_norm * r_t_norm));

    // some constant
    double A = sin(d_theta) * sqrt( (r_norm * r_t_norm) / (1 - cos(d_theta)) );

    // helper for y(z) function in lambert
    auto y_z = [&](double z) {
        return r_norm + r_t_norm + A * (z * Sz(z) - 1.0) / sqrt(Cz(z));
    };

    // iterate z until the trajectory's time of flight is the same as the input time of flight
    double z = 0.0; // initial number guess
    while (y_z(z) < 0.0) z += 0.1; // bump up z until y(z) is good enough

    // now iterate on z to find the best one using secant search
    double z0 = 0.0;
    double z1 = 0.1; // start with initial guesses that are kinda close to one another

    for (int i = 0; i < 60; i++) {
        // find F at z0
        double y0 = y_z(z0);
        double F0 = pow(y0 / Cz(z0), 1.5) * Sz(z0) + A * sqrt(y0) - sqrt(GM_EARTH) * tff;
        
        // find F at z1
        double y1 = y_z(z1);
        double F1 = pow(y1 / Cz(z1), 1.5) * Sz(z1) + A * sqrt(y1) - sqrt(GM_EARTH) * tff;
        
        // step
        double z_next = z1 - F1 * (z1 - z0) / (F1 - F0);
        
        // check if it converges
        if (fabs(z_next - z1) < 1e-8) {
            z = z_next;
            break;
        }
        
        // shift values for next iteration
        z0 = z1;
        z1 = z_next;
    }

    // now we determine the lagrange multipliers as part of the analytical version of the lambert problem to find our required velocity vector
    double y = y_z(z);
    double f = 1 - y / r_norm;
    double g = A * sqrt(y / GM_EARTH);

    return (r_t - r * f) * (1 / g); // optimal taret velocity
}

void FlightController::s2_powered() {
    static int counter = 3;
    counter++;
    static double burn_start_t = cs.time;
    double burn_time = cs.time - burn_start_t;

    static Vec3 v_req{}; // optimal required velocity

    ////////////////////////////////////////////////////////////
    // determine the minimum rquired velocity
    ////////////////////////////////////////////////////////////
    if (counter >= 3) {
        counter = 0; // reset counter

        ////////////////////////////////////////////////////////////////////////////////////////
        // use golden section search to find the optimal required velocity vector using lambert problem 
        ////////////////////////////////////////////////////////////////////////////////////////
        double rho = 1 / ( (1 + sqrt(5) ) / 2);

        double a = 300; // lower bound of guess
        double b = 2700; // no reasonable flight should take longer than this right?
        
        // pick two points based on upper and lower bounds
        double x1 = b - rho * (b - a);
        double x2 = a + rho * (b - a);

        // do the lambert problem on these two points (smallest change in velocity)
        double f1 = (lambert(cs.r, cs.is.r_target, x1) - cs.v).norm();
        double f2 = (lambert(cs.r, cs.is.r_target, x2) - cs.v).norm();

        // iterate on a and b to find the minimum delta v point
        while ((b - a) > 0.1) {
            if (f1 < f2) {
                b = x2;
                x2 = x1;
                f2 = f1;
                x1 = b - rho * (b - a);
                f1 = (lambert(cs.r, cs.is.r_target, x1) - cs.v).norm();
            }
            else {
                a = x1;
                x1 = x2;
                f1 = f2;
                x2 = a + rho * (b - a);
                f2 = (lambert(cs.r, cs.is.r_target, x2) - cs.v).norm();
            }
        }

        v_req = lambert(cs.r, cs.is.r_target, 0.5 * (a + b));
    }

    // velocity to be gained as a target
    Vec3 v_gain = v_req - cs.v;

    // stop the engine if the V is within proper cutoff range OR if the fuel is out
    // the cutoff should be tied to the max possible delta V of the 3rd stage engine
    double next_delta_v = props.stages[2].isp * g0 * log((props.stages[2].m_dry + props.stages[2].m_fuel) / props.stages[2].m_dry);
    double estimated_s2_burn_time = props.stages[1].m_fuel / (props.stages[1].max_mass_flow_rate());
    if (v_gain.norm() < next_delta_v - 0.3 * next_delta_v || burn_time > estimated_s2_burn_time) { // within 30% for margin of error, or motor depleted
        // stop engine to stop overshoot
        cs.cutoff_engine_flag = true;
        std::cout << "ENGINE_CUTOFF at " << v_gain.norm() << "m/s " << "with next stage having delta v: " << next_delta_v << "m/s\n";
        
        // switch stage
        cs.stage = PAYLOAD_DEPLOY;

        // set stage separation
        cs.separate_stage_flag = true;

        return;
    }

    // set attitude to new target
    cs.target_att = quat_from_vec(v_gain.normalized());

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// S3 GUIDANCE
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// simply operates a modified version of stage 2 that improves trajectory accuracy with small engine
void FlightController::payload_deploy() {
    // initially, set rcs flag to true so we can orient rocket in right direction
    cs.rcs_activated_flag = true;

    ////////////////////////////////////////////////////////////////////////////////////////
    // calculate error in direction to know when its safe to light engine (otherwise use RCS)
    ////////////////////////////////////////////////////////////////////////////////////////
    Quat target = cs.target_att;
    Quat current = cs.att;

    // calculate error between the two quaternions
    Quat q_err = current.inverse() * target;
    if (q_err.w < 0) q_err = {-q_err.w, -q_err.x, -q_err.y, -q_err.z};

    // for small angles the vector part of the error quaternion is proportional
    // to the rotational error about body axis. n is the roll, pitch, yaw error in radians
    Vec3 n = { .x = 2 * q_err.x, .y = 2 * q_err.y, .z = 2 * q_err.z };

    // burn is ready once the attitude error has gotten low enough so engins is not lit while the rocket is still wobbling around
    constexpr double attitude_tolerance = 0.5 * DEG_TO_RAD; // rad
    bool burn_ready = n.norm() < attitude_tolerance;
    if (burn_ready) { cs.light_engine_flag = true; }

    ////////////////////////////////////////////////////////////////////////////////////////
    // use golden section search to find the optimal required velocity vector using lambert problem 
    ////////////////////////////////////////////////////////////////////////////////////////
    static Vec3 v_req{}; // optimal required velocity

    double rho = 1 / ( (1 + sqrt(5) ) / 2);

    double a = 300; // lower bound of guess
    double b = 2700; // no reasonable flight should take longer than this right?
    
    // pick two points based on upper and lower bounds
    double x1 = b - rho * (b - a);
    double x2 = a + rho * (b - a);

    // do the lambert problem on these two points
    double f1 = (lambert(cs.r, cs.is.r_target, x1) - cs.v).norm();
    double f2 = (lambert(cs.r, cs.is.r_target, x2) - cs.v).norm();

    // iterate on a and b to find the minimum delta v point
    while ((b - a) > 0.1) {
        if (f1 < f2) {
            b = x2;
            x2 = x1;
            f2 = f1;
            x1 = b - rho * (b - a);
            f1 = (lambert(cs.r, cs.is.r_target, x1) - cs.v).norm();
        }
        else {
            a = x1;
            x1 = x2;
            f1 = f2;
            x2 = a + rho * (b - a);
            f2 = (lambert(cs.r, cs.is.r_target, x2) - cs.v).norm();
        }
    }

    v_req = lambert(cs.r, cs.is.r_target, 0.5 * (a + b));

    // velocity to be gained as a target
    Vec3 v_gain = v_req - cs.v;

    // stop the engine if the V is within proper cutoff range
    if (v_gain.norm() < 1.0) {
        // stop engine to stop overshoot
        cs.cutoff_engine_flag = true;
        std::cout << "ENGINE_CUTOFF\n";
        cs.rcs_activated_flag = false;
        return;
    }

    // set attitude to new target
    cs.target_att = quat_from_vec(v_gain.normalized());
}