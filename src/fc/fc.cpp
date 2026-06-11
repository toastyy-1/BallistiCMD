#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <random>
#include <vector>
#include <thread>
#include <chrono>
#include "fc.hpp"
#include "imu.hpp"
#include "types.hpp"
#include "sim/rocket.hpp"
#include "targeting.hpp"


///////////////////////////////////////////////////////////////////////////////////////////////
// structs                                                                                   //
///////////////////////////////////////////////////////////////////////////////////////////////

// holds the rocket's staging patterns
enum mission_stage {
    STANDBY,
    ARMED,
    STAGE_1_POWERED,
    STAGE_2_UNPOWERED,
    STAGE_2_POWERED,
    PAYLOAD_DEPLOY
};

// holds the states of controls for the rocket
struct control_states {
    mission_stage stage;

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
    Initial_States is;
};

///////////////////////////////////////////////////////////////////////////////////////////////
// helper functions                                                                          //
///////////////////////////////////////////////////////////////////////////////////////////////

// print telem in this beautiful function that ive conveniently made very readable
void print_telemetry(const control_states& cs, const sim::Sim& sim) {
    const char* stage;
    switch (cs.stage) {case STANDBY: stage = "STANDBY"; break;case ARMED:stage = "ARMED"; break;case STAGE_1_POWERED: stage = "STAGE 1 POWERED"; break;case STAGE_2_UNPOWERED: stage = "STAGE 2 UNPOWERED"; break;case STAGE_2_POWERED: stage = "STAGE 2 POWERED"; break;case PAYLOAD_DEPLOY: stage = "PAYLOAD DEPLOY"; break;default: stage = "UNKNOWN"; break;}
    std::cout << std::fixed << "\033[1;31m[" << "\033[1;33m" << stage << "\033[1;31m]" << "\033[0;37m  T+" << "\033[1;37m" << std::setprecision(1) << cs.time << "s" << "\n" << "\033[0;37m  acc(m/s²)  x=" << "\033[1;36m" << std::setw(9) << std::setprecision(4) << cs.a.x << "\033[0;37m  y=" << "\033[1;36m" << std::setw(9) << cs.a.y<< "\033[0;37m  z=" << "\033[1;36m" << std::setw(9) << cs.a.z<< "\033[0;37m  |a|=" << "\033[1;36m" << std::setw(9) << cs.a.norm()<< "\n"<< "\033[0;37m  gyr(rad/s) x=" << "\033[1;36m" << std::setw(9) << cs.w.x<< "\033[0;37m  y=" << "\033[1;36m" << std::setw(9) << cs.w.y<< "\033[0;37m  z=" << "\033[1;36m" << std::setw(9) << cs.w.z<< "\033[0;37m  |w|=" << "\033[1;36m" << std::setw(9) << cs.w.norm()<< "\033[0m\n";
    static std::ofstream csv = [] { bool has_data = std::ifstream("fc_telem.csv").peek() != std::ifstream::traits_type::eof(); std::ofstream f("fc_telem.csv", std::ios::app); if (!has_data) {f << "time,stage,r_x,r_y,r_z,v_x,v_y,v_z,a_x,a_y,a_z,w_x,w_y,w_z\n";} return f;}();
    csv << std::fixed << std::setprecision(6) << cs.time << ',' << stage << ',' << cs.r.x << ',' << cs.r.y << ',' << cs.r.z << ',' << cs.v.x << ',' << cs.v.y << ',' << cs.v.z << ',' << cs.a.x << ',' << cs.a.y << ',' << cs.a.z << ',' << cs.w.x << ',' << cs.w.y << ',' << cs.w.z << '\n';
    Vec3 sr = sim.get_rocket_pos(), sv = sim.get_rocket_vel(), sa = sim.get_rocket_acc(), sw = sim.get_rocket_ang_vel();
    static std::ofstream sim_csv = [] { bool has_data = std::ifstream("sim_telem.csv").peek() != std::ifstream::traits_type::eof(); std::ofstream f("sim_telem.csv", std::ios::app); if (!has_data) {f << "time,stage,r_x,r_y,r_z,v_x,v_y,v_z,a_x,a_y,a_z,w_x,w_y,w_z\n";} return f;}();
    sim_csv << std::fixed << std::setprecision(6) << cs.time << ',' << stage << ',' << sr.x << ',' << sr.y << ',' << sr.z << ',' << sv.x << ',' << sv.y << ',' << sv.z << ',' << sa.x << ',' << sa.y << ',' << sa.z << ',' << sw.x << ',' << sw.y << ',' << sw.z << '\n';
}

// aquires new data from the sim
void pull_new_data(const sim::Sim& sim, control_states& cs, INS& ins) {
    cs.a_inertial = ins.read_INS_acc();
    cs.w = ins.read_INS_gyr();
    double actual_sim_time = sim.get_time();
    cs.dt = actual_sim_time - cs.time;
    cs.time = actual_sim_time; // this must happen after cs.dt or else cs.dt will be 0 :)
}

// estimate state of the rocket in flight at the current moment
void estimate_state(control_states& cs) {
    // determine true acceleration from gravity based on past position state
    double r_norm = cs.r.norm();
    Vec3 g = cs.r * (-1.0 * GM_EARTH / (r_norm * r_norm * r_norm));
    cs.a = g + cs.a_inertial;

    // add delta v based on a to current v
    cs.v = cs.v + cs.a * cs.dt;

    // add delta r based on v to current r
    cs.r = cs.r + cs.v * cs.dt;
}


///////////////////////////////////////////////////////////////////////////////////////////////
// flight controller loop                                                                    //
///////////////////////////////////////////////////////////////////////////////////////////////

// runs on its own thread alongside the sim
int flight_controller_loop(sim::Sim& sim) {
    std::this_thread::sleep_for(std::chrono::duration<double>(3));
    
    ////////////////////////////
    // init                   //
    ////////////////////////////
    INS ins(sim);
    Rocket& r = sim.rocket;
    control_states cs = {};
        cs.stage = STANDBY;
        cs.is = create_target_trajectory(38.9072, -77.0369, sim);
        cs.r = cs.is.r_origin; // set initial r to starting r

    std::cout << "\033[1;31m" << "[ " << "\033[1;33m" << "MISSILE CONTROL" << "\033[1;31m" << " ] " << "\033[1;37m" << "LAUNCHING MISSILE" << "\033[0m" << std::endl;
    cs.stage = STAGE_1_POWERED;

    ////////////////////////////
    // control loop           //
    ////////////////////////////
    int tick = 0;
    cs.time = sim.get_time(); // take initial time measurement
    while (sim.is_running()) {
        // get data from the INS
        pull_new_data(sim, cs, ins);

        // estimate state based on pulled data
        estimate_state(cs);
        
        if (++tick % 10 == 0) print_telemetry(cs, sim);

        switch (cs.stage) {
        case STANDBY:
            break;
        case ARMED:
            break;
        case STAGE_1_POWERED: {
            static bool engine_lit = false;
            if (!engine_lit) {
                r.light_engine();
                engine_lit = true;
            }

            static double start = cs.time;
            static bool is_separated = false;
            if (cs.time - start > 130 && !is_separated) {
                r.advance_stage();
                is_separated = true;
                r.light_engine();
            }

            const double turn_time = 3.0;
            static double turn_time_start = cs.time;
            if ((cs.time - turn_time_start) < turn_time) {
                double half = (0.3 * M_PI / 180.0) / 2.0;
                r.set_engine_orientation( {std::cos(half), 0.0, std::sin(half), 0.0} );
            }
            else if ((cs.time - turn_time_start) < 2.0 * turn_time) {
                double half = (-0.3 * M_PI / 180.0) / 2.0;
                r.set_engine_orientation( {std::cos(half), 0.0, std::sin(half), 0.0} );
            }
            else r.set_engine_orientation({1, 0, 0, 0});
            break;
        }
        case STAGE_2_UNPOWERED:
            break;
        case STAGE_2_POWERED:
            break;
        case PAYLOAD_DEPLOY:
            break;
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
    }
    return 0;
}
