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


///////////////////////////////////////////////////////////////////////////////////////////////
// structs                                                                                   //
///////////////////////////////////////////////////////////////////////////////////////////////

// holds the latest raw measurements collected from the sensors
struct data_store {
    Vec3 a; // acceleration from the IMU
    Vec3 w; // angular velocity from the IMU
    double time; // time since start
};

// holds the rocket's staging patterns
enum mission_stage {
    STANDBY,
    ARMED,
    POWERED_ASCENT,
    UNPOWERED_ASCENT,
    STAGE_SEPARATION,
    DESCENT
};

// holds the states of controls for the rocket
struct control_states {
    mission_stage stage;
};

///////////////////////////////////////////////////////////////////////////////////////////////
// helper functions                                                                          //
///////////////////////////////////////////////////////////////////////////////////////////////

void print_telemetry(const data_store& ds, const control_states& cs) {
    const char* stage;
    switch (cs.stage) {
        case STANDBY:          stage = "STANDBY";
        case ARMED:            stage = "ARMED";
        case POWERED_ASCENT:   stage = "POWERED ASCENT";
        case UNPOWERED_ASCENT: stage = "UNPOWERED ASCENT";
        case STAGE_SEPARATION: stage = "STAGE SEP";
        case DESCENT:          stage = "DESCENT";
        default:               stage = "UNKNOWN";
    }

    std::cout << "\033[1;31m[" << "\033[1;33m" << stage
              << "\033[1;31m]"
              << "\033[0;37m  T+" << "\033[1;37m" << std::fixed << std::setprecision(1) << ds.time << "s"
              << "\033[0;37m  acc=" << "\033[1;36m" << std::setprecision(2) << ds.a.norm() << "m/s²"
              << "\033[0;37m  gyr=" << "\033[1;36m" << ds.w.norm() << "rad/s"
              << "\033[0m\n";
}

// aquires new data from the sim
void pull_new_data(const sim::Sim& sim, data_store& ds, IMU& imu) {
    ds.a = imu.read_IMU_acc();
    ds.w = imu.read_IMU_gyr();
    ds.time = sim.get_time();
}


///////////////////////////////////////////////////////////////////////////////////////////////
// flight controller loop                                                                    //
///////////////////////////////////////////////////////////////////////////////////////////////

// runs on its own thread alongside the sim
int flight_controller(sim::Sim& sim) {

    ////////////////////////////
    // init                   //
    ////////////////////////////
    IMU imu(sim);
    data_store ds;
    Rocket& r = sim.rocket;
    control_states cs = {
        .stage = STANDBY
    };

    std::this_thread::sleep_for(std::chrono::duration<double>(3));

    std::cout << "\033[1;31m" << "[ " << "\033[1;33m" << "MISSILE CONTROL" << "\033[1;31m" << " ] " << "\033[1;37m" << "LAUNCHING MISSILE" << "\033[0m" << std::endl;
    cs.stage = POWERED_ASCENT;

    ////////////////////////////
    // control loop           //
    ////////////////////////////
    int tick = 0;
    while (sim.is_running()) {
        pull_new_data(sim, ds, imu);

        if (++tick % 10 == 0)
            print_telemetry(ds, cs);

        switch (cs.stage) {
        case STANDBY:
            break;
        case ARMED:
            break;
        case POWERED_ASCENT: {
            static bool engine_lit = false;
            if (!engine_lit) {
                r.light_engine();
                engine_lit = true;
            }

            static double start = ds.time;
            static bool is_separated = false;
            if (ds.time - start > 130 && !is_separated) {
                r.advance_stage();
                is_separated = true;
                r.light_engine();
            }

            const double turn_time = 3.0;
            static double turn_time_start = ds.time;
            if ((ds.time - turn_time_start) < turn_time) {
                double half = (0.3 * M_PI / 180.0) / 2.0;
                r.set_engine_orientation( {std::cos(half), 0.0, std::sin(half), 0.0} );
            }
            else if ((ds.time - turn_time_start) < 2.0 * turn_time) {
                double half = (-0.3 * M_PI / 180.0) / 2.0;
                r.set_engine_orientation( {std::cos(half), 0.0, std::sin(half), 0.0} );
            }
            else r.set_engine_orientation({1, 0, 0, 0});
            break;
        }
        case UNPOWERED_ASCENT:
            break;
        case STAGE_SEPARATION:
            break;
        case DESCENT:
            break;
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
    }
    return 0;
}
