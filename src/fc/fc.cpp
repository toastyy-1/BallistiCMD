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
    Vec3 o; // magnetic field from the IMU
    double time; // time since start
};

// holds the rocket's staging patterns
enum mission_stage {
    STANDBY,
    ARMED,
    POWERED_ASCENT,
    UNPOWERED_ASCENT,
    STAGE_SEPARATION,
    DECENT
};

// holds the states of controls for the rocket
struct control_states {
    mission_stage stage;
};

///////////////////////////////////////////////////////////////////////////////////////////////
// helper functions                                                                          //
///////////////////////////////////////////////////////////////////////////////////////////////

static const char* stage_name(mission_stage s) {
    switch (s) {
        case STANDBY:          return "STANDBY";
        case ARMED:            return "ARMED";
        case POWERED_ASCENT:   return "POWERED ASCENT";
        case UNPOWERED_ASCENT: return "UNPOWERED ASCENT";
        case STAGE_SEPARATION: return "STAGE SEP";
        case DECENT:           return "DESCENT";
        default:               return "UNKNOWN";
    }
}

void print_telemetry(const data_store& ds, const control_states& cs) {
    std::cout << "\033[1;31m[" << "\033[1;33m" << stage_name(cs.stage)
              << "\033[1;31m]"
              << "\033[0;37m  T+" << "\033[1;37m" << std::fixed << std::setprecision(1) << ds.time << "s"
              << "\033[0;37m  acc=" << "\033[1;36m" << std::setprecision(2) << ds.a.norm() << "m/s²"
              << "\033[0;37m  gyr=" << "\033[1;36m" << ds.w.norm() << "rad/s"
              << "\033[0;37m  mag=" << "\033[1;36m" << ds.o.norm() << "μT"
              << "\033[0m\n";
}

// aquires new data from the sim
void pull_new_data(const sim::Sim& sim, data_store& ds, IMU& imu) {
    ds.a = imu.read_IMU_acc();
    ds.w = imu.read_IMU_gyr();
    ds.o = imu.read_IMU_mag();
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
        case POWERED_ASCENT:
            r.activate_engine(100);
            break;
        case UNPOWERED_ASCENT:
            break;
        case STAGE_SEPARATION:
            break;
        case DECENT:
            break;
        }

        std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
    }
    return 0;
}
