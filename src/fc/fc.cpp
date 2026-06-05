#include <iostream>
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


///////////////////////////////////////////////////////////////////////////////////////////////
// helper functions                                                                          //
///////////////////////////////////////////////////////////////////////////////////////////////

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

    ////////////////////////////
    // control loop           //
    ////////////////////////////
    while (sim.is_running()) {
        pull_new_data(sim, ds, imu);

        r.apply_thrust(100);

        std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
    }
    return 0;
}
