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

// holds the latest raw measurements collected from the sensors
struct data_store {
    Vec3 a; // acceleration from the IMU
    Vec3 w; // angular velocity from the IMU
    Vec3 o; // magnetic field from the IMU
    double time; // time since start
};

// runs on its own thread alongside the sim
int flight_controller(const sim::Sim& sim) {
    IMU imu(sim);
    data_store ds;

    while (sim.is_running()) {
        ds.a = imu.read_IMU_acc();
        ds.w = imu.read_IMU_gyr();
        ds.o = imu.read_IMU_mag();
        ds.time = sim.get_time();

        std::cout << ds.w.y << std::endl;

        std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
    }
    return 0;
}
