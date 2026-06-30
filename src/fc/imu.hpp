#pragma once

#include <random>
#include "types.hpp"
#include "constants.hpp"

class Rocket;

// simulated INS
class INS {
    public:
    INS() : rng(94058) {}

    // reads sim states and adds noise
    Vec3 read_INS_acc(const Rocket& r); // inertial specific force
    Vec3 read_INS_gyr(const Rocket& r); // angular velocity
    Vec3 read_INS_grav(const Rocket& r); // gravity vector only

    private:
    std::mt19937 rng;

    // nosie for sensors
    double acc_stddev = 0.001;
    double gyr_stddev = 0.0001;

    double noise(double val, double stddev) {
        std::normal_distribution<double> dist(0.0, stddev);
        return val + dist(rng);
    }

    Vec3 add_noise(Vec3 data_in, double sensor_stddev) {
        return {
            noise(data_in.x, sensor_stddev),
            noise(data_in.y, sensor_stddev),
            noise(data_in.z, sensor_stddev)
        };
    }

    // gravitational acceleration in ECI
    static Vec3 gravity_eci(Vec3 r) {
        double rn = r.norm();
        if (rn < 1.0) return {0.0, 0.0, 0.0};
        return r * (-GM_EARTH / (rn * rn * rn));
    }
};
