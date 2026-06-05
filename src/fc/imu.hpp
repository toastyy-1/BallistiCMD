#pragma once

#include <random>
#include "types.hpp"
#include "constants.hpp"
#include "sim/sim.hpp"

// simulated imu
class IMU {
    public:
    explicit IMU(const sim::Sim& sim) : sim(sim), rng(94058) {}

    // gets data from sim and adds noise
    Vec3 read_IMU_acc() {
        Vec3 a_eci = sim.get_rocket_acc();
        Vec3 specific_force_eci = a_eci - gravity_eci(sim.get_rocket_pos());
        return add_noise(eci_to_body(specific_force_eci), acc_stddev);
    }

    Vec3 read_IMU_gyr() {
        return add_noise(sim.get_rocket_ang_vel(), gyr_stddev);
    }

    Vec3 read_IMU_mag() {
        return add_noise(eci_to_body(mag_field_eci), mag_stddev);
    }

    private:
    const sim::Sim& sim;
    std::mt19937 rng;

    // nosie for sensors
    double acc_stddev = 0.02;
    double gyr_stddev = 0.001;
    double mag_stddev = 1.0e-7;

    Vec3 mag_field_eci = {0.0, 0.0, 50.0e-6};

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

    // rotate an ECI vector into body frame 
    Vec3 eci_to_body(Vec3 v) const {
        Quat q = sim.get_rocket_orientation();
        Quat q_conj = { q.w, -q.x, -q.y, -q.z };
        return qrot(q_conj, v);
    }

    // rotate vector v by quaternion q
    static Vec3 qrot(Quat q, Vec3 v) {
        Vec3 qv = {q.x, q.y, q.z};
        Vec3 t = qv.cross(v);
        return v + t * (2.0 * q.w) + qv.cross(t) * 2.0;
    }
};
