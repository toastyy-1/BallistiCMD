#include "sim/sim.hpp"
#include "sim/rocket.hpp"
#include "types.hpp"
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>
#include "constants.hpp"

namespace sim {

    Sim::Sim() {
        t = 0.0; // start at time 0
    }
    Sim::~Sim() {}

    void Sim::Run() {
        rocket.set_pos({0, 0, EARTH_RADIUS_M});
        rocket.set_dry_mass(5900.0);
        rocket.set_fuel_mass(1000.0);
        rocket.set_mass_flow_rate(53);
        rocket.set_nose_to_engine_length(rocket_length);
        rocket.set_CM_dist(rocket_cm_dist);
        rocket.set_moment_of_inertia(62000.0);
        rocket.set_drag_coeff(0.3);
        rocket.set_max_thrust(131000.0);
        rocket.set_engine_dist(rocket_engine_dist);
        rocket.set_engine_gimball_range(5.0);
        rocket.set_engine_orientation({1, 0, 0, 0});

        while (running.load()) {

            ///////////////////////////////////////////////////////////////////////////////////////////////
            // sim                                                                                       //
            ///////////////////////////////////////////////////////////////////////////////////////////////

            /*
                !!! NOTE !!! THE ROCKET SHOULD NOT BE CONTROLLED FROM HERE AT ALL ASSUMING THE FC IS ACTIVE
            */

            // update the position, orientation, and mass of the rocket
            rocket.update_dynamics();
            rocket.update_rotation();
            rocket.update_mass();

            // increment time step
            t += TIME_STEP;

            ///////////////////////////////////////////////////////////////////////////////////////////////
            // pass shit to renderer and fc                                                              //
            ///////////////////////////////////////////////////////////////////////////////////////////////

            // store the rocket's position so that the renderer can access it
            Vec3 p = rocket.get_pos();
            rocket_x.store(p.x);
            rocket_y.store(p.y);
            rocket_z.store(p.z);
            Vec3 vel = rocket.get_vel();
            rocket_vx.store(vel.x);
            rocket_vy.store(vel.y);
            rocket_vz.store(vel.z);
            Quat q = rocket.get_orientation();
            rocket_qw.store(q.w);
            rocket_qx.store(q.x);
            rocket_qy.store(q.y);
            rocket_qz.store(q.z);
            Quat qe = rocket.get_engine_orientation();
            engine_qw.store(qe.w);
            engine_qx.store(qe.x);
            engine_qy.store(qe.y);
            engine_qz.store(qe.z);

            // store kinematics the IMU samples
            Vec3 a = rocket.get_acc();
            rocket_ax.store(a.x);
            rocket_ay.store(a.y);
            rocket_az.store(a.z);
            Vec3 wv = rocket.get_ang_vel();
            rocket_wx.store(wv.x);
            rocket_wy.store(wv.y);
            rocket_wz.store(wv.z);
            sim_time.store(t);

            std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
        }
    }



}
