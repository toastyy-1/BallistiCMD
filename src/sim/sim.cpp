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

    void Sim::configure_rocket() {
        double origin_latitude  =  35.948416;
        double origin_longitude = -83.936084;
        double target_latitude  =  55.753331;
        double target_longitude =  37.616062;

        rocket.set_start(origin_latitude, origin_longitude, target_latitude, target_longitude);

        Stage s1 = {
            .id = 1,
            .m_dry = 2292.0,
            .m_fuel = 20780.0,
            .isp = 238.0,
            .tip_to_end_length = 7.49,
            .CoM_dist = 3.75,
            .max_thrust = 792000.0,
            .engine_distance = 7.49,
            .engine_gimball_range = 5.0
        };

        Stage s2 = {
            .id = 2,
            .m_dry = 1107.0,
            .m_fuel = 6170.0,
            .isp = 292.0,
            .tip_to_end_length = 4.12,
            .CoM_dist = 2.06,
            .max_thrust = 267700.0,
            .engine_distance = 4.12,
            .engine_gimball_range = 5.0
        };

        Stage payload = {
            .id = 3,
            .m_dry = 1150.0,
            .m_fuel = 0.0,
            .isp = 0.0,
            .tip_to_end_length = 1.0,
            .CoM_dist = 0.5,
            .max_thrust = 0.0,
            .engine_distance = 1.0,
            .engine_gimball_range = 0.0
        };

        rocket.set_stage(1, s1);
        rocket.set_stage(2, s2);
        rocket.set_stage(3, payload);

        rocket.set_radius(0.84);
    }

    void Sim::Run() {

        // configure the rocket for starting settings
        configure_rocket();
        publish_sim_states();

        while (running.load()) {

            ///////////////////////////////////////////////////////////////////////////////////////////////
            // sim                                                                                       //
            ///////////////////////////////////////////////////////////////////////////////////////////////

            /*
                !!! NOTE !!! THE ROCKET SHOULD NOT BE CONTROLLED FROM HERE AT ALL ASSUMING THE FC IS ACTIVE
            */

            // lets the flight controller process data to send commands to the rocket
            rocket.update_flight_controller(t);

            // update the position, orientation, and mass of the rocket
            rocket.update_mass();
            rocket.update_dynamics();
            rocket.update_rotation();

            // increment time step
            t += TIME_STEP;

            // make snapshot for other threads of sim states
            publish_sim_states();

            std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
        }
    }

    void Sim::publish_sim_states() {
        RocketState s = rocket.get_state();
        s.t = t;
        snap.store(s, std::memory_order_release);
    }



}
