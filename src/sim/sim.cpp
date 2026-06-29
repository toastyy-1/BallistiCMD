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
            .m_dry = 4000.0,
            .m_fuel = 117910.0,
            .isp = 296.0,
            .tip_to_end_length = 21.4,
            .CoM_dist = 11.0,
            .max_thrust = 2200000.0,
            .engine_distance = 21.4,
            .engine_gimball_range = 5.0
        };

        Stage s2 = {
            .id = 2,
            .m_dry = 2800.0,
            .m_fuel = 27200.0,
            .isp = 316.0,
            .tip_to_end_length = 9.4,
            .CoM_dist = 4.7,
            .max_thrust = 445000.0,
            .engine_distance = 9.4,
            .engine_gimball_range = 5.0
        };

        Stage payload = {
            .id = 3,
            .m_dry = 3700.0,
            .m_fuel = 0.0,
            .isp = 0.0,
            .tip_to_end_length = 3.1,
            .CoM_dist = 1.5,
            .max_thrust = 0.0,
            .engine_distance = 3.1,
            .engine_gimball_range = 0.0
        };

        rocket.set_stage(1, s1);
        rocket.set_stage(2, s2);
        rocket.set_stage(3, payload);

        rocket.set_radius(1.524);
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

            std::this_thread::sleep_for(std::chrono::duration<double>(0.02));
        }
    }

    void Sim::publish_sim_states() {
        RocketState s = rocket.get_state();
        s.t = t;
        snap.store(s, std::memory_order_release);
    }



}
