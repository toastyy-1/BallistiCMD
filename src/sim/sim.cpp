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
        double origin_latitude  =  35.9606;
        double origin_longitude = -83.9207;
        double target_latitude  =  38.9072;
        double target_longitude = -77.0369;

        rocket.set_start(origin_latitude, origin_longitude, target_latitude, target_longitude);

        Stage s1 = {
            .id = 1,
            .m_dry = 2292.0,
            .m_fuel = 20780.0,
            .m_flow_rate = 339.0,
            .tip_to_end_length = 7.49,
            .CM_dist = 3.75,
            .max_thrust = 792000.0,
            .engine_distance = 7.49,
            .engine_gimball_range = 5.0
        };

        Stage s2 = {
            .id = 2,
            .m_dry = 1107.0,
            .m_fuel = 6170.0,
            .m_flow_rate = 93.5,
            .tip_to_end_length = 4.12,
            .CM_dist = 2.06,
            .max_thrust = 267700.0,
            .engine_distance = 4.12,
            .engine_gimball_range = 5.0
        };

        Stage s3 = {
            .id = 3,
            .m_dry = 545.0,
            .m_fuel = 3306.0,
            .m_flow_rate = 54.0,
            .tip_to_end_length = 2.34,
            .CM_dist = 1.17,
            .max_thrust = 152000.0,
            .engine_distance = 2.34,
            .engine_gimball_range = 5.0
        };

        Stage payload = {
            .id = 4,
            .m_dry = 1150.0,
            .m_fuel = 0.0,
            .m_flow_rate = 0.0,
            .tip_to_end_length = 1.0,
            .CM_dist = 0.5,
            .max_thrust = 0.0,
            .engine_distance = 1.0,
            .engine_gimball_range = 0.0
        };

        rocket.set_stage(1, s1);
        rocket.set_stage(2, s2);
        rocket.set_stage(3, s3);
        rocket.set_stage(4, payload);

        rocket.set_radius(0.84);
    }

    void Sim::Run() {

        // configure the rocket for starting settings
        configure_rocket();

        while (running.load()) {

            ///////////////////////////////////////////////////////////////////////////////////////////////
            // sim                                                                                       //
            ///////////////////////////////////////////////////////////////////////////////////////////////

            /*
                !!! NOTE !!! THE ROCKET SHOULD NOT BE CONTROLLED FROM HERE AT ALL ASSUMING THE FC IS ACTIVE
            */

            // update the position, orientation, and mass of the rocket
            rocket.update_mass();
            rocket.update_dynamics();
            rocket.update_rotation();

            // increment time step
            t += TIME_STEP;

            std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
        }
    }



}
