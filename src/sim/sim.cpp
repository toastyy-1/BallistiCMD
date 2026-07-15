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
        Rocket rocket{35.948416, -83.936084, 81.093395, -63.211146};

        // configure the rocket for starting settings
        publish_sim_states(rocket);

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
            publish_sim_states(rocket);

            std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
        }
    }

    void Sim::publish_sim_states(const Rocket& r) {
        RocketState s = r.get_state();
        s.t = t;
        snap.store(s, std::memory_order_release);
    }



}
