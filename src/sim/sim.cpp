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
        std::vector<Rocket> rocket_list = {};

        Rocket r1{35.948416, -83.936084, 81.093395, -63.211146};
        Rocket r2{-35.948416, -83.936084, 81.093395, -63.211146};

        rocket_list.push_back(r1);
        rocket_list.push_back(r2);

        // configure the rocket for starting settings
        publish_sim_states(rocket_list);

        while (running.load()) {

            ///////////////////////////////////////////////////////////////////////////////////////////////
            // sim                                                                                       //
            ///////////////////////////////////////////////////////////////////////////////////////////////

            /*
                !!! NOTE !!! THE ROCKET SHOULD NOT BE CONTROLLED FROM HERE AT ALL ASSUMING THE FC IS ACTIVE
            */

            // for each rocket, perform sim calculations individually for each one through the list until every one is updated independently of one another
            for (size_t i = 0; i < rocket_list.size(); i++) {
                // lets the flight controller process data to send commands to the rocket
                rocket_list[i].update_flight_controller(t);

                // update the position, orientation, and mass of the rocket
                rocket_list[i].update_mass();
                rocket_list[i].update_dynamics();
                rocket_list[i].update_rotation();
            }

            // increment time step
            t += TIME_STEP;

            // make snapshot for other threads of sim states
            publish_sim_states(rocket_list);

            std::this_thread::sleep_for(std::chrono::duration<double>(0.01));
        }
    }

    void Sim::publish_sim_states(const std::vector<Rocket>& rocket_list) {
        std::vector<RocketState> states;
        states.reserve(rocket_list.size());

        for (size_t i = 0; i < rocket_list.size(); i++) {
            RocketState s = rocket_list[i].get_state();
            s.t = t;
            states.push_back(s);
        }

        std::lock_guard<std::mutex> lk(rocket_states_mutex);
        rocket_states = std::move(states);
    }



}
