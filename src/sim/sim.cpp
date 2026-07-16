#include "sim/sim.hpp"
#include "sim/rocket.hpp"
#include "types.hpp"
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>
#include "constants.hpp"
#include <random>


namespace sim {

    Sim::Sim() {
        t = 0.0; // start at time 0
    }
    Sim::~Sim() {}

    void Sim::Run() {
        // array that holds all the rockets
        std::vector<Rocket> rocket_list = {};

        ///////////////////////////////////////////////////////////////////////////////////////////////
        // rocket placement process                                                                  //
        ///////////////////////////////////////////////////////////////////////////////////////////////
        const double origin_center_lat = 48.209;
        const double origin_center_long = -101.406;
        const double possible_launch_site_radius = 100; //km -- the radius by which rocket launch sites will be generated randomly

        const double target_center_lat = 53.9;
        const double target_center_long = 43.3;

        std::mt19937 rng(1201);
        std::uniform_real_distribution<double> ro(-0.01, 0.01);
        std::uniform_real_distribution<double> rt(-0.2, 0.2);

        for (int i = 0; i < 150; i++) {
            double start_lat_gen = origin_center_lat * (1 + ro(rng));
            double start_long_gen = origin_center_long * (1 + ro(rng));

            double target_lat_gen = target_center_lat * (1 + rt(rng));
            double target_long_gen = target_center_long * (1 + 4.0 * rt(rng));

            Rocket new_rocket{start_lat_gen, start_long_gen, target_lat_gen, target_long_gen};
            rocket_list.push_back(new_rocket);
        }

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
