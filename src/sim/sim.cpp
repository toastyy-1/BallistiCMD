#include "sim/sim.hpp"
#include "sim/rocket.hpp"
#include "types.hpp"
#include <cmath>
#include <iostream>
#include <thread>
#include <chrono>
#include "constants.hpp"
#include <random>
#include <fstream>


namespace sim {

    Sim::Sim() {
        t = 0.0; // start at time 0
    }
    Sim::~Sim() {}

    void Sim::Run(std::function<bool()> renderer_ready) {
        // wait for the renderer to load terrain before placing rockets
        while (running.load() && renderer_ready && !renderer_ready()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        // array that holds all the rockets
        std::vector<Rocket> rocket_list = {};

        ///////////////////////////////////////////////////////////////////////////////////////////////
        // rocket placement process                                                                  //
        ///////////////////////////////////////////////////////////////////////////////////////////////
        const double origin_center_lat = 48.209;
        const double origin_center_long = -101.406;

        const double target_center_lat = 53.9;
        const double target_center_long = 43.3;

        std::mt19937 rng(1201);
        std::uniform_real_distribution<double> ro(-0.2, 0.2);
        std::uniform_real_distribution<double> rt(-0.2, 0.2);

        for (int i = 0; i < 10; i++) {
            double start_lat_gen = origin_center_lat * (1 + ro(rng));
            double start_long_gen = origin_center_long * (1 + ro(rng));

            double target_lat_gen = target_center_lat; /** (1 + rt(rng));*/
            double target_long_gen = target_center_long; /** (1 + 4.0 * rt(rng));*/

            Rocket new_rocket{start_lat_gen, start_long_gen, target_lat_gen, target_long_gen};
            rocket_list.push_back(new_rocket);
        }

        // configure the rocket for starting settings
        publish_sim_states(rocket_list);

        // snapshots for renderer when its ready
        using wall_clock = std::chrono::steady_clock;
        auto next_publish = wall_clock::now();

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
                rocket_list[i].update_dynamics(t);
                rocket_list[i].update_rotation();
            }

            // increment time step
            t += TIME_STEP;

            // make snapshot for other threads of sim states
            if (wall_clock::now() >= next_publish) {
                publish_sim_states(rocket_list);
                next_publish = wall_clock::now() + std::chrono::milliseconds(10);
            }

            std::this_thread::sleep_for(std::chrono::duration<double>(0.00001));
        }

        publish_sim_states(rocket_list); // final states
        write_results_csv(rocket_list);
    }

    void Sim::write_results_csv(const std::vector<Rocket>& rocket_list) {
        std::ofstream file("landing_errors.csv");
        file << "rocket,x,y,z,error_m\n";

        for (size_t i = 0; i < rocket_list.size(); i++) {
            RocketState s = rocket_list[i].get_state();
            Vec3 pos_ecef = eci_to_ecef(s.r, t);
            double error = (pos_ecef - s.init.target_r_ecef).norm();
            file << i << "," << pos_ecef.x << "," << pos_ecef.y << "," << pos_ecef.z << "," << error << "\n";
        }
    }

    void Sim::publish_sim_states(const std::vector<Rocket>& rocket_list) {
        scratch_states.clear();
        scratch_states.reserve(rocket_list.size());

        for (size_t i = 0; i < rocket_list.size(); i++) {
            RocketState s = rocket_list[i].get_state();
            s.t = t;
            scratch_states.push_back(s);
        }

        std::lock_guard<std::mutex> lk(rocket_states_mutex);
        std::swap(rocket_states, scratch_states);
    }



}
