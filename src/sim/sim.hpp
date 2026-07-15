#pragma once
#include <atomic>
#include <mutex>
#include <vector>
#include "types.hpp"
#include "rocket.hpp"

namespace sim {

    class Sim {
    public:

        Sim();
        ~Sim();
        void Run();
        void Stop() { running.store(false); }
        bool is_running() const { return running.load(); }

        RocketState get_state() const {
            std::lock_guard<std::mutex> lk(rocket_states_mutex);
            if (rocket_states.empty()) {
                return RocketState{};
            }
            else return rocket_states[0];
        }

    private:
        void publish_sim_states(const std::vector<Rocket>& rocket_list);

        mutable std::mutex rocket_states_mutex;
        std::vector<RocketState> rocket_states;

        double t = 0;
        std::atomic<bool> running{true};
    };

}
