#pragma once
#include <atomic>
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

        RocketState get_state() const { return snap.load(std::memory_order_acquire); }

    private:
        void publish_sim_states(const Rocket& r);

        std::atomic<RocketState> snap{RocketState{}};

        double t = 0;
        std::atomic<bool> running{true};
    };

}
