#pragma once
#include <atomic>
#include <mutex>
#include <array>
#include "types.hpp"
#include "rocket.hpp"

namespace sim {

    struct State {
        double t = 0;
        Vec3 r{}, v{}, a{}, w{};
        Quat q_rocket{1, 0, 0, 0};
        Quat q_engine{1, 0, 0, 0};
        double mass = 0, fuel = 0;
        double length = 0, cm_dist = 0, engine_dist = 0, radius = 0;
        InitialStates init{};
        std::array<Stage, Rocket::NUM_STAGES> stages{};
    };

    class Sim {
    public:

        Sim();
        ~Sim();
        void Run();
        void Stop() { running.store(false); }
        bool is_running() const { return running.load(); }

        State get_state() const { std::lock_guard<std::mutex> lk(mtx); return snap; }

        void light_engine() { ignite_cmd.store(true); }
        bool advance_stage() { advance_cmd.store(true); return true; }
        void set_engine_orientation(const Quat& q) { std::lock_guard<std::mutex> lk(mtx); gimbal_cmd = q; }

    private:
        void configure_rocket();
        void publish_sim_states();

        Rocket rocket;

        mutable std::mutex mtx;
        State snap;
        std::atomic<bool> ignite_cmd{false};
        std::atomic<bool> advance_cmd{false};
        Quat gimbal_cmd{1, 0, 0, 0};

        double t = 0;
        std::atomic<bool> running{true};

        double rocket_cm_dist = 5.0;
    };

}
