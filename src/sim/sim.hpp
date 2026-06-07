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

        Rocket rocket;

        // geter
        double get_time() const { return t; }
        Vec3 get_rocket_pos() const { return rocket.get_pos(); }
        Vec3 get_rocket_vel() const { return rocket.get_vel(); }
        Vec3 get_rocket_acc() const { return rocket.get_acc(); }
        Vec3 get_rocket_ang_vel() const { return rocket.get_ang_vel(); }
        Quat get_rocket_orientation() const { return rocket.get_orientation(); }
        Quat get_engine_orientation() const { return rocket.get_engine_orientation(); }
        double get_rocket_mass() const { return rocket.get_mass(); }
        double get_rocket_fuel() const { return rocket.get_fuel_mass(); }

        // static geometry
        double get_rocket_length() const { return rocket.get_length(); }
        double get_rocket_cm_dist() const { return rocket_cm_dist; }
        double get_engine_distance() const { return rocket.get_length(); }
        double get_rocket_radius() const { return rocket.get_radius(); }

    private:
        void configure_rocket();

        double t; // sim time
        std::atomic<bool>   running{true};

        double rocket_length = 11.25;
        double rocket_cm_dist = 5.0;
        double rocket_engine_dist = 10.5;
    };

}
