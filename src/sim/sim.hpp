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

        double get_time() const { return sim_time.load(); }
        Vec3 get_rocket_pos() const {
            return { rocket_x.load(), rocket_y.load(), rocket_z.load() };
        }
        Quat get_rocket_orientation() const {
            return { rocket_qw.load(), rocket_qx.load(), rocket_qy.load(), rocket_qz.load() };
        }
        Quat get_engine_orientation() const {
            return { engine_qw.load(), engine_qx.load(), engine_qy.load(), engine_qz.load() };
        }
        Vec3 get_rocket_acc() const {
            return { rocket_ax.load(), rocket_ay.load(), rocket_az.load() };
        }
        Vec3 get_rocket_ang_vel() const {
            return { rocket_wx.load(), rocket_wy.load(), rocket_wz.load() };
        }

        double get_rocket_length() const { return rocket_length; }
        double get_rocket_cm_dist() const { return rocket_cm_dist; }
        double get_engine_distance() const { return rocket_engine_dist; }
        double get_rocket_radius() const { return rocket_radius; }

    private:
        double t; // sim time
        std::atomic<bool>   running{true};
        std::atomic<double> rocket_x{0.0};
        std::atomic<double> rocket_y{0.0};
        std::atomic<double> rocket_z{0.0};
        std::atomic<double> rocket_qw{1.0};
        std::atomic<double> rocket_qx{0.0};
        std::atomic<double> rocket_qy{0.0};
        std::atomic<double> rocket_qz{0.0};
        std::atomic<double> engine_qw{1.0};
        std::atomic<double> engine_qx{0.0};
        std::atomic<double> engine_qy{0.0};
        std::atomic<double> engine_qz{0.0};
        std::atomic<double> rocket_ax{0.0};
        std::atomic<double> rocket_ay{0.0};
        std::atomic<double> rocket_az{0.0};
        std::atomic<double> rocket_wx{0.0};
        std::atomic<double> rocket_wy{0.0};
        std::atomic<double> rocket_wz{0.0};
        std::atomic<double> sim_time{0.0};

        double rocket_length = 11.25;
        double rocket_cm_dist = 5.0;
        double rocket_engine_dist = 10.5;
        double rocket_radius = 0.5;

    };

}
