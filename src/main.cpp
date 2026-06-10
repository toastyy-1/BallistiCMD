#include "renderer/renderer.hpp"
#include "sim/sim.hpp"
#include "fc/fc.hpp"
#include <thread>

int main() {
    sim::Sim s;
    std::thread sim_thread([&]{ s.Run(); });
    std::thread fc_thread([&]{ flight_controller_loop(s); });

    renderer::Renderer r(s);
    r.Run();

    s.Stop();
    sim_thread.join();
    fc_thread.join();
    return 0;
}
