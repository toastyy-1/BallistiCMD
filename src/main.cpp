#include "renderer/renderer.hpp"
#include "renderer/raylib/raylib_backend.hpp"
#include "sim/sim.hpp"
#include "fc/fc.hpp"
#include <thread>

int main() {
    sim::Sim s;
    std::thread sim_thread([&]{ s.Run(); });
    std::thread fc_thread([&]{ flight_controller_loop(s); });

    renderer::RaylibBackend backend;
    renderer::Renderer r(backend, s);
    r.Run();

    s.Stop();
    sim_thread.join();
    fc_thread.join();
    return 0;
}
