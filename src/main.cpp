#include "renderer/renderer.hpp"
#include "sim/sim.hpp"
#include <thread>

int main() {
    sim::Sim s;
    std::thread sim_thread([&]{ s.Run(); });

    renderer::Renderer r(s);
    r.Run();

    s.Stop();
    sim_thread.join();
    return 0;
}
