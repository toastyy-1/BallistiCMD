#pragma once

#include "../render_backend.hpp"
#include <vector>

// bgfx's rocket model. A separate copy from raylib's (src/renderer/raylib/
// models.hpp) so the bgfx backend can evolve a richer rocket independently.
// Starts identical to the reference look; consumes the same RocketFrame.

namespace renderer {

class RocketModel {
public:
    // Select (building on first sight) the hull/bell for these dimensions. A single
    // RocketModel is shared by every rocket, so the per-frame draw loop calls this
    // once per rocket; caching by dims means each distinct stage config is built
    // once and reused, rather than rebuilt every time the loop switches between
    // rockets at different staging phases (which thrashed GPU buffers with a fleet).
    void Ensure(RenderBackend& b, const RocketDims& dims);
    void Draw(RenderBackend& b, const RocketFrame& f) const;

private:
    // Draw the detonation effect (flash -> fireball -> shockwave -> lingering smoke)
    // in place of the intact rocket, animated off f.det_time. Additive glow spheres,
    // depth-tested but not depth-writing (matches the plume).
    void drawDetonation(RenderBackend& b, const RocketFrame& f) const;

    // Build the hull + bell meshes for the given dimensions and return their handles.
    struct HullBell { RocketDims dims; MeshHandle hull; MeshHandle bell; };
    HullBell buildHullBell(RenderBackend& b, const RocketDims& d) const;

    std::vector<HullBell> cache_;   // one entry per distinct stack dimension set

    RocketDims dims_{};             // dims of the currently selected hull/bell
    MeshHandle hull_   = 0;         // currently selected (set by Ensure, read by Draw)
    MeshHandle bell_   = 0;
    MeshHandle cone_   = 0;         // unit cone, for the plume
    MeshHandle sphere_ = 0;         // unit sphere, for the nozzle glow
};

}
