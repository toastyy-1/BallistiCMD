#pragma once

#include "../render_backend.hpp"

// bgfx's rocket model. A separate copy from raylib's (src/renderer/raylib/
// models.hpp) so the bgfx backend can evolve a richer rocket independently.
// Starts identical to the reference look; consumes the same RocketFrame.

namespace renderer {

class RocketModel {
public:
    // Build on first use, and rebuild the hull/bell when the stack dimensions
    // change (staging). Cheap no-op once built and unchanged.
    void Ensure(RenderBackend& b, const RocketDims& dims);
    void Draw(RenderBackend& b, const RocketFrame& f) const;

private:
    void buildHullBell(RenderBackend& b);

    RocketDims dims_{};
    bool       firstStage_ = true;   // fins only on the booster (first) stage
    MeshHandle hull_   = 0;
    MeshHandle bell_   = 0;
    MeshHandle cone_   = 0;   // unit cone, for the plume
    MeshHandle sphere_ = 0;   // unit sphere, for the nozzle glow
};

}
