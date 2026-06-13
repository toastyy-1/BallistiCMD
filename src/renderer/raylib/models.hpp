#pragma once

#include "../render_backend.hpp"

namespace renderer {

class RocketModel {
public:
    // Build on first use and rebuilt staging.
    void Ensure(RenderBackend& b, const RocketDims& dims);
    void Draw(RenderBackend& b, const RocketFrame& f) const;

private:
    void buildHullBell(RenderBackend& b);

    RocketDims dims_{};
    MeshHandle hull_   = 0;
    MeshHandle bell_   = 0;
    MeshHandle cone_   = 0;   // unit cone, for the plume
    MeshHandle sphere_ = 0;   // unit sphere, for the nozzle glow
};

}
