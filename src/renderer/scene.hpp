#pragma once

#include "render_types.hpp"
#include "rmath.hpp"

// Backend-neutral *domain* description of the scene's high-level objects. The
// renderer computes these each frame from the sim (it owns the precision-
// critical ECI->view shift).

namespace renderer {

// Static rocket dimensions (metres). Changes at staging; the backend rebuilds
// its meshes when these move.
struct RocketDims {
    double length;
    double cm_dist;
    double radius;
    double engine_dist;
};

// Everything needed to draw the rocket for one frame. Transforms are already in
// view space (the backend must not redo the ECI->view shift, or float jitter
// returns). The backend owns the actual geometry/materials.
struct RocketFrame {
    RocketDims dims;        // current stack dimensions
    RMat4 hull;            // body-local -> view
    RMat4 bell;            // bell-local (gimballed about the engine pivot) -> view
    bool  firing;          // plume visible
    float thrust;          // [0,1] plume intensity
    float flick;           // flame flicker multiplier
    float air;             // [0,1] atmospheric density (1 = sea level, 0 = vacuum)
    RVec3 nozzle;          // bell exit, view space (km)
    RVec3 exhaust_dir;     // unit direction the plume travels, view space
};

// Everything needed to draw the Earth for one frame. The backend owns the mesh,
// texture(s), and shader; the renderer supplies the placement and lighting.
struct EarthFrame {
    RMat4 model;           // ECI-metre sphere -> view
    RVec3 sun_dir;         // unit view-space direction TO the sun
    RVec3 center;          // view-space sphere centre (km)
    RVec3 cam_pos;         // view-space camera position (km)
};

}
