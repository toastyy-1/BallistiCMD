#pragma once

#include "render_backend.hpp"

// Procedural scene models. Each owns its meshes/textures (built once via the
// geometry builders) and knows how to submit itself. The renderer computes the
// per-frame transforms and domain inputs (it owns ToView and the sim state) and
// hands them in; the models hold the geometry + draw recipe.

namespace renderer {

// Static rocket dimensions (metres), sampled once at build time.
struct RocketDims {
    double length;
    double cm_dist;
    double radius;
    double engine_dist;
};

// Per-frame inputs for drawing the rocket, all computed by the renderer.
struct RocketFrame {
    RMat4 hull;        // body-local -> view
    RMat4 bell;        // bell-local (gimballed about the engine pivot) -> view
    bool  firing;      // plume visible
    float thrust;      // [0,1] plume intensity
    float flick;       // flame flicker multiplier
    RVec3 nozzle;      // bell exit, view space (km)
    RVec3 exhaust_dir; // unit direction the plume travels, view space
};

// The rocket: a static hull (body + collar + nose + fins, per-vertex coloured),
// a static engine bell, and a shared unit cone/sphere reused for the additive
// exhaust plume and nozzle glow.
class RocketModel {
public:
    void Build(RenderBackend& b, const RocketDims& dims);
    // Rebuild the hull/bell meshes if the dimensions changed (e.g. staging
    // drops a section, shortening the stack). Cheap no-op when unchanged.
    void Update(RenderBackend& b, const RocketDims& dims);
    void Draw(RenderBackend& b, const RocketFrame& f) const;

    // Shared unit sphere (radius 1) — also used by the renderer for markers.
    MeshHandle unitSphere() const { return sphere_; }

private:
    // (Re)build the dimension-dependent meshes (hull + bell) from dims_,
    // releasing any previous ones.
    void buildHullBell(RenderBackend& b);

    RocketDims dims_{};
    MeshHandle hull_   = 0;
    MeshHandle bell_   = 0;
    MeshHandle cone_   = 0;   // unit cone, for plume (dimension-independent)
    MeshHandle sphere_ = 0;   // unit sphere, for glow / markers (dimension-independent)
};

// The Earth: an ECI-aligned UV sphere plus the surface texture, drawn through
// the backend's lit/atmospheric Earth path.
class EarthModel {
public:
    void Build(RenderBackend& b);
    void Draw(RenderBackend& b, const RMat4& model, const RVec3& sun_dir,
              const RVec3& earth_center, const RVec3& cam_pos) const;

    MeshHandle mesh() const { return mesh_; }

private:
    MeshHandle    mesh_ = 0;
    TextureHandle tex_  = 0;
};

}
