#pragma once
#include "render_backend.hpp"
#include "../constants.hpp"
#include "../types.hpp"

namespace sim { class Sim; }

namespace renderer {

// Backend-agnostic renderer for providing orbit camera, the rocket,
// trajectory geometry, and telemetry.
class Renderer {
public:
    Renderer(RenderBackend& backend, const sim::Sim& s);
    ~Renderer();
    void Run();

private:
    void HandleInput();
    void UpdateThrustLevel();
    RCamera BuildCamera() const;
    void DrawFrame(const RCamera& cam);
    void DrawEarth(const RCamera& cam, RVec3 earthC);
    void DrawECIAxes() const;
    void DrawBodyAxes() const;
    void DrawSurfaceMarkers() const;
    void DrawRocket() const;
    void DrawPredictedTrajectory() const;
    void DrawTelemetry() const;

    // ECI position (m) -> shifted render-scene position (km), with the rocket
    // held at the world origin. Differencing in double before the float cast
    // kills the jitter you'd get from float at Earth-radius magnitudes. The
    // reorientation (x,y,z)->(x,z,-y) is RotateX(-90 deg): it brings ECI +Z
    // (north) to view +Y (up) as a proper rotation (det +1), so winding/normals
    // survive. Must match rmath::viewBasis exactly.
    RVec3 ToView(const Vec3& eci_m) const {
        Vec3 d = eci_m - p_ref_eci_;
        return { float(d.x * M_TO_KM), float(d.z * M_TO_KM), float(-d.y * M_TO_KM) };
    }

    // The scene is shifted so this point (the rocket's ECI position) sits at the
    // world origin, keeping geometry inside float's precision sweet spot.
    // Sampled once per frame so camera, axes, rocket, and Earth all agree.
    Vec3 p_ref_eci_ { 0, 0, 0 };

    RenderBackend&   backend_;
    const sim::Sim&  sim_;

    // Renderer-owned unit sphere (radius 1) for the surface markers.
    MeshHandle markerSphere_ = 0;

    float yaw   = 0.0f;
    float pitch = 0.3f;
    float dist  = 0.05f;  // 50 m: framed on the rocket; scroll out to see the Earth

    // Exhaust plume state. The sim exposes no "firing" flag, so we infer it from
    // propellant being burned and smooth it into a level that drives the plume.
    float  thrust_   = 0.0f;   // [0,1], ramps with the motor
    double prevFuel_ = -1.0;   // previous propellant sample (-1 = none yet)
};

}
