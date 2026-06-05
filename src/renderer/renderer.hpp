#pragma once
#include <raylib.h>
#include "../constants.hpp"
#include "../types.hpp"

namespace sim { class Sim; }

namespace renderer {

class Renderer {
public:
    Renderer(const sim::Sim& s);
    ~Renderer();
    void Run();

private:
    void HandleInput();
    Camera3D BuildCamera() const;
    void DrawFrame(const Camera3D& cam);
    void DrawEarth(const Camera3D& cam, Vector3 earthC);
    void DrawECIAxes() const;
    void DrawBodyAxes() const;
    void DrawRocket() const;
    void DrawPredictedTrajectory() const;
    void DrawTelemetry() const;

    // ECI position (m) -> shifted render-scene position (km), with the rocket
    // held at the world origin. Differencing in double before the float cast
    // kills the jitter you'd get from float at Earth-radius magnitudes. The
    // axis swap (x,y,z)->(x,z,y) matches the sphere's -90 deg X rotation, so the
    // texture and the ECI frame line up.
    Vector3 ToView(const Vec3& eci_m) const {
        Vec3 d = eci_m - p_ref_eci_;
        return { float(d.x * M_TO_KM), float(d.z * M_TO_KM), float(d.y * M_TO_KM) };
    }

    // The scene is shifted so this point (the rocket's ECI position) sits at the
    // world origin, keeping geometry inside float's precision sweet spot.
    // Sampled once per frame so camera, axes, rocket, and Earth all agree.
    Vec3 p_ref_eci_ { 0, 0, 0 };

    const sim::Sim& sim_;
    Texture2D tex{};
    Model     sphere{};
    Shader    earthShader{};
    int       sunDirLoc      = -1;
    int       earthCenterLoc = -1;
    int       camPosLoc      = -1;

    float yaw   = 0.0f;
    float pitch = 0.3f;
    float dist  = 0.05f;  // 50 m: framed on the rocket; scroll out to see the Earth
};

}
