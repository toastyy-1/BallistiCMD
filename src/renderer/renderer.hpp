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
    void DrawECIAxes() const;
    void DrawBodyAxes() const;
    void DrawRocket() const;
    void DrawPredictedTrajectory() const;

    // The scene is shifted so this point (the rocket's ECI position) sits at the
    // world origin, which keeps the rocket geometry inside the precision sweet
    // spot of float vertex coords. Sampled once per frame for consistency.
    Vec3 p_ref_eci_ { 0, 0, 0 };

    const sim::Sim& sim_;
    Texture2D tex;
    Model sphere;
    float yaw   = 0.0f;
    float pitch = 0.3f;
    float dist  = EARTH_RADIUS_KM * 3.0f;
    double t    = 0.0;
};

}
