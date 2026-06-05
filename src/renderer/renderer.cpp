#include "renderer.hpp"
#include "../sim/sim.hpp"
#include <raymath.h>
#include <rlgl.h>
#include <math.h>

namespace renderer {

Renderer::Renderer(const sim::Sim& s) : sim_(s) {
    SetConfigFlags(FLAG_WINDOW_RESIZABLE);
    InitWindow(1920, 1080, "Missile Program");
    SetWindowMonitor(0);
    SetTargetFPS(60);

    tex = LoadTexture("src/renderer/world.jpg");
    Mesh mesh = GenMeshSphere(EARTH_RADIUS_KM, 128, 128);
    for (int i = 0; i < mesh.vertexCount; i++) {
        float u = mesh.texcoords[i*2 + 0];
        float v = mesh.texcoords[i*2 + 1];
        mesh.texcoords[i*2 + 0] = v;
        mesh.texcoords[i*2 + 1] = u;
    }
    UpdateMeshBuffer(mesh, 1, mesh.texcoords, mesh.vertexCount * 2 * sizeof(float), 0);
    sphere = LoadModelFromMesh(mesh);
    sphere.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = tex;
    sphere.transform = MatrixRotateX(-PI/2);
}

Renderer::~Renderer() {
    UnloadModel(sphere);
    UnloadTexture(tex);
    CloseWindow();
}

void Renderer::Run() {
    while (!WindowShouldClose()) {
        HandleInput();
        // Sample once per frame so camera, axes, rocket, and earth all agree.
        p_ref_eci_ = sim_.get_rocket_pos();
        Camera3D cam = BuildCamera();
        DrawFrame(cam);
    }
}

void Renderer::HandleInput() {
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 d = GetMouseDelta();
        yaw   -= d.x * 0.005f;
        pitch += d.y * 0.005f;
        if (pitch >  1.5f) pitch =  1.5f;
        if (pitch < -1.5f) pitch = -1.5f;
    }
    float wheel = GetMouseWheelMove();
    if (wheel != 0.0f) dist *= powf(0.9f, wheel);
    if (dist < 0.02f) dist = 0.02f;  // 20m min — close enough to see real-size rocket
    if (dist > EARTH_RADIUS_KM * 50.0f) dist = EARTH_RADIUS_KM * 50.0f;
}

Camera3D Renderer::BuildCamera() const {
    // Scene is rendered with the rocket at the world origin (see p_ref_eci_),
    // so the camera target is (0,0,0) and the camera offset stays small.
    return Camera3D {
        {
            dist * cosf(pitch) * sinf(yaw),
            dist * sinf(pitch),
            dist * cosf(pitch) * cosf(yaw),
        },
        { 0, 0, 0 },
        { 0, 1, 0 },
        60.0f,
        CAMERA_PERSPECTIVE,
    };
}

void Renderer::DrawRocket() const {
    // Scene is shifted so p_ref_eci_ sits at the world origin. We subtract it
    // in double precision before converting to float to avoid jitter from
    // ~7-digit float precision at ECI magnitudes (Earth radius ~6.4e6 m).
    // Sphere model is rotated -90deg about X, so ECI (x, y, z) -> world (x, z, y).
    auto eci_to_view = [this](Vec3 v_m) {
        Vec3 d = v_m - p_ref_eci_;
        return Vector3 {
            float(d.x * M_TO_KM),
            float(d.z * M_TO_KM),
            float(d.y * M_TO_KM),
        };
    };
    // Rotate vector v by quaternion q (Rodrigues form: v + 2w(q × v) + 2(q × (q × v))).
    auto qrot = [](Quat q, Vec3 v) {
        Vec3 qv {q.x, q.y, q.z};
        Vec3 t = qv.cross(v);
        return v + t * (2.0 * q.w) + qv.cross(t) * 2.0;
    };

    Quat qr  = sim_.get_rocket_orientation();
    Quat qe  = sim_.get_engine_orientation();

    double length   = sim_.get_rocket_length();
    double cm_dist  = sim_.get_rocket_cm_dist();
    double eng_dist = sim_.get_engine_distance();
    double radius   = sim_.get_rocket_radius();

    // Body +Z (nose direction) in ECI.
    Vec3 bz = qrot(qr, {0, 0, 1});

    // Body endpoints (ECI meters). Nose-cone is top 20% of body length.
    double nose_cone_len = length * 0.2;
    Vec3 nose_eci    = p_ref_eci_ + bz * cm_dist;
    Vec3 cone_base   = p_ref_eci_ + bz * (cm_dist - nose_cone_len);
    Vec3 tail_eci    = p_ref_eci_ + bz * (cm_dist - length);
    Vec3 engine_eci  = p_ref_eci_ + bz * (cm_dist - eng_dist);

    Vector3 noseW    = eci_to_view(nose_eci);
    Vector3 coneW    = eci_to_view(cone_base);
    Vector3 tailW    = eci_to_view(tail_eci);
    Vector3 engineW  = eci_to_view(engine_eci);
    float   radiusW  = float(radius * M_TO_KM);

    // Main body + nose cone.
    DrawCylinderEx(tailW, coneW, radiusW, radiusW, 16, LIGHTGRAY);
    DrawCylinderEx(coneW, noseW, radiusW, 0.0f,    16, LIGHTGRAY);

    // Engine bell: gimbaled. Thrust direction in body = q_engine rotated +Z;
    // then q_rocket carries it to ECI. Bell points opposite the thrust.
    Vec3 thrust_body = qrot(qe, {0, 0, 1});
    Vec3 thrust_eci  = qrot(qr, thrust_body);

    double bell_length = 1.5;
    Vec3 bell_end_eci  = engine_eci - thrust_eci * bell_length;
    Vector3 bellW      = eci_to_view(bell_end_eci);

    DrawCylinderEx(engineW, bellW, radiusW * 0.5f, radiusW * 1.1f, 16, ORANGE);
}

void Renderer::DrawPredictedTrajectory() const {
    // Ballistic prediction: take the rocket's current position and velocity and
    // propagate them forward under point-mass gravity only (no thrust, no drag),
    // i.e. where the rocket would coast if the engine cut out right now. The path
    // is integrated with RK4 and drawn as a polyline in the shifted scene.
    auto eci_to_view = [this](Vec3 v_m) {
        Vec3 d = v_m - p_ref_eci_;
        return Vector3 {
            float(d.x * M_TO_KM),
            float(d.z * M_TO_KM),
            float(d.y * M_TO_KM),
        };
    };
    // Gravitational acceleration at ECI position p.
    auto grav = [](Vec3 p) {
        double rn = p.norm();
        return p * (-GM_EARTH / (rn * rn * rn));
    };

    Vec3 r = sim_.get_rocket_pos();
    Vec3 v = sim_.get_rocket_vel();

    const double dt        = 1.0;     // s per integration step
    const int    max_steps = 20000;   // cap on path length

    rlSetLineWidth(2.0f);
    Vector3 prev = eci_to_view(r);
    for (int i = 0; i < max_steps; i++) {
        // RK4 step on (r, v) under gravity.
        Vec3 k1r = v;
        Vec3 k1v = grav(r);
        Vec3 k2r = v + k1v * (dt / 2);
        Vec3 k2v = grav(r + k1r * (dt / 2));
        Vec3 k3r = v + k2v * (dt / 2);
        Vec3 k3v = grav(r + k2r * (dt / 2));
        Vec3 k4r = v + k3v * dt;
        Vec3 k4v = grav(r + k3r * dt);

        r += (k1r + k2r * 2 + k3r * 2 + k4r) * (dt / 6);
        v += (k1v + k2v * 2 + k3v * 2 + k4v) * (dt / 6);

        Vector3 cur = eci_to_view(r);
        DrawLine3D(prev, cur, YELLOW);
        prev = cur;

        // Stop once the predicted path reaches the surface.
        if (r.norm() <= EARTH_RADIUS_M) break;
    }
}

void Renderer::DrawECIAxes() const {
    // ECI axes: X = vernal equinox (red), Y = 90E equatorial (green), Z = north pole (blue).
    // Sphere is drawn with a -90deg X rotation, so world Y here corresponds to ECI Z.
    // ECI origin is at (-p_ref) in the shifted scene.
    const float L  = EARTH_RADIUS_KM * 1.5f;
    const float ox = -float(p_ref_eci_.x * M_TO_KM);
    const float oy = -float(p_ref_eci_.z * M_TO_KM);
    const float oz = -float(p_ref_eci_.y * M_TO_KM);
    rlSetLineWidth(2.0f);
    DrawLine3D({ ox - L, oy,     oz     }, { ox + L, oy,     oz     }, RED);
    DrawLine3D({ ox,     oy,     oz - L }, { ox,     oy,     oz + L }, GREEN);
    DrawLine3D({ ox,     oy - L, oz     }, { ox,     oy + L, oz     }, BLUE);
}

void Renderer::DrawBodyAxes() const {
    // Body axes are the columns of the rotation matrix built from q_rocket
    // (body -> ECI). Body +Z is the nose direction.
    Quat q = sim_.get_rocket_orientation();
    double w = q.w, x = q.x, y = q.y, z = q.z;
    Vec3 bx { 1.0 - 2.0*(y*y + z*z), 2.0*(x*y + w*z),       2.0*(x*z - w*y)       };
    Vec3 by { 2.0*(x*y - w*z),       1.0 - 2.0*(x*x + z*z), 2.0*(y*z + w*x)       };
    Vec3 bz { 2.0*(x*z + w*y),       2.0*(y*z - w*x),       1.0 - 2.0*(x*x + y*y) };

    // Rocket sits at the world origin in the shifted scene.
    Vector3 origin = { 0, 0, 0 };
    // ECI (x, y, z) -> world (x, z, y), same swap used elsewhere.
    auto tipWorld = [&](const Vec3& v, float L) {
        return Vector3 {
            origin.x + L * float(v.x),
            origin.y + L * float(v.z),
            origin.z + L * float(v.y),
        };
    };
    const float L = dist * 0.08f;  // scales with zoom so triad stays visible
    rlSetLineWidth(2.0f);
    DrawLine3D(origin, tipWorld(bx, L), PINK);     // body X
    DrawLine3D(origin, tipWorld(by, L), LIME);     // body Y
    DrawLine3D(origin, tipWorld(bz, L), SKYBLUE);  // body Z (nose)
}

void Renderer::DrawFrame(const Camera3D& cam) {
    BeginDrawing();
    ClearBackground(BLACK);
    BeginMode3D(cam);
    rlSetClipPlanes(0.005, 1.0e8);  // 5m near to keep a real-size rocket visible up close
    // Earth center is the ECI origin; in the shifted scene that's -p_ref.
    Vector3 earthPos {
        -float(p_ref_eci_.x * M_TO_KM),
        -float(p_ref_eci_.z * M_TO_KM),
        -float(p_ref_eci_.y * M_TO_KM),
    };
    DrawModel(sphere, earthPos, 1.0f, WHITE);
    DrawECIAxes();
    DrawBodyAxes();
    DrawPredictedTrajectory();
    DrawRocket();
    EndMode3D();
    t = sim_.get_time();
    DrawText(TextFormat("T+  %.1f s",  (float)t), 10, 10, 20, WHITE);
    EndDrawing();
}

}
