#include "renderer.hpp"
#include "../sim/sim.hpp"
#include <raymath.h>
#include <rlgl.h>
#include <math.h>
#include <initializer_list>

namespace renderer {

namespace {

// Rotate vector v by quaternion q (body -> ECI), Rodrigues form.
Vec3 qrot(const Quat& q, const Vec3& v) {
    Vec3 qv {q.x, q.y, q.z};
    Vec3 t = qv.cross(v);
    return v + t * (2.0 * q.w) + qv.cross(t) * 2.0;
}

double clampd(double v, double lo, double hi) {
    return fmax(lo, fmin(hi, v));
}

// Sun-lit Earth. The surface normal is rebuilt from the fragment's world
// position relative to the sphere centre, so the result never depends on
// raylib's normal-matrix convention. mvp / matModel / texture0 / colDiffuse are
// auto-bound by raylib from their standard names.
const char* kEarthVS = R"(#version 330
in vec3 vertexPosition;
in vec2 vertexTexCoord;
uniform mat4 mvp;
uniform mat4 matModel;
out vec2 fragTexCoord;
out vec3 fragWorldPos;
void main() {
    fragTexCoord = vertexTexCoord;
    fragWorldPos = (matModel*vec4(vertexPosition, 1.0)).xyz;
    gl_Position  = mvp*vec4(vertexPosition, 1.0);
}
)";

const char* kEarthFS = R"(#version 330
in vec2 fragTexCoord;
in vec3 fragWorldPos;
uniform sampler2D texture0;
uniform vec4 colDiffuse;
uniform vec3 sunDir;       // direction TO the sun (world)
uniform vec3 earthCenter;  // sphere centre (world)
uniform vec3 camPos;       // camera position (world)
out vec4 finalColor;
void main() {
    vec3 N = normalize(fragWorldPos - earthCenter);
    vec3 L = normalize(sunDir);
    vec3 V = normalize(camPos - fragWorldPos);

    vec3  albedo = texture(texture0, fragTexCoord).rgb*colDiffuse.rgb;
    float diff   = max(dot(N, L), 0.0);
    vec3  color  = albedo*(0.15 + 0.85*diff);            // ambient + lambert

    // Atmospheric limb glow: strongest at the silhouette, brighter in sunlight.
    float rim = pow(1.0 - max(dot(N, V), 0.0), 3.0);
    color += vec3(0.30, 0.50, 0.95)*rim*(0.35 + 0.65*diff);

    finalColor = vec4(color, 1.0);
}
)";

} // namespace

Renderer::Renderer(const sim::Sim& s) : sim_(s) {
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(1920, 1080, "Missile Program");
    SetWindowMonitor(0);
    SetTargetFPS(60);

    // Trilinear + mipmaps keep the Earth crisp and shimmer-free at any zoom.
    tex = LoadTexture("src/renderer/world.jpg");
    GenTextureMipmaps(&tex);
    SetTextureFilter(tex, TEXTURE_FILTER_TRILINEAR);

    // Equirectangular map: swap the generated UVs so it wraps cleanly, then spin
    // the model -90 deg about X so its poles sit on the ECI Z axis.
    // The texture's prime meridian is mounted a few degrees off the ECI +X axis,
    // so geography lands east/west of the rocket's true longitude. Shifting every
    // column by this fraction spins the map around the pole to line it up. Pure
    // longitude offset (no U flip, so the map keeps its correct handedness);
    // fine-tune against a sharp coastline if it's still a touch off.
    const float kLonOffset = 12.0f / 360.0f;  // degrees east -> UV fraction
    Mesh mesh = GenMeshSphere(EARTH_RADIUS_KM, 128, 128);
    for (int i = 0; i < mesh.vertexCount; i++) {
        float u = mesh.texcoords[i*2 + 0];
        mesh.texcoords[i*2 + 0] = mesh.texcoords[i*2 + 1] + kLonOffset;
        mesh.texcoords[i*2 + 1] = u;
    }
    UpdateMeshBuffer(mesh, 1, mesh.texcoords, mesh.vertexCount * 2 * sizeof(float), 0);
    sphere = LoadModelFromMesh(mesh);
    sphere.transform = MatrixRotateX(-PI/2);

    earthShader    = LoadShaderFromMemory(kEarthVS, kEarthFS);
    sunDirLoc      = GetShaderLocation(earthShader, "sunDir");
    earthCenterLoc = GetShaderLocation(earthShader, "earthCenter");
    camPosLoc      = GetShaderLocation(earthShader, "camPos");
    sphere.materials[0].shader = earthShader;
    sphere.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = tex;
}

Renderer::~Renderer() {
    UnloadModel(sphere);   // also releases the Earth shader and texture it owns
    CloseWindow();
}

void Renderer::Run() {
    while (!WindowShouldClose()) {
        HandleInput();
        // Sample once per frame so camera, axes, rocket, and Earth all agree.
        p_ref_eci_ = sim_.get_rocket_pos();
        UpdateThrustLevel();
        DrawFrame(BuildCamera());
    }
}

void Renderer::UpdateThrustLevel() {
    // Infer engine firing from propellant draw-down: any drop between frames
    // means the motor is lit. Ease the on/off into a level so the plume fades
    // in and out instead of popping.
    double fuel   = sim_.get_rocket_fuel();
    bool   firing = prevFuel_ >= 0.0 && fuel < prevFuel_ - 1e-9 && fuel > 0.0;
    prevFuel_     = fuel;
    float k = fminf(1.0f, GetFrameTime() * 12.0f);
    thrust_ += ((firing ? 1.0f : 0.0f) - thrust_) * k;
}

void Renderer::HandleInput() {
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 d = GetMouseDelta();
        yaw   -= d.x * 0.005f;
        pitch += d.y * 0.005f;
        pitch = Clamp(pitch, -1.5f, 1.5f);  // avoid the straight-up/down singularity
    }
    float wheel = GetMouseWheelMove();
    if (wheel != 0.0f) dist *= powf(0.9f, wheel);
    dist = Clamp(dist, 0.02f, EARTH_RADIUS_KM * 50.0f);  // 20 m .. 50 Earth radii
}

Camera3D Renderer::BuildCamera() const {
    // Orbit the world origin (where the rocket sits in the shifted scene).
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

void Renderer::DrawFrame(const Camera3D& cam) {
    // Tight, per-frame clip planes are what keep the depth buffer usable across
    // this scene's enormous scale range (an 11 m rocket beside a 6378 km Earth).
    // They MUST be set before BeginMode3D, which bakes them into the projection.
    // Near tracks the closest geometry (rocket or near Earth limb); far reaches
    // just past the planet so nothing useful is clipped while precision stays high.
    Vector3 earthC  = ToView({0, 0, 0});            // Earth centre == ECI origin
    float    toEarth = Vector3Distance(cam.position, earthC);
    float    nearGeom = fminf(toEarth - EARTH_RADIUS_KM, dist - 0.02f);
    float    nearP    = fmaxf(nearGeom * 0.5f, 0.001f);
    float    farP     = toEarth + 2.5f * EARTH_RADIUS_KM;
    rlSetClipPlanes(nearP, farP);

    BeginDrawing();
    ClearBackground(BLACK);
    BeginMode3D(cam);
        DrawEarth(cam, earthC);
        DrawECIAxes();
        DrawBodyAxes();
        DrawSurfaceMarkers();
        DrawPredictedTrajectory();
        DrawRocket();
    EndMode3D();
    DrawTelemetry();
    EndDrawing();
}

void Renderer::DrawEarth(const Camera3D& cam, Vector3 earthC) {
    // Bias the sun toward the rocket's local-up so the launch area stays lit,
    // with a fixed offset that gives a natural day/night terminator on the limb.
    Vector3 up  = Vector3Normalize(Vector3Negate(earthC));   // origin - centre
    Vector3 sun = Vector3Normalize(Vector3Add(Vector3Scale(up, 0.55f),
                                              Vector3{ 0.5f, 0.45f, 0.35f }));
    SetShaderValue(earthShader, sunDirLoc,      &sun,          SHADER_UNIFORM_VEC3);
    SetShaderValue(earthShader, earthCenterLoc, &earthC,       SHADER_UNIFORM_VEC3);
    SetShaderValue(earthShader, camPosLoc,      &cam.position, SHADER_UNIFORM_VEC3);
    DrawModel(sphere, earthC, 1.0f, WHITE);
}

void Renderer::DrawRocket() const {
    // Palette.
    const Color kBody = { 226, 229, 235, 255 };  // brushed silver
    const Color kNose = { 196,  58,  58, 255 };  // red cap
    const Color kFin  = {  74,  80,  92, 255 };  // gunmetal
    const Color kBell = {  54,  56,  62, 255 };  // dark nozzle
    const int   kSides = 24;                      // smoother than the old 16

    Quat   qr       = sim_.get_rocket_orientation();
    Quat   qe       = sim_.get_engine_orientation();
    double length   = sim_.get_rocket_length();
    double cm_dist  = sim_.get_rocket_cm_dist();
    double eng_dist = sim_.get_engine_distance();
    double radius   = sim_.get_rocket_radius();

    // Body frame in ECI: bz = nose direction, bx/by span the radial plane.
    Vec3 bz = qrot(qr, {0, 0, 1});
    Vec3 bx = qrot(qr, {1, 0, 0});
    Vec3 by = qrot(qr, {0, 1, 0});
    auto along = [&](double d) { return p_ref_eci_ + bz * (cm_dist - d); };  // d metres below the nose tip

    // Hull: cylindrical body capped by a conical nose (top 18% of the length).
    double cone_len = length * 0.18;
    Vector3 noseW   = ToView(along(0.0));
    Vector3 coneW   = ToView(along(cone_len));
    Vector3 shoulderW = ToView(along(cone_len + length * 0.04));  // thin collar under the cone
    Vector3 tailW   = ToView(along(length));
    Vector3 engineW = ToView(along(eng_dist));
    float   radiusW = float(radius * M_TO_KM);

    DrawCylinderEx(tailW, coneW, radiusW, radiusW, kSides, kBody);          // main body
    DrawCylinderEx(shoulderW, coneW, radiusW * 1.04f, radiusW, kSides, kFin); // collar
    DrawCylinderEx(coneW, noseW, radiusW, 0.0f, kSides, kNose);             // nose cone

    // Four swept tail fins. Each is a double-sided triangle so it reads from
    // any angle: leading edge up the body, trailing edge swept past the tail.
    Vec3   tail    = along(length);
    double finUp   = length * 0.16;     // how far the leading edge climbs the body
    double finOut  = radius * 2.4;      // span beyond the hull
    double finBack = length * 0.05;     // trailing-edge sweep behind the tail
    for (const Vec3& rad : { bx, by, -bx, -by }) {
        Vector3 a = ToView(tail + bz * finUp + rad * radius);
        Vector3 b = ToView(tail + rad * radius);
        Vector3 c = ToView(tail - bz * finBack + rad * (radius + finOut));
        DrawTriangle3D(a, b, c, kFin);
        DrawTriangle3D(a, c, b, kFin);  // back face
    }

    // Gimbaled engine bell: thrust = q_rocket * (q_engine * +Z); bell points opposite.
    Vec3    thrust_eci = qrot(qr, qrot(qe, {0, 0, 1}));
    Vec3    nozzle     = along(eng_dist) - thrust_eci * 1.5;     // bell exit, ECI
    DrawCylinderEx(engineW, ToView(nozzle), radiusW * 0.4f, radiusW * 1.15f, kSides, kBell);

    // Exhaust plume: nested additive cones streaming out the nozzle along the
    // exhaust (-thrust) direction, with a flicker so it looks alive. Drawn last,
    // depth-test on but depth-write off so the layers glow through each other.
    if (thrust_ > 0.02f) {
        float t     = (float)GetTime();
        float flick = 0.82f + 0.12f*sinf(t*46.0f) + 0.06f*sinf(t*71.0f + 1.7f);
        double L    = length * 0.8 * thrust_ * flick;           // plume length, metres
        auto cone = [&](double r0, double len, Color c) {
            c.a = (unsigned char)(c.a * thrust_);
            DrawCylinderEx(ToView(nozzle), ToView(nozzle - thrust_eci * len),
                           float(r0 * M_TO_KM), 0.0f, kSides, c);
        };
        rlDisableDepthMask();
        BeginBlendMode(BLEND_ADDITIVE);
        cone(radius * 1.5,  L * 1.25, { 180,  70, 20,  90 });   // outer haze
        cone(radius * 1.0,  L,        { 255, 140, 40, 140 });   // flame body
        cone(radius * 0.55, L * 0.6,  { 255, 235, 180, 210 });  // white-hot core
        DrawSphere(ToView(nozzle), radiusW * (0.7f + 0.2f*flick),
                   { 255, 190, 90, (unsigned char)(150 * thrust_) });  // nozzle glow
        EndBlendMode();
        rlEnableDepthMask();
    }
}

void Renderer::DrawPredictedTrajectory() const {
    // Where the rocket would coast if the engine cut out now: propagate the
    // current state under point-mass gravity (no thrust, no drag) with RK4.
    auto grav = [](Vec3 p) {
        double rn = p.norm();
        return p * (-GM_EARTH / (rn * rn * rn));
    };

    Vec3 r = sim_.get_rocket_pos();
    Vec3 v = sim_.get_rocket_vel();

    const double dt        = 1.0;     // s per step
    const int    max_steps = 20000;   // path-length cap

    rlSetLineWidth(2.0f);
    Vector3 prev = ToView(r);
    for (int i = 0; i < max_steps; i++) {
        Vec3 k1r = v,                k1v = grav(r);
        Vec3 k2r = v + k1v*(dt/2),   k2v = grav(r + k1r*(dt/2));
        Vec3 k3r = v + k2v*(dt/2),   k3v = grav(r + k2r*(dt/2));
        Vec3 k4r = v + k3v*dt,       k4v = grav(r + k3r*dt);
        r += (k1r + k2r*2 + k3r*2 + k4r) * (dt/6);
        v += (k1v + k2v*2 + k3v*2 + k4v) * (dt/6);

        Vector3 cur = ToView(r);
        DrawLine3D(prev, cur, YELLOW);
        prev = cur;

        if (r.norm() <= EARTH_RADIUS_M) break;      // reached the surface
    }
}

void Renderer::DrawECIAxes() const {
    // ECI axes through the Earth's centre: X vernal equinox (red),
    // Y 90E equatorial (green), Z north pole (blue).
    const double L = EARTH_RADIUS_M * 1.5;
    rlSetLineWidth(2.0f);
    DrawLine3D(ToView({-L, 0, 0}), ToView({L, 0, 0}), RED);
    DrawLine3D(ToView({0, -L, 0}), ToView({0, L, 0}), GREEN);
    DrawLine3D(ToView({0, 0, -L}), ToView({0, 0, L}), BLUE);
}

void Renderer::DrawBodyAxes() const {
    // Body triad at the rocket (world origin), scaled with zoom to stay visible.
    Quat q = sim_.get_rocket_orientation();
    const float L = dist * 0.08f;
    auto tip = [&](const Vec3& v) {
        return Vector3 { L * float(v.x), L * float(v.z), L * float(v.y) };  // ECI->world swap
    };
    rlSetLineWidth(2.0f);
    DrawLine3D({0, 0, 0}, tip(qrot(q, {1, 0, 0})), PINK);     // body X
    DrawLine3D({0, 0, 0}, tip(qrot(q, {0, 1, 0})), LIME);     // body Y
    DrawLine3D({0, 0, 0}, tip(qrot(q, {0, 0, 1})), SKYBLUE);  // body Z (nose)
}

void Renderer::DrawSurfaceMarkers() const {
    // Launch origin and intended target: fixed points on the Earth's surface.
    // Each is drawn as a small "pin" — a stalk along the local vertical capped
    // by a sphere — so it reads as a point planted on the ground and is occluded
    // by the globe when it rotates to the far side. Sized with zoom so the pins
    // stay a roughly constant apparent size from the rocket's launch pad out to
    // a full view of the planet.
    InitialStates init = sim_.rocket.get_rocket_initial_states();

    const float sphereR = dist * 0.02f;          // marker radius (km, view units)
    const float pinLen  = dist * 0.06f;          // stalk height above the surface
    rlSetLineWidth(2.0f);

    auto pin = [&](const Vec3& surf_eci, Color c) {
        double n = surf_eci.norm();
        if (n < 1e-6) return;
        Vec3    out      = surf_eci / n;                        // local vertical (ECI)
        Vec3    tip_eci  = surf_eci + out * (pinLen * KM_TO_M); // lift the cap off the ground
        Vector3 baseW    = ToView(surf_eci);
        Vector3 tipW     = ToView(tip_eci);
        DrawLine3D(baseW, tipW, c);
        DrawSphere(tipW, sphereR, c);
    };

    pin(init.origin_r_eci, GREEN);   // launch origin
    pin(init.target_r_eci, RED);     // intended target
}

void Renderer::DrawTelemetry() const {
    // --- gather raw state (all ECI, SI units) from sim ---
    Vec3 r = sim_.get_rocket_pos();
    Vec3 v = sim_.get_rocket_vel();
    Vec3 a = sim_.get_rocket_acc();
    Vec3 w = sim_.get_rocket_ang_vel();
    Quat qr = sim_.get_rocket_orientation();
    Quat qe = sim_.get_engine_orientation();
    double t    = sim_.get_time();
    double mass = sim_.get_rocket_mass();
    double fuel = sim_.get_rocket_fuel();

    // --- attitude ---
    double rmag  = r.norm();
    Vec3   up    = rmag > 1.0 ? r / rmag : Vec3{0, 0, 1};
    Vec3   nose  = qrot(qr, {0, 0, 1});
    double pitch = std::asin(clampd(nose.dot(up), -1.0, 1.0)) * RAD_TO_DEG;
    double gimbal = 2.0 * std::acos(clampd(qe.w, -1.0, 1.0)) * RAD_TO_DEG;
    Vec3   wdeg  = w * RAD_TO_DEG;

    // --- layout ---
    // Panel size is derived from how many headers/rows we draw below; keep these
    // counts in sync with the sections so the box always wraps the text exactly.
    const int x = 10, valx = 165;
    const int fs = 16, lh = 18, hh = lh + 6;  // row height, header block height
    const int nHeaders = 6, nRows = 17;
    int y = 12;
    const int panelW = 320;
    const int panelH = y + nHeaders * hh + nRows * lh + 8;
    DrawRectangle(0, 0, panelW, panelH, Fade(BLACK, 0.55f));
    DrawRectangleLines(0, 0, panelW, panelH, Fade(SKYBLUE, 0.3f));

    auto header = [&](const char* s) {
        y += 4;
        DrawText(s, x, y, fs, SKYBLUE);
        y += lh + 2;
    };
    auto row = [&](const char* label, const char* val, Color c) {
        DrawText(label, x, y, fs, GRAY);
        DrawText(val, valx, y, fs, c);
        y += lh;
    };

    header("MISSION");
    row("MET",        TextFormat("T+ %7.1f s", t),       WHITE);

    header("VEHICLE");
    row("Mass",       TextFormat("%8.1f kg", mass),      WHITE);
    row("Propellant", TextFormat("%8.1f kg", fuel),      fuel < 50.0 ? RED : GREEN);

    header("ECI POSITION (km)");
    row("X",  TextFormat("%+14.3f", r.x * M_TO_KM), WHITE);
    row("Y",  TextFormat("%+14.3f", r.y * M_TO_KM), WHITE);
    row("Z",  TextFormat("%+14.3f", r.z * M_TO_KM), WHITE);

    header("ECI VELOCITY (m/s)");
    row("Vx", TextFormat("%+14.3f", v.x), WHITE);
    row("Vy", TextFormat("%+14.3f", v.y), WHITE);
    row("Vz", TextFormat("%+14.3f", v.z), WHITE);

    header("ECI ACCELERATION (m/s^2)");
    row("Ax", TextFormat("%+14.6f", a.x), WHITE);
    row("Ay", TextFormat("%+14.6f", a.y), WHITE);
    row("Az", TextFormat("%+14.6f", a.z), WHITE);

    header("ATTITUDE");
    row("Pitch",      TextFormat("%+8.2f deg", pitch),    WHITE);
    row("Gimbal",     TextFormat("%8.2f deg", gimbal),    gimbal > 5.0 ? ORANGE : WHITE);
    row("Roll rate",  TextFormat("%+8.2f d/s", wdeg.x),   WHITE);
    row("Pitch rate", TextFormat("%+8.2f d/s", wdeg.y),   WHITE);
    row("Yaw rate",   TextFormat("%+8.2f d/s", wdeg.z),   WHITE);

    DrawFPS(panelW + 10, 12);
}

}
