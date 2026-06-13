#include "renderer.hpp"
#include "geometry.hpp"
#include "../sim/sim.hpp"
#include <cmath>
#include <cstdio>
#include <cstdarg>
#include <vector>

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

float clampf(float v, float lo, float hi) {
    return fmaxf(lo, fminf(hi, v));
}

float rvDist(const RVec3& a, const RVec3& b) {
    float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return sqrtf(dx*dx + dy*dy + dz*dz);
}

RVec3 rvNorm(const RVec3& v) {
    float n = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
    if (n < 1e-9f) return { 0, 0, 0 };
    return { v.x/n, v.y/n, v.z/n };
}

// An ECI direction, expressed as a unit vector in view space. Mirrors ToView's
// (x,y,z)->(x,z,-y) reorientation; the metre->km scale drops out under normalize.
RVec3 rvDir(const Vec3& d) {
    return rvNorm(RVec3{ (float)d.x, (float)d.z, (float)-d.y });
}

// printf into a reusable buffer, for telemetry rows. Each value is formatted and
// drawn before the next call, so a single rotating buffer is safe here.
const char* fmt(const char* f, ...) {
    static thread_local char buf[128];
    va_list ap;
    va_start(ap, f);
    vsnprintf(buf, sizeof buf, f, ap);
    va_end(ap);
    return buf;
}

} // namespace

Renderer::Renderer(RenderBackend& backend, const sim::Sim& s)
    : backend_(backend), sim_(s) {
    backend_.Init(1920, 1080, "Missile Program");
    markerSphere_ = backend_.CreateMesh(geom::buildSphere(1.0f, 16, 24));
}

Renderer::~Renderer() {
    backend_.Shutdown();
}

void Renderer::Run() {
    while (!backend_.ShouldClose()) {
        HandleInput();
        // Sample once per frame so camera, axes, rocket, and Earth all agree.
        p_ref_eci_ = sim_.get_state().r;
        UpdateThrustLevel();
        DrawFrame(BuildCamera());
    }
}

void Renderer::UpdateThrustLevel() {
    // Infer engine firing from propellant draw-down: any drop between frames
    // means the motor is lit. Ease the on/off into a level so the plume fades
    // in and out instead of popping.
    double fuel   = sim_.get_state().fuel;
    bool   firing = prevFuel_ >= 0.0 && fuel < prevFuel_ - 1e-9 && fuel > 0.0;
    prevFuel_     = fuel;
    float k = fminf(1.0f, backend_.FrameTime() * 12.0f);
    thrust_ += ((firing ? 1.0f : 0.0f) - thrust_) * k;
}

void Renderer::HandleInput() {
    FrameInput in = backend_.PollInput();
    if (in.left_down) {
        yaw   -= in.mouse_dx * 0.005f;
        pitch += in.mouse_dy * 0.005f;
        pitch = clampf(pitch, -1.5f, 1.5f);  // avoid the straight-up/down singularity
    }
    if (in.wheel != 0.0f) dist *= powf(0.9f, in.wheel);
    dist = clampf(dist, 0.02f, EARTH_RADIUS_KM * 50.0f);  // 20 m .. 50 Earth radii
}

RCamera Renderer::BuildCamera() const {
    // Orbit the world origin (where the rocket sits in the shifted scene).
    return RCamera {
        {
            dist * cosf(pitch) * sinf(yaw),
            dist * sinf(pitch),
            dist * cosf(pitch) * cosf(yaw),
        },
        { 0, 0, 0 },
        { 0, 1, 0 },
        60.0f,
    };
}

void Renderer::DrawFrame(const RCamera& cam) {
    // Tight, per-frame clip planes are what keep the depth buffer usable across
    // this scene's enormous scale range (an 11 m rocket beside a 6378 km Earth).
    // They MUST be set before Begin3D, which bakes them into the projection.
    // Near tracks the closest geometry (rocket or near Earth limb); far reaches
    // just past the planet so nothing useful is clipped while precision stays high.
    RVec3 earthC   = ToView({0, 0, 0});            // Earth centre == ECI origin
    float toEarth  = rvDist(cam.position, earthC);
    float nearGeom = fminf(toEarth - EARTH_RADIUS_KM, dist - 0.02f);
    float nearP    = fmaxf(nearGeom * 0.5f, 0.001f);
    float farP     = toEarth + 2.5f * EARTH_RADIUS_KM;
    backend_.SetClipPlanes(nearP, farP);

    backend_.BeginFrame(kBlack);
    backend_.Begin3D(cam);
        DrawEarth(cam, earthC);
        DrawECIAxes();
        DrawBodyAxes();
        DrawSurfaceMarkers();
        DrawPredictedTrajectory();
        DrawRocket();
    backend_.End3D();
    DrawTelemetry();
    backend_.EndFrame();
}

void Renderer::DrawEarth(const RCamera& cam, RVec3 earthC) {
    // Bias the sun toward the rocket's local-up so the launch area stays lit,
    // with a fixed offset that gives a natural day/night terminator on the limb.
    RVec3 up  = rvNorm({ -earthC.x, -earthC.y, -earthC.z });   // origin - centre
    RVec3 sun = rvNorm({ up.x*0.55f + 0.5f,
                         up.y*0.55f + 0.45f,
                         up.z*0.55f + 0.35f });
    // Earth mesh is in ECI metres (+Z = north); place it at the shifted centre
    // and apply the metre->km view basis. The differencing for earthC happened
    // in double inside ToView, so float precision is fine from here on.
    EarthFrame f;
    f.model   = rmath::mul(rmath::translate(earthC), rmath::viewBasis((float)M_TO_KM));
    f.sun_dir = sun;
    f.center  = earthC;
    f.cam_pos = cam.position;
    backend_.DrawEarth(f);
}

void Renderer::DrawRocket() const {
    sim::State st = sim_.get_state();
    Quat   qr       = st.q_rocket;
    Quat   qe       = st.q_engine;
    double cm_dist  = st.cm_dist;
    double eng_dist = st.engine_dist;

    // Body axis (nose) and gimballed thrust direction, in ECI.
    Vec3 bz         = qrot(qr, {0, 0, 1});
    Vec3 thrust_eci = qrot(qr, qrot(qe, {0, 0, 1}));
    Vec3 engine_eci = p_ref_eci_ + bz * (cm_dist - eng_dist);   // along(engine_dist)
    Vec3 nozzle_eci = engine_eci - thrust_eci * 1.5;            // bell exit

    // Model matrices. viewBasis maps ECI metres -> view km (with the axis swap);
    // translate(ToView(p_ref)) carries the (double-differenced) scene shift.
    RMat4 V   = rmath::viewBasis((float)M_TO_KM);
    RMat4 pv  = rmath::translate(ToView(p_ref_eci_));
    RMat4 Rqr = rmath::fromQuat(qr.w, qr.x, qr.y, qr.z);
    RMat4 Rqe = rmath::fromQuat(qe.w, qe.x, qe.y, qe.z);

    RocketFrame f;
    f.dims = { st.length, st.cm_dist, st.radius, st.engine_dist };
    f.hull = rmath::mul(pv, rmath::mul(V, Rqr));
    // Bell pivots (gimbal) about the engine attach point in the body frame.
    f.bell = rmath::mul(pv, rmath::mul(V, rmath::mul(Rqr,
                 rmath::mul(rmath::translate({0, 0, (float)(cm_dist - eng_dist)}), Rqe))));

    float t   = (float)backend_.Time();
    f.firing  = thrust_ > 0.02f;
    f.thrust  = thrust_;
    f.flick   = 0.82f + 0.12f*sinf(t*46.0f) + 0.06f*sinf(t*71.0f + 1.7f);
    // Atmospheric density factor (~8km scale height): drives Mach diamonds, which
    // only form in atmosphere (over/under-expanded nozzle), not in vacuum.
    double altitude = st.r.norm() - EARTH_RADIUS_M;
    f.air     = (float)exp(-fmax(altitude, 0.0) / 8000.0);
    f.nozzle  = ToView(nozzle_eci);
    f.exhaust_dir = rvDir(-thrust_eci);

    backend_.DrawRocket(f);
}

void Renderer::DrawPredictedTrajectory() const {
    // Where the rocket would coast if the engine cut out now: propagate the
    // current state under point-mass gravity (no thrust, no drag) with RK4.
    auto grav = [](Vec3 p) {
        double rn = p.norm();
        return p * (-GM_EARTH / (rn * rn * rn));
    };

    sim::State st = sim_.get_state();
    Vec3 r = st.r;
    Vec3 v = st.v;

    const double dt        = 1.0;     // s per step
    const int    max_steps = 20000;   // path-length cap

    std::vector<LineVertex> path;
    path.reserve(max_steps * 2);
    RVec3 prev = ToView(r);
    for (int i = 0; i < max_steps; i++) {
        Vec3 k1r = v,                k1v = grav(r);
        Vec3 k2r = v + k1v*(dt/2),   k2v = grav(r + k1r*(dt/2));
        Vec3 k3r = v + k2v*(dt/2),   k3v = grav(r + k2r*(dt/2));
        Vec3 k4r = v + k3v*dt,       k4v = grav(r + k3r*dt);
        r += (k1r + k2r*2 + k3r*2 + k4r) * (dt/6);
        v += (k1v + k2v*2 + k3v*2 + k4v) * (dt/6);

        RVec3 cur = ToView(r);
        path.push_back({ prev, kYellow });
        path.push_back({ cur,  kYellow });
        prev = cur;

        if (r.norm() <= EARTH_RADIUS_M) break;      // reached the surface
    }
    backend_.DrawLines(path.data(), path.size(), 2.0f);
}

void Renderer::DrawECIAxes() const {
    // ECI axes through the Earth's centre: X vernal equinox (red),
    // Y 90E equatorial (green), Z north pole (blue).
    const double L = EARTH_RADIUS_M * 1.5;
    LineVertex ax[6] = {
        { ToView({-L, 0, 0}), kRed },   { ToView({L, 0, 0}), kRed },
        { ToView({0, -L, 0}), kGreen }, { ToView({0, L, 0}), kGreen },
        { ToView({0, 0, -L}), kBlue },  { ToView({0, 0, L}), kBlue },
    };
    backend_.DrawLines(ax, 6, 2.0f);
}

void Renderer::DrawBodyAxes() const {
    // Body triad at the rocket (world origin), scaled with zoom to stay visible.
    Quat q = sim_.get_state().q_rocket;
    const float L = dist * 0.08f;
    auto tip = [&](const Vec3& v) {
        return RVec3 { L * float(v.x), L * float(v.z), -L * float(v.y) };  // ECI->view rotation
    };
    LineVertex bx[6] = {
        { {0,0,0}, kPink },    { tip(qrot(q, {1, 0, 0})), kPink },     // body X
        { {0,0,0}, kLime },    { tip(qrot(q, {0, 1, 0})), kLime },     // body Y
        { {0,0,0}, kSkyBlue }, { tip(qrot(q, {0, 0, 1})), kSkyBlue },  // body Z (nose)
    };
    backend_.DrawLines(bx, 6, 2.0f);
}

void Renderer::DrawSurfaceMarkers() const {
    // Launch origin and intended target: fixed points on the Earth's surface.
    // Each is a small "pin" — a stalk along the local vertical capped by a
    // sphere — so it reads as a point planted on the ground and is occluded by
    // the globe when it rotates to the far side. Sized with zoom so the pins
    // stay a roughly constant apparent size across the zoom range.
    InitialStates init = sim_.get_state().init;

    const float sphereR = dist * 0.02f;          // marker radius (km, view units)
    const float pinLen  = dist * 0.06f;          // stalk height above the surface

    std::vector<LineVertex> stalks;
    auto pin = [&](const Vec3& surf_eci, RColor c) {
        double n = surf_eci.norm();
        if (n < 1e-6) return;
        Vec3  out     = surf_eci / n;                          // local vertical (ECI)
        Vec3  tip_eci = surf_eci + out * (pinLen * KM_TO_M);   // lift the cap off the ground
        RVec3 baseW   = ToView(surf_eci);
        RVec3 tipW    = ToView(tip_eci);
        stalks.push_back({ baseW, c });
        stalks.push_back({ tipW,  c });
        Material m; m.color = c;
        backend_.DrawModel(markerSphere_, rmath::placeSphere(tipW, sphereR), m);
    };

    pin(init.origin_r_eci, kGreen);   // launch origin
    pin(init.target_r_eci, kRed);     // intended target
    backend_.DrawLines(stalks.data(), stalks.size(), 2.0f);
}

void Renderer::DrawTelemetry() const {
    // --- gather raw state (all ECI, SI units) from sim ---
    sim::State st = sim_.get_state();
    Vec3 r = st.r;
    Vec3 v = st.v;
    Vec3 a = st.a;
    Vec3 w = st.w;
    Quat qr = st.q_rocket;
    Quat qe = st.q_engine;
    double t    = st.t;
    double mass = st.mass;
    double fuel = st.fuel;

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
    backend_.DrawRect(0, 0, panelW, panelH, { 0, 0, 0, 140 });               // Fade(BLACK, 0.55)
    backend_.DrawRectLines(0, 0, panelW, panelH, { 102, 191, 255, 76 });     // Fade(SKYBLUE, 0.3)

    auto header = [&](const char* s) {
        y += 4;
        backend_.DrawText(s, x, y, fs, kSkyBlue);
        y += lh + 2;
    };
    auto row = [&](const char* label, const char* val, RColor c) {
        backend_.DrawText(label, x, y, fs, kGray);
        backend_.DrawText(val, valx, y, fs, c);
        y += lh;
    };

    header("MISSION");
    row("MET",        fmt("T+ %7.1f s", t),       kWhite);

    header("VEHICLE");
    row("Mass",       fmt("%8.1f kg", mass),      kWhite);
    row("Propellant", fmt("%8.1f kg", fuel),      fuel < 50.0 ? kRed : kGreen);

    header("ECI POSITION (km)");
    row("X",  fmt("%+14.3f", r.x * M_TO_KM), kWhite);
    row("Y",  fmt("%+14.3f", r.y * M_TO_KM), kWhite);
    row("Z",  fmt("%+14.3f", r.z * M_TO_KM), kWhite);

    header("ECI VELOCITY (m/s)");
    row("Vx", fmt("%+14.3f", v.x), kWhite);
    row("Vy", fmt("%+14.3f", v.y), kWhite);
    row("Vz", fmt("%+14.3f", v.z), kWhite);

    header("ECI ACCELERATION (m/s^2)");
    row("Ax", fmt("%+14.6f", a.x), kWhite);
    row("Ay", fmt("%+14.6f", a.y), kWhite);
    row("Az", fmt("%+14.6f", a.z), kWhite);

    header("ATTITUDE");
    row("Pitch",      fmt("%+8.2f deg", pitch),    kWhite);
    row("Gimbal",     fmt("%8.2f deg", gimbal),    gimbal > 5.0 ? kOrange : kWhite);
    row("Roll rate",  fmt("%+8.2f d/s", wdeg.x),   kWhite);
    row("Pitch rate", fmt("%+8.2f d/s", wdeg.y),   kWhite);
    row("Yaw rate",   fmt("%+8.2f d/s", wdeg.z),   kWhite);

    backend_.DrawFPS(panelW + 10, 12);
}

}
