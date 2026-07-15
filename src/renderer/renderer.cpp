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

RVec3 rvCross(const RVec3& a, const RVec3& b) {
    return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
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
    backend_.Init(1920, 1080, "Trajectory Sim");
    markerSphere_ = backend_.CreateMesh(geom::buildSphere(1.0f, 16, 24));
}

Renderer::~Renderer() {
    backend_.Shutdown();
}

RocketState Renderer::primaryState() const {
    if (states_.empty()) return RocketState{};
    int i = (primary_ >= 0 && primary_ < (int)states_.size()) ? primary_ : 0;
    return states_[i];
}

void Renderer::Run() {
    while (!backend_.ShouldClose()) {
        // Sample every rocket once per frame so camera, axes, rockets, Earth, and
        // the ID overlay all agree. HandleInput may re-select the primary, so it
        // runs after the sample (it reads states_.size() to wrap).
        states_ = sim_.get_state();
        HandleInput();
        p_ref_eci_ = primaryState().r;   // scene centred on the selected rocket
        UpdateThrustLevels();
        UpdateTrails();
        DrawFrame(BuildCamera());
    }
}

void Renderer::UpdateThrustLevels() {
    // Infer per-rocket engine firing from propellant draw-down: any drop between
    // frames means that motor is lit. Ease the on/off into a level so each plume
    // fades in and out instead of popping. Reset the smoothing buffers when the
    // rocket count changes (indices no longer line up).
    const size_t n = states_.size();
    if (thrustLvl_.size() != n) { thrustLvl_.assign(n, 0.0f); prevFuel_.assign(n, -1.0); }
    const float k = fminf(1.0f, backend_.FrameTime() * 12.0f);
    for (size_t i = 0; i < n; i++) {
        double fuel   = states_[i].fuel;
        bool   firing = prevFuel_[i] >= 0.0 && fuel < prevFuel_[i] - 1e-9 && fuel > 0.0;
        prevFuel_[i]  = fuel;
        thrustLvl_[i] += ((firing ? 1.0f : 0.0f) - thrustLvl_[i]) * k;
    }
}

void Renderer::UpdateTrails() {
    // Record each rocket's flown path in ECI metres. Points are added by distance
    // (not per frame) for even spatial density, and capped so a long flight can't
    // grow the buffer without bound. Recorded even while trails are hidden so the
    // full path appears the moment they're toggled on.
    const size_t n = states_.size();
    if (trails_.size() != n) trails_.assign(n, {});   // rocket count changed: reset
    const double minStep = 1000.0;   // metres between recorded points (~1 km)
    const size_t maxPts  = 8000;     // ~8000 km of trail per rocket
    for (size_t i = 0; i < n; i++) {
        Vec3  p  = states_[i].r;
        auto& tr = trails_[i];
        if (tr.empty() || (p - tr.back()).norm() >= minStep) {
            tr.push_back(p);
            if (tr.size() > maxPts) tr.erase(tr.begin(), tr.begin() + (tr.size() - maxPts));
        }
    }
}

void Renderer::HandleInput() {
    FrameInput in = backend_.PollInput();

    // Number keys toggle overlays (1 = trails, as requested; others follow).
    switch (in.toggle) {
        case 1: showTrails_         = !showTrails_;         break;
        case 2: showPredicted_      = !showPredicted_;      break;
        case 3: showBodyAxes_       = !showBodyAxes_;       break;
        case 4: showStateVectors_   = !showStateVectors_;   break;
        case 5: showSurfaceMarkers_ = !showSurfaceMarkers_; break;
        case 6: showEciAxes_        = !showEciAxes_;        break;
        case 7: showLabels_         = !showLabels_;         break;
        case 8: showTelemetry_      = !showTelemetry_;      break;
        default: break;
    }

    // Cycle the primary rocket with Tab / Shift+Tab. Snap the orbit pivot back to
    // the rocket so the view re-frames the newly selected target.
    if (!states_.empty() && (in.next_target || in.prev_target)) {
        int n = (int)states_.size();
        if (in.next_target) primary_ = (primary_ + 1) % n;
        if (in.prev_target) primary_ = (primary_ - 1 + n) % n;
        pivot_ = { 0.0f, 0.0f, 0.0f };
    }

    if (in.left_down) {
        yaw   -= in.mouse_dx * 0.005f;
        pitch += in.mouse_dy * 0.005f;
        pitch = clampf(pitch, -1.5f, 1.5f);  // avoid the straight-up/down singularity
    }
    if (in.wheel != 0.0f) dist *= powf(0.9f, in.wheel);
    dist = clampf(dist, 0.02f, EARTH_RADIUS_KM * 50.0f);  // 20 m .. 50 Earth radii

    // Free-fly: translate the orbit pivot along the camera's own axes. Speed
    // scales with the zoom distance (and FrameTime for frame-rate independence)
    // so it crawls when zoomed to the surface and covers ground fast from orbit.
    if (in.move_x != 0.0f || in.move_y != 0.0f || in.move_z != 0.0f) {
        float cy = cosf(yaw), sy = sinf(yaw), cp = cosf(pitch), sp = sinf(pitch);
        RVec3 fwd   = { -cp*sy, -sp, -cp*cy };   // eye -> pivot (matches BuildCamera)
        RVec3 right = {  cy,    0.0f, -sy   };    // camera right, horizontal
        float s = dist * (in.boost ? 4.0f : 1.0f) * backend_.FrameTime();
        // right * strafe + worldUp * lift + fwd * forward  (worldUp = {0,1,0})
        pivot_.x += (right.x*in.move_x + fwd.x*in.move_z) * s;
        pivot_.y += (in.move_y         + fwd.y*in.move_z) * s;
        pivot_.z += (right.z*in.move_x + fwd.z*in.move_z) * s;
    }
    if (in.recenter) pivot_ = { 0.0f, 0.0f, 0.0f };
}

RCamera Renderer::BuildCamera() const {
    // Orbit the pivot (the rocket at the origin by default; flown by WASD).
    return RCamera {
        {
            pivot_.x + dist * cosf(pitch) * sinf(yaw),
            pivot_.y + dist * sinf(pitch),
            pivot_.z + dist * cosf(pitch) * cosf(yaw),
        },
        pivot_,
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
        if (showEciAxes_)        DrawECIAxes();
        if (showBodyAxes_)       DrawBodyAxes();
        if (showStateVectors_)   DrawStateVectors();
        if (showSurfaceMarkers_) DrawSurfaceMarkers();
        if (showPredicted_)      DrawPredictedTrajectory();
        if (showTrails_)         DrawTrails();
        DrawRocket();
    backend_.End3D();
    if (showTelemetry_) DrawTelemetry();
    if (showLabels_)    DrawRocketLabels();
    DrawOverlayLegend();
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
    // Draw every rocket. Each is positioned at its own ECI state; the primary sits
    // at the scene origin (p_ref_eci_ == its r), the rest are offset via ToView.
    for (size_t i = 0; i < states_.size(); i++)
        DrawOneRocket(states_[i], i < thrustLvl_.size() ? thrustLvl_[i] : 0.0f);
}

void Renderer::DrawOneRocket(const RocketState& st, float thrustLevel) const {
    Quat   qr       = st.q_rocket;
    Quat   qe       = st.q_engine;
    double cm_dist  = st.cm_dist;
    double eng_dist = st.engine_dist;

    // Body axis (nose) and gimballed thrust direction, in ECI.
    Vec3 bz         = qrot(qr, {0, 0, 1});
    Vec3 thrust_eci = qrot(qr, qrot(qe, {0, 0, 1}));
    Vec3 engine_eci = st.r + bz * (cm_dist - eng_dist);   // along(engine_dist)
    Vec3 nozzle_eci = engine_eci - thrust_eci * 1.5;      // bell exit

    // Model matrices. viewBasis maps ECI metres -> view km (with the axis swap);
    // translate(ToView(st.r)) carries the (double-differenced) scene shift.
    RMat4 V   = rmath::viewBasis((float)M_TO_KM);
    RMat4 pv  = rmath::translate(ToView(st.r));
    RMat4 Rqr = rmath::fromQuat(qr.w, qr.x, qr.y, qr.z);
    RMat4 Rqe = rmath::fromQuat(qe.w, qe.x, qe.y, qe.z);

    RocketFrame f;
    f.dims = { st.length, st.cm_dist, st.radius, st.engine_dist };
    f.hull = rmath::mul(pv, rmath::mul(V, Rqr));
    // Bell pivots (gimbal) about the engine attach point in the body frame.
    f.bell = rmath::mul(pv, rmath::mul(V, rmath::mul(Rqr,
                 rmath::mul(rmath::translate({0, 0, (float)(cm_dist - eng_dist)}), Rqe))));

    float t   = (float)backend_.Time();
    f.firing  = thrustLevel > 0.02f;
    f.thrust  = thrustLevel;
    f.flick   = 0.82f + 0.12f*sinf(t*46.0f) + 0.06f*sinf(t*71.0f + 1.7f);
    // Atmospheric density factor (~8km scale height): drives Mach diamonds, which
    // only form in atmosphere (over/under-expanded nozzle), not in vacuum.
    double altitude = st.r.norm() - EARTH_RADIUS;
    f.air     = (float)exp(-fmax(altitude, 0.0) / 8000.0);
    // Aerodynamic heating ~ dynamic pressure (air * v^2): glows on ascent through
    // the dense atmosphere at speed and (much more) on reentry.
    double speed = st.v.norm();
    double q     = (double)f.air * speed * speed;   // ~ dynamic pressure (normalised air)
    f.heating = (float)fmax(0.0, fmin(1.0, (q - 2.0e4) / 4.0e5));
    f.vel_dir = rvDir(st.v);
    f.nozzle  = ToView(nozzle_eci);
    f.exhaust_dir = rvDir(-thrust_eci);

    backend_.DrawRocket(f);
}

void Renderer::DrawPredictedTrajectory() const {
    // Where each rocket would coast if its engine cut out now: propagate the
    // current state under point-mass gravity (no thrust, no drag) with RK4. The
    // primary's path is drawn bright yellow; the others a dimmer grey so the
    // selected rocket stays legible in a crowd.
    auto grav = [](Vec3 p) {
        double rn = p.norm();
        return p * (-GM_EARTH / (rn * rn * rn));
    };

    const double dt        = 1.0;     // s per step
    const int    max_steps = 20000;   // path-length cap

    std::vector<LineVertex> path;
    for (size_t idx = 0; idx < states_.size(); idx++) {
        Vec3  r = states_[idx].r;
        Vec3  v = states_[idx].v;
        RColor col = ((int)idx == primary_) ? kYellow : kGray;

        path.clear();
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
            path.push_back({ prev, col });
            path.push_back({ cur,  col });
            prev = cur;

            if (r.norm() <= EARTH_RADIUS) break;      // reached the surface
        }
        backend_.DrawLines(path.data(), path.size(), 2.0f);
    }
}

void Renderer::DrawTrails() const {
    // The actual flown path of each rocket (history), as a polyline. Primary in
    // sky-blue, others dim grey — distinct from the yellow predicted trajectory.
    std::vector<LineVertex> seg;
    for (size_t i = 0; i < trails_.size(); i++) {
        const std::vector<Vec3>& tr = trails_[i];
        if (tr.size() < 2) continue;
        RColor col = ((int)i == primary_) ? kSkyBlue : kGray;

        seg.clear();
        seg.reserve(tr.size() * 2);
        RVec3 prev = ToView(tr[0]);
        for (size_t k = 1; k < tr.size(); k++) {
            RVec3 cur = ToView(tr[k]);
            seg.push_back({ prev, col });
            seg.push_back({ cur,  col });
            prev = cur;
        }
        backend_.DrawLines(seg.data(), seg.size(), 2.0f);
    }
}

void Renderer::DrawECIAxes() const {
    // ECI axes through the Earth's centre: X vernal equinox (red),
    // Y 90E equatorial (green), Z north pole (blue).
    const double L = EARTH_RADIUS * 1.5;
    LineVertex ax[6] = {
        { ToView({-L, 0, 0}), kRed },   { ToView({L, 0, 0}), kRed },
        { ToView({0, -L, 0}), kGreen }, { ToView({0, L, 0}), kGreen },
        { ToView({0, 0, -L}), kBlue },  { ToView({0, 0, L}), kBlue },
    };
    backend_.DrawLines(ax, 6, 2.0f);
}

void Renderer::DrawBodyAxes() const {
    // Body triad at each rocket, scaled with zoom to stay visible. The triad stems
    // from the rocket's view-space position (the origin for the primary).
    const float L = dist * 0.08f;
    std::vector<LineVertex> bx;
    bx.reserve(states_.size() * 6);
    for (const RocketState& st : states_) {
        Quat  q = st.q_rocket;
        RVec3 o = ToView(st.r);
        auto tip = [&](const Vec3& v) {   // ECI->view rotation, offset to the rocket
            return RVec3 { o.x + L*float(v.x), o.y + L*float(v.z), o.z - L*float(v.y) };
        };
        bx.push_back({ o, kPink });    bx.push_back({ tip(qrot(q, {1, 0, 0})), kPink });     // body X
        bx.push_back({ o, kLime });    bx.push_back({ tip(qrot(q, {0, 1, 0})), kLime });     // body Y
        bx.push_back({ o, kSkyBlue }); bx.push_back({ tip(qrot(q, {0, 0, 1})), kSkyBlue });  // body Z (nose)
    }
    backend_.DrawLines(bx.data(), bx.size(), 2.0f);
}

void Renderer::DrawStateVectors() const {
    // Velocity and net-acceleration pointers stemming from each rocket's centre.
    // Fixed length (scaled with zoom) — they show direction, not magnitude, which
    // spans too many orders to draw to scale. Velocity is drawn a little longer
    // than acceleration so the two stay distinguishable when they nearly align
    // (e.g. thrust along the velocity vector on ascent).
    std::vector<LineVertex> lines;
    auto arrow = [&](const RVec3& o, const Vec3& dir_eci, float len, RColor c) {
        RVec3 d = rvDir(dir_eci);                       // unit direction in view space
        if (d.x == 0 && d.y == 0 && d.z == 0) return;  // degenerate (near-zero vector)
        RVec3 tip = { o.x + d.x*len, o.y + d.y*len, o.z + d.z*len };
        lines.push_back({ o,   c });                    // shaft: rocket centre -> tip
        lines.push_back({ tip, c });

        // Arrowhead: four barbs swept back from the tip along two axes both
        // perpendicular to the shaft (pick a reference not parallel to d).
        RVec3 ref  = fabsf(d.y) < 0.99f ? RVec3{0,1,0} : RVec3{1,0,0};
        RVec3 p1   = rvNorm(rvCross(d, ref));
        RVec3 p2   = rvNorm(rvCross(d, p1));
        RVec3 base = { tip.x - d.x*len*0.2f, tip.y - d.y*len*0.2f, tip.z - d.z*len*0.2f };
        const float hw = len * 0.1f;                    // barb half-spread
        for (const RVec3& p : { p1, p2 }) {
            for (float s : { 1.0f, -1.0f }) {
                RVec3 b = { base.x + p.x*hw*s, base.y + p.y*hw*s, base.z + p.z*hw*s };
                lines.push_back({ tip, c });
                lines.push_back({ b,   c });
            }
        }
    };

    for (const RocketState& st : states_) {
        RVec3 o = ToView(st.r);
        arrow(o, st.v, dist * 0.20f, kYellow);   // velocity (matches the predicted path)
        arrow(o, st.a, dist * 0.14f, kOrange);   // net acceleration (gravity + thrust + drag)
    }
    backend_.DrawLines(lines.data(), lines.size(), 2.0f);
}

void Renderer::DrawSurfaceMarkers() const {
    // Launch origin and intended target: fixed points on the Earth's surface.
    // Each is a small "pin" — a stalk along the local vertical capped by a
    // sphere — so it reads as a point planted on the ground and is occluded by
    // the globe when it rotates to the far side. Sized with zoom so the pins
    // stay a roughly constant apparent size across the zoom range. One origin
    // (green) + target (red) pin per rocket.
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

    for (const RocketState& st : states_) {
        pin(st.init.origin_r_eci, kGreen);   // launch origin
        pin(st.init.target_r_eci, kRed);     // intended target
    }
    backend_.DrawLines(stalks.data(), stalks.size(), 2.0f);
}

void Renderer::DrawTelemetry() const {
    // --- gather raw state (all ECI, SI units) from sim ---
    RocketState st = primaryState();
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
    const int nHeaders = 6, nRows = 15;
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

    std::vector<std::string> ids = rocketIds();
    const char* idStr = (primary_ >= 0 && primary_ < (int)ids.size()) ? ids[primary_].c_str() : "--";

    header("MISSION");
    row("Target ID",  fmt("%s  (%d/%d)", idStr, primary_ + 1, (int)states_.size()), kYellow);
    row("MET",        fmt("T+ %7.1f s", t),       kWhite);

    header("VEHICLE");
    row("Mass",       fmt("%8.1f kg", mass),      kWhite);
    row("Propellant", fmt("%8.1f kg", fuel),      fuel < 50.0 ? kRed : kGreen);

    header("ECI POSITION (km)");
    row("X",  fmt("%+12.3f", r.x * M_TO_KM), kWhite);
    row("Y",  fmt("%+12.3f", r.y * M_TO_KM), kWhite);
    row("Z",  fmt("%+12.3f", r.z * M_TO_KM), kWhite);
    row("ALT", fmt("%+12.3f", (r.norm() - EARTH_RADIUS) * M_TO_KM), kWhite);

    header("VELOCITY (m/s)");
    row("Speed", fmt("%12.3f", v.norm()), kWhite);

    header("ACCELERATION (m/s^2)");
    row("Accel", fmt("%12.6f", a.norm()), kWhite);

    header("ATTITUDE");
    row("Pitch",      fmt("%+8.2f deg", pitch),    kWhite);
    row("Gimbal",     fmt("%8.2f deg", gimbal),    gimbal > 5.0 ? kOrange : kWhite);
    row("Roll rate",  fmt("%+8.2f d/s", wdeg.x),   kWhite);
    row("Pitch rate", fmt("%+8.2f d/s", wdeg.y),   kWhite);
    row("Yaw rate",   fmt("%+8.2f d/s", wdeg.z),   kWhite);

    backend_.DrawFPS(panelW + 10, 12);
}

std::vector<std::string> Renderer::rocketIds() const {
    // 24 Greek letters α..ω (final sigma ς skipped) — one per 7.5° latitude band,
    // so the 180° pole-to-pole span maps exactly onto the alphabet. Literal UTF-8
    // (the backends' text path decodes UTF-8 and renders these from the atlas).
    static const char* GREEK[24] = {
        "α","β","γ","δ","ε","ζ","η","θ",  // α β γ δ ε ζ η θ
        "ι","κ","λ","μ","ν","ξ","ο","π",  // ι κ λ μ ν ξ ο π
        "ρ","σ","τ","υ","φ","χ","ψ","ω",  // ρ σ τ υ φ χ ψ ω
    };

    std::vector<std::string> ids(states_.size());
    int bandCount[24] = { 0 };
    for (size_t i = 0; i < states_.size(); i++) {
        // Launch-origin latitude (stable for the whole flight) -> band -> letter.
        Vec3   o   = states_[i].init.origin_r_eci;
        double n   = o.norm();
        double lat = n > 1e-6 ? std::asin(clampd(o.z / n, -1.0, 1.0)) * RAD_TO_DEG : 0.0;
        int    b   = (int)std::floor((lat + 90.0) / 7.5);
        if (b < 0) b = 0; else if (b > 23) b = 23;
        int ord = ++bandCount[b];   // 1-based ordinal within the band
        ids[i] = std::string(GREEK[b]) + std::to_string(ord);
    }
    return ids;
}

void Renderer::DrawRocketLabels() const {
    if (states_.empty()) return;
    std::vector<std::string> ids = rocketIds();
    const int W = backend_.ScreenWidth(), H = backend_.ScreenHeight();

    // Primary rocket ID: a prominent top-centre HUD element.
    if (primary_ >= 0 && primary_ < (int)ids.size()) {
        std::string hud = "TARGET " + ids[primary_];
        const int fs = 30;
        // No text-measure API; approximate glyph count (UTF-8 lead bytes) to centre.
        int glyphs = 0;
        for (unsigned char ch : hud) if ((ch & 0xC0) != 0x80) glyphs++;
        int approxW = (int)(glyphs * fs * 0.55f);
        backend_.DrawText(hud.c_str(), W / 2 - approxW / 2, 14, fs, kYellow);
    }

    // Floating labels for every other rocket, at its projected screen position.
    for (size_t i = 0; i < states_.size(); i++) {
        if ((int)i == primary_) continue;
        ScreenPoint sp = backend_.WorldToScreen(ToView(states_[i].r));
        if (!sp.visible) continue;
        if (sp.x < 0 || sp.x > (float)W || sp.y < 0 || sp.y > (float)H) continue;
        const int fs = 18;
        backend_.DrawText(ids[i].c_str(), (int)sp.x + 8, (int)sp.y - fs - 6, fs, kWhite);
    }
}

void Renderer::DrawOverlayLegend() const {
    // Bottom-left legend of the number-key toggles. Doubles as documentation: an
    // entry is lit (lime) when its overlay is on, dim (grey) when off.
    const struct { int key; const char* name; bool on; } items[] = {
        { 1, "Trails",       showTrails_ },
        { 2, "Predicted",    showPredicted_ },
        { 3, "Body axes",    showBodyAxes_ },
        { 4, "Vectors",      showStateVectors_ },
        { 5, "Pins",         showSurfaceMarkers_ },
        { 6, "ECI axes",     showEciAxes_ },
        { 7, "Labels",       showLabels_ },
        { 8, "Telemetry",    showTelemetry_ },
    };
    const int n  = (int)(sizeof(items) / sizeof(items[0]));
    const int fs = 14, lh = 16;
    int y = backend_.ScreenHeight() - lh * n - 8;
    for (const auto& it : items) {
        backend_.DrawText(fmt("%d  %s", it.key, it.name), 10, y, fs, it.on ? kLime : kGray);
        y += lh;
    }
}

}
