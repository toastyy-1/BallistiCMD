#include "models.hpp"
#include "../geometry.hpp"
#include "../../constants.hpp"
#include <vector>
#include <cmath>

namespace renderer {

namespace {
constexpr int kSides = 32;   // hull/bell facet count

// A quad with an explicit outward normal (lighting), auto-oriented from a rough
// outward reference. Hull/bell are drawn unculled, so winding only needs to be
// consistent, not perfect.
void quadN(Mesh& m, RVec3 a, RVec3 b, RVec3 c, RVec3 d, RVec3 outwardRef, RColor col) {
    RVec3 n = rmath::normalize(rmath::cross(rmath::sub(b, a), rmath::sub(c, a)));
    if (rmath::dot(n, outwardRef) < 0.0f) n = { -n.x, -n.y, -n.z };
    uint32_t base = (uint32_t)m.verts.size();
    m.verts.push_back({ a, n, 0, 0, col });
    m.verts.push_back({ b, n, 0, 0, col });
    m.verts.push_back({ c, n, 0, 0, col });
    m.verts.push_back({ d, n, 0, 0, col });
    m.idx.push_back(base); m.idx.push_back(base+1); m.idx.push_back(base+2);
    m.idx.push_back(base); m.idx.push_back(base+2); m.idx.push_back(base+3);
}

// A solid 3D tail fin (swept trapezoidal prism) at angular axis `rad` with
// in-plane tangent `tan`. z values are in the body frame.
void appendFin(Mesh& m, RVec3 rad, RVec3 tng, float radius, float span,
               float rootTopZ, float rootBotZ, float tipTopZ, float tipBotZ,
               float halfThick, RColor col) {
    auto P = [&](float r, float z, float s) -> RVec3 {
        return { rad.x*r + tng.x*s*halfThick, rad.y*r + tng.y*s*halfThick, z };
    };
    const float r0 = radius, r1 = radius + span;
    RVec3 RTp=P(r0,rootTopZ, 1), RTm=P(r0,rootTopZ,-1);
    RVec3 RBp=P(r0,rootBotZ, 1), RBm=P(r0,rootBotZ,-1);
    RVec3 TBp=P(r1,tipBotZ,  1), TBm=P(r1,tipBotZ, -1);
    RVec3 TTp=P(r1,tipTopZ,  1), TTm=P(r1,tipTopZ, -1);

    RVec3 upOut = rmath::normalize({ rad.x, rad.y, rad.z + 1.0f });
    RVec3 dnOut = rmath::normalize({ rad.x, rad.y, rad.z - 1.0f });
    quadN(m, RTp, RBp, TBp, TTp, tng,                       col);  // +thickness face
    quadN(m, RTm, RBm, TBm, TTm, {-tng.x,-tng.y,-tng.z},    col);  // -thickness face
    quadN(m, RTp, TTp, TTm, RTm, upOut,                     col);  // leading edge
    quadN(m, RBp, TBp, TBm, RBm, dnOut,                     col);  // trailing edge
    quadN(m, TTp, TBp, TBm, TTm, rad,                       col);  // tip edge
}

// An open, fluted (ridged) engine bell: a single-walled surface of revolution
// from throat (zT) to exit (zE), flaring outward, with longitudinal ridges.
// Drawn unculled so the concave interior is visible (the actual opening).
void appendBell(Mesh& m, float zT, float zE, float rT, float rE,
                int sides, int ribs, float amp, RColor col) {
    const int M = 14;
    uint32_t base = (uint32_t)m.verts.size();
    for (int i = 0; i <= M; ++i) {
        float t  = (float)i / M;
        float z  = zT + (zE - zT) * t;
        float Ro = rT + (rE - rT) * powf(t, 1.5f);
        float dRodz = (zE != zT) ? ((rE - rT) * 1.5f * powf(fmaxf(t,1e-4f), 0.5f)) / (zE - zT) : 0.0f;
        for (int j = 0; j <= sides; ++j) {
            float th = (float)(TAU * j / sides), cs = cosf(th), sn = sinf(th);
            float mod = 1.0f + amp * cosf(ribs * th);
            float R   = Ro * mod;
            float dRdth = Ro * (-amp * ribs * sinf(ribs * th));
            RVec3 dPt = { dRdth*cs - R*sn, dRdth*sn + R*cs, 0 };
            RVec3 dPz = { dRodz*mod*cs, dRodz*mod*sn, 1 };
            RVec3 n = rmath::normalize(rmath::cross(dPz, dPt));
            if (n.x*cs + n.y*sn < 0.0f) n = { -n.x, -n.y, -n.z };   // outward
            m.verts.push_back({ {R*cs, R*sn, z}, n, 0, 0, col });
        }
    }
    const int stride = sides + 1;
    for (int i = 0; i < M; ++i)
        for (int j = 0; j < sides; ++j) {
            uint32_t a = base + i*stride + j, b = a + 1;
            uint32_t c = base + (i+1)*stride + j, d = c + 1;
            m.idx.push_back(a); m.idx.push_back(b); m.idx.push_back(d);
            m.idx.push_back(a); m.idx.push_back(d); m.idx.push_back(c);
        }
}

} // namespace

void RocketModel::Ensure(RenderBackend& b, const RocketDims& dims) {
    if (cone_ == 0) {   // first use: build the dimension-independent meshes too
        cone_   = b.CreateMesh(geom::buildCone(kSides));
        sphere_ = b.CreateMesh(geom::buildSphere(1.0f, 16, 24));
        dims_   = dims;
        firstStage_ = true;          // booster stage: keep the fins
        buildHullBell(b);
        return;
    }
    if (dims.length == dims_.length && dims.cm_dist == dims_.cm_dist &&
        dims.radius == dims_.radius && dims.engine_dist == dims_.engine_dist)
        return;
    dims_ = dims;
    firstStage_ = false;             // staging dropped the booster: no more fins
    buildHullBell(b);
}

void RocketModel::buildHullBell(RenderBackend& b) {
    if (hull_) b.DestroyMesh(hull_);
    if (bell_) b.DestroyMesh(bell_);

    const float L      = (float)dims_.length;
    const float cm     = (float)dims_.cm_dist;
    const float radius = (float)dims_.radius;

    const RColor kBody   = { 226, 229, 235, 255 };  // brushed silver
    const RColor kNose   = { 196,  58,  58, 255 };  // red cap
    const RColor kCollar = {  74,  80,  92, 255 };  // gunmetal collar
    const RColor kFin    = {  90,  96, 108, 255 };  // gunmetal fins
    const RColor kBell   = {  46,  47,  53, 255 };  // dark nozzle

    // Body frame: +Z = nose, CM at the origin. Nose tip is fixed at z = cm; the
    // nose has a *fixed* length so the tip stays the same size across staging.
    const float nose_z   = cm;
    const float noseLen  = fminf(radius * 2.8f, 0.6f * L);
    const float nose_base = nose_z - noseLen;
    const float tail_z   = cm - L;

    Mesh hull;

    // Main body.
    geom::appendFrustum(hull, tail_z, nose_base, radius, radius, kSides, kBody);
    // Thin collar where the nose meets the body.
    geom::appendFrustum(hull, nose_base - radius * 0.18f, nose_base,
                        radius * 1.03f, radius, kSides, kCollar, false, false);

    // Smoothed (elliptical) nose, revolved from a fixed-shape profile.
    std::vector<RVec3> prof;
    const int NS = 22;
    for (int i = 0; i <= NS; ++i) {
        float t = (float)i / NS;                       // 0 base .. 1 tip
        float z = nose_base + t * noseLen;
        float r = radius * sqrtf(fmaxf(0.0f, 1.0f - t * t));
        prof.push_back({ z, r, 0 });
    }
    geom::appendRevolve(hull, prof, kSides, kNose, /*capBase=*/true);

    // Tail fins: only on the booster (first) stage, as solid swept 3D wedges.
    if (firstStage_) {
        const float span     = radius * 1.5f;
        const float rootTopZ = tail_z + L * 0.20f;
        const float rootBotZ = tail_z - radius * 0.30f;
        const float tipTopZ  = tail_z + L * 0.02f;
        const float tipBotZ  = tail_z - L * 0.06f;
        const float halfThk  = radius * 0.06f;
        const RVec3 rads[4] = { {1,0,0}, {0,1,0}, {-1,0,0}, {0,-1,0} };
        for (const RVec3& rad : rads) {
            RVec3 tng = { -rad.y, rad.x, 0 };           // in-plane perpendicular
            appendFin(hull, rad, tng, radius, span, rootTopZ, rootBotZ, tipTopZ, tipBotZ, halfThk, kFin);
        }
    }
    hull_ = b.CreateMesh(hull);

    // Engine bell: open, fluted nozzle in the gimbal-pivot frame (throat at z=0,
    // exit 1.6 m down). Throat narrow, flaring to a wide ridged exit.
    Mesh bell;
    appendBell(bell, 0.0f, -1.6f, radius * 0.45f, radius * 1.25f, kSides, 20, 0.05f, kBell);
    bell_ = b.CreateMesh(bell);
}

void RocketModel::Draw(RenderBackend& b, const RocketFrame& f) const {
    Material body;  body.lit = true; body.cull = false;   // unculled: fins + open shapes
    b.DrawModel(hull_, f.hull, body);
    Material bell;  bell.lit = true; bell.cull = false;    // unculled: concave nozzle interior
    b.DrawModel(bell_, f.bell, bell);

    if (!f.firing) return;

    // Exhaust plume: nested additive cones streaming out the nozzle along the
    // exhaust direction, depth-write off so the layers glow through each other.
    const double radius = dims_.radius;
    const double Lm     = dims_.length * 0.8 * f.thrust * f.flick;  // plume length, metres
    auto cone = [&](double r0_m, double len_m, RColor c) {
        c.a = (unsigned char)(c.a * f.thrust);
        RVec3 apex = { f.nozzle.x + f.exhaust_dir.x * (float)(len_m * M_TO_KM),
                       f.nozzle.y + f.exhaust_dir.y * (float)(len_m * M_TO_KM),
                       f.nozzle.z + f.exhaust_dir.z * (float)(len_m * M_TO_KM) };
        Material m; m.color = c; m.blend = BlendMode::Additive; m.depth_write = false;
        b.DrawModel(cone_, rmath::orientCone(f.nozzle, apex, (float)(r0_m * M_TO_KM)), m);
    };
    cone(radius * 1.5,  Lm * 1.25, { 180,  70, 20,  90 });   // outer haze
    cone(radius * 1.0,  Lm,        { 255, 140, 40, 140 });   // flame body
    cone(radius * 0.55, Lm * 0.6,  { 255, 235, 180, 210 });  // white-hot core

    // Nozzle glow.
    const float radiusW = (float)(radius * M_TO_KM);
    Material g;
    g.color = { 255, 190, 90, (unsigned char)(150 * f.thrust) };
    g.blend = BlendMode::Additive; g.depth_write = false;
    b.DrawModel(sphere_, rmath::placeSphere(f.nozzle, radiusW * (0.7f + 0.2f * f.flick)), g);
}

}
