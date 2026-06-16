#include "models.hpp"
#include "../geometry.hpp"
#include "../../constants.hpp"
#include <vector>
#include <cmath>

namespace renderer {

namespace {
constexpr int kSides = 32;   // hull/bell facet count

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
        buildHullBell(b);
        return;
    }
    if (dims.length == dims_.length && dims.cm_dist == dims_.cm_dist &&
        dims.radius == dims_.radius && dims.engine_dist == dims_.engine_dist)
        return;
    dims_ = dims;
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
    const RColor kBell   = {  46,  47,  53, 255 };  // dark nozzle

    // Body frame: +Z = nose, CM at the origin. Nose tip is fixed at z = cm; the
    // nose has a *fixed* length so the tip stays the same size across staging.
    const float nose_z   = cm;
    const float noseLen  = fminf(radius * 2.8f, 0.6f * L);
    const float nose_base = nose_z - noseLen;
    const float tail_z   = cm - L;

    // PBR params (metallic, roughness) are packed into vertex UVs per part.
    auto setMR = [](Mesh& m, size_t from, float metal, float rough) {
        for (size_t i = from; i < m.verts.size(); ++i) { m.verts[i].u = metal; m.verts[i].v = rough; }
    };

    Mesh hull;
    size_t k;

    // Main body — glossy white paint: mostly diffuse with only a mild reflection
    // (low metallic so the white albedo dominates instead of acting as a mirror).
    k = hull.verts.size();
    geom::appendFrustum(hull, tail_z, nose_base, radius, radius, kSides, kBody);
    setMR(hull, k, 0.12f, 0.30f);

    // Thin collar where the nose meets the body.
    k = hull.verts.size();
    geom::appendFrustum(hull, nose_base - radius * 0.18f, nose_base,
                        radius * 1.03f, radius, kSides, kCollar, false, false);
    setMR(hull, k, 0.80f, 0.45f);

    // Smoothed (elliptical) nose — painted dielectric.
    std::vector<RVec3> prof;
    const int NS = 22;
    for (int i = 0; i <= NS; ++i) {
        float t = (float)i / NS;                       // 0 base .. 1 tip
        float z = nose_base + t * noseLen;
        float r = radius * sqrtf(fmaxf(0.0f, 1.0f - t * t));
        prof.push_back({ z, r, 0 });
    }
    k = hull.verts.size();
    geom::appendRevolve(hull, prof, kSides, kNose, /*capBase=*/true);
    setMR(hull, k, 0.05f, 0.5f);

    hull_ = b.CreateMesh(hull);

    // Engine bell: open, fluted nozzle in the gimbal-pivot frame (throat at z=0,
    // exit 1.6 m down). Throat narrow, flaring to a wide ridged exit. Dark, rough metal.
    Mesh bell;
    appendBell(bell, 0.0f, -1.6f, radius * 0.45f, radius * 1.25f, kSides, 20, 0.05f, kBell);
    setMR(bell, 0, 0.55f, 0.55f);
    bell_ = b.CreateMesh(bell);
}

void RocketModel::Draw(RenderBackend& b, const RocketFrame& f) const {
    Material body;  body.lit = true; body.cull = false;   // unculled: open collar + nozzle shapes
    b.DrawModel(hull_, f.hull, body);
    Material bell;  bell.lit = true; bell.cull = false;    // unculled: concave nozzle interior
    b.DrawModel(bell_, f.bell, bell);

    // Ablation sheath: re-draw the hull slightly inflated as an additive glow
    // shell, so the plasma follows the rocket's shape (windward incandescence +
    // a bulge ahead of the nose) instead of a sphere. Intensity comes from
    // u_heat in the shader; shown on ascent and (brightly) on reentry.
    if (f.heating > 0.03f) {
        Material heat; heat.emissive = true; heat.depth_write = false;
        b.DrawModel(hull_, rmath::mul(f.hull, rmath::scale(1.05f)), heat);
    }

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
    cone(radius * 0.30, Lm * 0.32, { 205, 225, 255, 220 });  // blue-white inner jet

    // Mach diamonds: periodic shock nodes along the supersonic exhaust. They form
    // only in atmosphere (over/under-expanded nozzle), so fade with altitude
    // (f.air) and down the plume. In vacuum (f.air -> 0) they vanish.
    if (f.air > 0.12f) {
        const int   nD      = 5;
        const float spacing = (float)(radius * 1.7) * (0.15f + 0.5f * (1.0f - f.air));  // metres
        for (int k = 1; k <= nD; ++k) {
            float dist = spacing * k;
            if (dist > Lm * 0.95) break;
            float fade = 1.0f - (float)(k - 1) / nD;
            float br   = f.air * f.thrust * fade;
            RVec3 c = { f.nozzle.x + f.exhaust_dir.x * (float)(dist * M_TO_KM),
                        f.nozzle.y + f.exhaust_dir.y * (float)(dist * M_TO_KM),
                        f.nozzle.z + f.exhaust_dir.z * (float)(dist * M_TO_KM) };
            float sz = (float)(radius * 0.45 * M_TO_KM) * (0.6f + 0.4f * fade);
            Material d;
            d.color = { 215, 230, 255, (unsigned char)(220.0f * br) };
            d.blend = BlendMode::Additive; d.depth_write = false;
            b.DrawModel(sphere_, rmath::placeSphere(c, sz), d);
        }
    }

    // Nozzle glow.
    const float radiusW = (float)(radius * M_TO_KM);
    Material g;
    g.color = { 255, 190, 90, (unsigned char)(150 * f.thrust) };
    g.blend = BlendMode::Additive; g.depth_write = false;
    b.DrawModel(sphere_, rmath::placeSphere(f.nozzle, radiusW * (0.7f + 0.2f * f.flick)), g);
}

}
