#include "models.hpp"
#include "geometry.hpp"
#include "../constants.hpp"

namespace renderer {

namespace {
constexpr int kSides = 24;   // hull/bell facet count (smoother than the old 16)
}

void RocketModel::Build(RenderBackend& b, const RocketDims& dims) {
    dims_ = dims;
    cone_   = b.CreateMesh(geom::buildCone(kSides));
    sphere_ = b.CreateMesh(geom::buildSphere(1.0f, 16, 24));
    buildHullBell(b);
}

void RocketModel::Update(RenderBackend& b, const RocketDims& dims) {
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

    // Body frame: +Z = nose, CM at the origin. Z of a point `d` metres below the
    // nose tip is (cm_dist - d), matching the old `along()` helper.
    const float cone_len   = L * 0.18f;
    const float nose_z     = cm;                          // d = 0
    const float cone_z     = cm - cone_len;
    const float shoulder_z = cm - (cone_len + L * 0.04f);
    const float tail_z     = cm - L;

    const RColor kBody = { 226, 229, 235, 255 };  // brushed silver
    const RColor kNose = { 196,  58,  58, 255 };  // red cap
    const RColor kFin  = {  74,  80,  92, 255 };  // gunmetal
    const RColor kBell = {  54,  56,  62, 255 };  // dark nozzle

    Mesh hull;
    geom::appendFrustum(hull, tail_z, cone_z, radius, radius, kSides, kBody);            // main body
    geom::appendFrustum(hull, shoulder_z, cone_z, radius * 1.04f, radius, kSides, kFin); // collar
    geom::appendFrustum(hull, cone_z, nose_z, radius, 0.0f, kSides, kNose, true, false); // nose cone

    // Four swept tail fins, each a double-sided triangle (leading edge up the
    // body, trailing edge swept past the tail).
    const float finUp   = L * 0.16f;
    const float finBack = L * 0.05f;
    const float finOut  = radius * 2.4f;
    const RVec3 rads[4] = { {1,0,0}, {0,1,0}, {-1,0,0}, {0,-1,0} };
    for (const RVec3& rad : rads) {
        RVec3 a = { rad.x*radius,          rad.y*radius,          tail_z + finUp };
        RVec3 bb = { rad.x*radius,          rad.y*radius,          tail_z };
        RVec3 c = { rad.x*(radius+finOut), rad.y*(radius+finOut), tail_z - finBack };
        geom::appendTriangle(hull, a, bb, c, kFin, /*doubleSided=*/true);
    }
    hull_ = b.CreateMesh(hull);

    // Engine bell in a frame centred on the gimbal pivot (+Z = nose,
    // un-gimballed): throat at z=0 (r=0.4), exit 1.5 m down at z=-1.5 (r=1.15).
    Mesh bell;
    geom::appendFrustum(bell, -1.5f, 0.0f, radius * 1.15f, radius * 0.4f, kSides, kBell);
    bell_ = b.CreateMesh(bell);
}

void RocketModel::Draw(RenderBackend& b, const RocketFrame& f) const {
    Material solid;  // white tint, alpha, depth-write on, unlit — shows vertex colours
    b.DrawModel(hull_, f.hull, solid);
    b.DrawModel(bell_, f.bell, solid);

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

void EarthModel::Build(RenderBackend& b) {
    // Sphere in metres (radius EARTH_RADIUS_M) so the renderer's view basis
    // (metres -> km) applies to it like everything else. The 12 deg east offset
    // matches the texture alignment the raylib backend used.
    const float kLonOffset = 12.0f / 360.0f;
    mesh_ = b.CreateMesh(geom::buildSphere((float)EARTH_RADIUS_M, 128, 128, kLonOffset));
    tex_  = b.LoadTexture("src/renderer/world.jpg");
}

void EarthModel::Draw(RenderBackend& b, const RMat4& model, const RVec3& sun_dir,
                      const RVec3& earth_center, const RVec3& cam_pos) const {
    b.DrawEarth(mesh_, tex_, model, sun_dir, earth_center, cam_pos);
}

}
