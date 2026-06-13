#pragma once

#include "render_types.hpp"
#include <cmath>

// Backend-neutral 4x4 float matrices, column-major (OpenGL / bgfx convention:
// element at row r, column c lives at m[c*4 + r], translation in m[12..14]).
// This memory order is byte-compatible with raylib's Matrix, so the raylib
// backend copies it field-for-field.
//
// All transforms here operate in *view space* (km, the rocket-centred shifted
// scene) unless noted. The precision-critical ECI subtraction stays in double
// inside Renderer::ToView; matrices only ever see the small local offsets.

namespace renderer {

struct RMat4 { float m[16]; };

namespace rmath {

inline RMat4 identity() {
    return RMat4{{1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1}};
}

// Column-major multiply: result = a * b (apply b first, then a).
inline RMat4 mul(const RMat4& a, const RMat4& b) {
    RMat4 o{};
    for (int c = 0; c < 4; ++c)
        for (int r = 0; r < 4; ++r) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k) s += a.m[k*4 + r] * b.m[c*4 + k];
            o.m[c*4 + r] = s;
        }
    return o;
}

inline RMat4 translate(const RVec3& t) {
    RMat4 o = identity();
    o.m[12] = t.x; o.m[13] = t.y; o.m[14] = t.z;
    return o;
}

inline RMat4 scale(float x, float y, float z) {
    RMat4 o = identity();
    o.m[0] = x; o.m[5] = y; o.m[10] = z;
    return o;
}
inline RMat4 scale(float s) { return scale(s, s, s); }

// Rotation from a (body->ECI) quaternion. Components are normalized first.
inline RMat4 fromQuat(double w, double x, double y, double z) {
    double n = std::sqrt(w*w + x*x + y*y + z*z);
    if (n < 1e-12) return identity();
    w /= n; x /= n; y /= n; z /= n;
    float xx = float(x*x), yy = float(y*y), zz = float(z*z);
    float xy = float(x*y), xz = float(x*z), yz = float(y*z);
    float wx = float(w*x), wy = float(w*y), wz = float(w*z);
    RMat4 o = identity();
    o.m[0] = 1 - 2*(yy+zz); o.m[1] = 2*(xy+wz);     o.m[2]  = 2*(xz-wy);
    o.m[4] = 2*(xy-wz);     o.m[5] = 1 - 2*(xx+zz); o.m[6]  = 2*(yz+wx);
    o.m[8] = 2*(xz+wy);     o.m[9] = 2*(yz-wx);     o.m[10] = 1 - 2*(xx+yy);
    return o;
}

// The fixed linear part of Renderer::ToView: ECI-metres -> view-km with the
// metre->km scale and a +Z(north)->+Y(up) reorientation. This is a proper
// rotation (RotateX(-90 deg), determinant +1) so it preserves triangle winding
// and normals; a bare axis *swap* would be a reflection and turn every mesh
// inside out. Must stay in lockstep with Renderer::ToView. No translation (the
// shift is applied separately, in double, via translate(ToView(P))).
inline RMat4 viewBasis(float metresToKm) {
    RMat4 o{};
    o.m[0]  = metresToKm;   // out.x =  s * in.x
    o.m[6]  = -metresToKm;  // out.z = -s * in.y
    o.m[9]  = metresToKm;   // out.y =  s * in.z
    o.m[15] = 1.0f;
    return o;
}

// --- view-space placement helpers (operate on already-shifted km coords) -----

inline RVec3 sub(const RVec3& a, const RVec3& b) { return { a.x-b.x, a.y-b.y, a.z-b.z }; }
inline float dot(const RVec3& a, const RVec3& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
inline RVec3 cross(const RVec3& a, const RVec3& b) {
    return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}
inline float length(const RVec3& v) { return std::sqrt(dot(v, v)); }
inline RVec3 normalize(const RVec3& v) {
    float n = length(v);
    return n < 1e-9f ? RVec3{0,0,0} : RVec3{ v.x/n, v.y/n, v.z/n };
}

// Map a unit sphere (radius 1, centred at origin) onto a sphere of the given
// radius at the given view-space centre.
inline RMat4 placeSphere(const RVec3& center, float radius) {
    return mul(translate(center), scale(radius));
}

// Map a unit cone (base circle radius 1 in the z=0 plane, apex at z=1) onto a
// cone with its base at `base`, apex at `apex`, and the given base radius.
inline RMat4 orientCone(const RVec3& base, const RVec3& apex, float baseRadius) {
    RVec3 axis = sub(apex, base);
    float len  = length(axis);
    RVec3 zdir = len < 1e-9f ? RVec3{0,0,1} : RVec3{ axis.x/len, axis.y/len, axis.z/len };
    // Any perpendicular basis around zdir.
    RVec3 ref  = std::fabs(zdir.y) < 0.99f ? RVec3{0,1,0} : RVec3{1,0,0};
    RVec3 xdir = normalize(cross(ref, zdir));
    RVec3 ydir = cross(zdir, xdir);
    RMat4 o{};
    o.m[0] = xdir.x*baseRadius; o.m[1] = xdir.y*baseRadius; o.m[2]  = xdir.z*baseRadius;
    o.m[4] = ydir.x*baseRadius; o.m[5] = ydir.y*baseRadius; o.m[6]  = ydir.z*baseRadius;
    o.m[8] = zdir.x*len;        o.m[9] = zdir.y*len;        o.m[10] = zdir.z*len;
    o.m[12] = base.x; o.m[13] = base.y; o.m[14] = base.z; o.m[15] = 1.0f;
    return o;
}

} // namespace rmath
} // namespace renderer
