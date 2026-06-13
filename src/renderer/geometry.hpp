#pragma once

#include "mesh.hpp"

// Lightweight parametric mesh builders. All coordinates are local to the part;
// the renderer places them with a model matrix (see rmath.hpp).
// UVs are only meaningful for buildSphere (the Earth).

namespace renderer::geom {

// A frustum along +Z: radius r0 at z0, radius r1 at z1 (a cylinder when
// r0==r1, a cone when one radius is 0). Caps are added at each non-degenerate
// end unless suppressed. Assumes z1 > z0. Vertices carry `col`.
void appendFrustum(Mesh& m, float z0, float z1, float r0, float r1,
                   int sides, RColor col, bool cap0 = true, bool cap1 = true);

// A single triangle with a flat normal. If doubleSided, the back face is also
// emitted so it reads from any angle (used for the tail fins).
void appendTriangle(Mesh& m, const RVec3& a, const RVec3& b, const RVec3& c,
                    RColor col, bool doubleSided = false);

// Unit cone: base circle radius 1 in the z=0 plane, apex at z=1. Recoloured per
// draw via Material; used (scaled/oriented) for the exhaust plume.
Mesh buildCone(int sides);

// UV sphere of the given radius, built in an ECI-aligned frame (+Z = north
// pole) with equirectangular texcoords (u = longitude, v = latitude from the
// north pole), so an equirectangular Earth texture maps directly. lonOffset
// shifts the texture east as a fraction of a full turn.
Mesh buildSphere(float radius, int rings, int sectors, float lonOffset = 0.0f);

}
