#pragma once

#include "render_types.hpp"
#include <cstdint>
#include <vector>

// Backend-neutral mesh + material data. Meshes are built on the CPU by the
// geometry builders (geometry.hpp), uploaded once to a backend (CreateMesh ->
// MeshHandle), and drawn with a model matrix + Material. Per-frame line work
// goes through LineVertex instead (no upload, view-space coords).

namespace renderer {

struct Vertex {
    RVec3  pos;
    RVec3  normal;
    float  u, v;
    RColor color;
};

struct LineVertex {
    RVec3  pos;
    RColor color;
};

struct Mesh {
    std::vector<Vertex>   verts;
    std::vector<uint32_t> idx;
};

// Opaque GPU resource handles. 0 is reserved as "invalid / none".
using MeshHandle    = std::uint32_t;
using TextureHandle = std::uint32_t;

// How a mesh is shaded/blended when drawn. Final colour is
//   vertexColor * color * (texture ? sample : 1)
// with `lit` adding a simple lambert term (off by default to match the current
// flat-shaded look).
struct Material {
    RColor        color       = kWhite;
    bool          lit         = false;
    BlendMode     blend       = BlendMode::Alpha;
    bool          depth_write = true;
    bool          cull        = true;   // back-face cull (off for open/thin geometry)
    TextureHandle texture     = 0;
};

}
