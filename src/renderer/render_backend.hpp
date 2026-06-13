#pragma once

#include "render_types.hpp"
#include "mesh.hpp"
#include "rmath.hpp"

// Universal rendering backend interface.
//
// The rendering *logic* (camera orbit, rocket geometry, predicted trajectory,
// telemetry layout — all of renderer::Renderer) is written once against this
// interface using backend-neutral types. Concrete backends (raylib today, BGFX
// or anything else later) implement the primitives below and can be swapped in
// without touching Renderer. Pick the backend in main.cpp.
//
// The model is retained-mode (bgfx's natural shape): geometry is built on the
// CPU by the geometry builders, uploaded once via CreateMesh, and drawn with a
// model matrix + Material. Only per-frame line work (trajectory, axes, markers)
// is transient, via DrawLines. Backend-neutral value types live in
// render_types.hpp (RVec3/RColor/...), mesh.hpp (Vertex/Mesh/Material/handles),
// and rmath.hpp (RMat4).

namespace renderer {

class RenderBackend {
public:
    virtual ~RenderBackend() = default;

    // Window / lifecycle.
    virtual void Init(int width, int height, const char* title) = 0;
    virtual void Shutdown() = 0;
    virtual bool ShouldClose() const = 0;

    // Input / timing.
    virtual FrameInput PollInput() = 0;
    virtual float  FrameTime() const = 0;   // seconds since last frame
    virtual double Time() const = 0;         // seconds since Init

    // Resources. Meshes/textures are uploaded once and referenced by handle
    // (0 = invalid / none). Textures get mipmaps + trilinear filtering.
    virtual TextureHandle LoadTexture(const char* path) = 0;
    virtual MeshHandle    CreateMesh(const Mesh& m) = 0;
    virtual void          DestroyMesh(MeshHandle h) = 0;

    // Frame structure.
    virtual void BeginFrame(RColor clear) = 0;
    virtual void EndFrame() = 0;
    // Near/far clip planes. Must be set before Begin3D (they are baked into the
    // projection), which is what keeps the depth buffer usable across this
    // scene's huge scale range.
    virtual void SetClipPlanes(float near_plane, float far_plane) = 0;
    virtual void Begin3D(const RCamera& cam) = 0;
    virtual void End3D() = 0;

    // 3D drawing (between Begin3D/End3D).
    // A static mesh placed by a model matrix, shaded/blended per Material.
    virtual void DrawModel(MeshHandle h, const RMat4& model, const Material& mat) = 0;
    // A transient line list in view space (pairs of vertices), per-vertex colour.
    virtual void DrawLines(const LineVertex* v, size_t count, float width) = 0;
    // High-level: a sun-lit, textured Earth. The caller supplies the (renderer-
    // built) sphere mesh + texture and the domain inputs; each backend owns the
    // lit/atmospheric shader. sun_dir is the (view-space) direction TO the sun;
    // earth_center / cam_pos are view-space (km).
    virtual void DrawEarth(MeshHandle sphere, TextureHandle tex, const RMat4& model,
                           const RVec3& sun_dir, const RVec3& earth_center,
                           const RVec3& cam_pos) = 0;

    // 2D overlay (screen-space, drawn after End3D).
    virtual void DrawRect(int x, int y, int w, int h, RColor c) = 0;
    virtual void DrawRectLines(int x, int y, int w, int h, RColor c) = 0;
    virtual void DrawText(const char* text, int x, int y, int font_size, RColor c) = 0;
    virtual void DrawFPS(int x, int y) = 0;
};

}
