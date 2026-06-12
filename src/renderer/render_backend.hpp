#pragma once

// Universal rendering backend interface.
//
// The rendering *logic* (camera orbit, rocket geometry, predicted trajectory,
// telemetry layout — all of renderer::Renderer) is written once against this
// interface using backend-neutral types. Concrete backends (raylib today, BGFX
// or anything else later) implement the primitives below and can be swapped in
// without touching Renderer. Pick the backend in main.cpp.

namespace renderer {

// --- backend-neutral value types -------------------------------------------

// View-space position, float (km after the rocket-centred shift; see
// Renderer::ToView). Deliberately not a raylib Vector3 so this header pulls in
// no backend dependency.
struct RVec3 { float x, y, z; };

struct RColor { unsigned char r, g, b, a; };

// Perspective camera. (Every backend here renders perspective; add a flag if an
// orthographic mode is ever needed.)
struct RCamera {
    RVec3 position;
    RVec3 target;
    RVec3 up;
    float fovy;   // vertical field of view, degrees
};

// Per-frame input snapshot, polled once at the top of each frame.
struct FrameInput {
    float mouse_dx, mouse_dy;  // mouse movement since last frame, pixels
    float wheel;               // scroll wheel delta
    bool  left_down;           // left mouse button held
};

enum class BlendMode { Alpha, Additive };

// --- palette ----------------------------------------------------------------
// Mirrors the raylib named colours the renderer used, so the visual output is
// unchanged. Backend-neutral so they live with the rest of the render types.
inline constexpr RColor kBlack   {   0,   0,   0, 255 };
inline constexpr RColor kWhite   { 255, 255, 255, 255 };
inline constexpr RColor kGray    { 130, 130, 130, 255 };
inline constexpr RColor kRed     { 230,  41,  55, 255 };
inline constexpr RColor kGreen   {   0, 228,  48, 255 };
inline constexpr RColor kBlue    {   0, 121, 241, 255 };
inline constexpr RColor kYellow  { 253, 249,   0, 255 };
inline constexpr RColor kOrange  { 255, 161,   0, 255 };
inline constexpr RColor kPink    { 255, 109, 194, 255 };
inline constexpr RColor kLime    {   0, 158,  47, 255 };
inline constexpr RColor kSkyBlue { 102, 191, 255, 255 };

// --- interface --------------------------------------------------------------

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

    // Frame structure.
    virtual void BeginFrame(RColor clear) = 0;
    virtual void EndFrame() = 0;
    // Near/far clip planes. Must be set before Begin3D (they are baked into the
    // projection), which is what keeps the depth buffer usable across this
    // scene's huge scale range.
    virtual void SetClipPlanes(float near_plane, float far_plane) = 0;
    virtual void Begin3D(const RCamera& cam) = 0;
    virtual void End3D() = 0;

    // 3D primitives.
    virtual void SetLineWidth(float w) = 0;
    virtual void DrawLine3D(const RVec3& a, const RVec3& b, RColor c) = 0;
    virtual void DrawCylinder(const RVec3& start, const RVec3& end,
                              float r_start, float r_end, int sides, RColor c) = 0;
    virtual void DrawTriangle3D(const RVec3& a, const RVec3& b, const RVec3& c,
                                RColor col) = 0;
    virtual void DrawSphere(const RVec3& center, float radius, RColor c) = 0;

    // Render state.
    virtual void BeginBlend(BlendMode m) = 0;
    virtual void EndBlend() = 0;
    virtual void SetDepthMask(bool enabled) = 0;

    // High-level: a sun-lit, textured Earth. Each backend owns its own
    // mesh/shader/texture and renders this however it likes; the caller only
    // supplies domain inputs. sun_dir is the (world) direction TO the sun.
    virtual void DrawEarth(const RVec3& center, float radius,
                           const RVec3& sun_dir, const RVec3& cam_pos) = 0;

    // 2D overlay (screen-space, drawn after End3D).
    virtual void DrawRect(int x, int y, int w, int h, RColor c) = 0;
    virtual void DrawRectLines(int x, int y, int w, int h, RColor c) = 0;
    virtual void DrawText(const char* text, int x, int y, int font_size, RColor c) = 0;
    virtual void DrawFPS(int x, int y) = 0;
};

}
