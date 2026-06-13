#pragma once

// Backend-neutral value types shared by the render interface, the geometry
// builders, and the math helpers. Kept in its own header so rmath.hpp and
// mesh.hpp can pull in the primitives (RVec3/RColor/...) without depending on
// the full RenderBackend interface (which in turn depends on them).

namespace renderer {

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

}
