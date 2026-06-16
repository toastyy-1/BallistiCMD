#pragma once

namespace renderer {

// View-space position, float (km after the rocket-centred shift.)
struct RVec3 { float x, y, z; };

struct RColor { unsigned char r, g, b, a; };

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
    // Free-fly translation axes (defaulted so existing aggregate inits stay valid).
    float move_x   = 0.0f;     // strafe:  +right (D) / -left (A)
    float move_y   = 0.0f;     // vertical:+up    (E) / -down (Q)
    float move_z   = 0.0f;     // forward: +fwd   (W) / -back (S)
    bool  boost    = false;    // hold to move faster (Left Shift)
    bool  recenter = false;    // snap the view pivot back to the rocket (F)
};

enum class BlendMode { Alpha, Additive };

// --- palette ----------------------------------------------------------------
// Raylib colours
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
