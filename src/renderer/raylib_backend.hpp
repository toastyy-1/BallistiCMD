#pragma once
#include <raylib.h>
#include "render_backend.hpp"

namespace renderer {

class RaylibBackend : public RenderBackend {
public:
    void Init(int width, int height, const char* title) override;
    void Shutdown() override;
    bool ShouldClose() const override;

    FrameInput PollInput() override;
    float  FrameTime() const override;
    double Time() const override;

    void BeginFrame(RColor clear) override;
    void EndFrame() override;
    void SetClipPlanes(float near_plane, float far_plane) override;
    void Begin3D(const RCamera& cam) override;
    void End3D() override;

    void SetLineWidth(float w) override;
    void DrawLine3D(const RVec3& a, const RVec3& b, RColor c) override;
    void DrawCylinder(const RVec3& start, const RVec3& end,
                      float r_start, float r_end, int sides, RColor c) override;
    void DrawTriangle3D(const RVec3& a, const RVec3& b, const RVec3& c,
                        RColor col) override;
    void DrawSphere(const RVec3& center, float radius, RColor c) override;

    void BeginBlend(BlendMode m) override;
    void EndBlend() override;
    void SetDepthMask(bool enabled) override;

    void DrawEarth(const RVec3& center, float radius,
                   const RVec3& sun_dir, const RVec3& cam_pos) override;

    void DrawRect(int x, int y, int w, int h, RColor c) override;
    void DrawRectLines(int x, int y, int w, int h, RColor c) override;
    void DrawText(const char* text, int x, int y, int font_size, RColor c) override;
    void DrawFPS(int x, int y) override;

private:
    Texture2D tex_{};
    Model     sphere_{};
    Shader    earthShader_{};
    int       sunDirLoc_      = -1;
    int       earthCenterLoc_ = -1;
    int       camPosLoc_      = -1;
};

}
