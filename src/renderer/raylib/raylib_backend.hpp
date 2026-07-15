#pragma once
#include <raylib.h>
#include <vector>
#include "../render_backend.hpp"
#include "models.hpp"

namespace renderer {

class RaylibBackend : public RenderBackend {
public:
    void Init(int width, int height, const char* title) override;
    void Shutdown() override;
    bool ShouldClose() const override;

    FrameInput PollInput() override;
    float  FrameTime() const override;
    double Time() const override;

    TextureHandle LoadTexture(const char* path) override;
    MeshHandle    CreateMesh(const Mesh& m) override;
    void          DestroyMesh(MeshHandle h) override;

    void BeginFrame(RColor clear) override;
    void EndFrame() override;
    void SetClipPlanes(float near_plane, float far_plane) override;
    void Begin3D(const RCamera& cam) override;
    void End3D() override;

    ScreenPoint WorldToScreen(const RVec3& viewPos) const override;
    int ScreenWidth() const override;
    int ScreenHeight() const override;

    void DrawModel(MeshHandle h, const RMat4& model, const Material& mat) override;
    void DrawLines(const LineVertex* v, size_t count, float width) override;
    void DrawRocket(const RocketFrame& f) override;
    void DrawEarth(const EarthFrame& f) override;

    void DrawRect(int x, int y, int w, int h, RColor c) override;
    void DrawRectLines(int x, int y, int w, int h, RColor c) override;
    void DrawText(const char* text, int x, int y, int font_size, RColor c) override;
    void DrawFPS(int x, int y) override;

private:
    void ensureEarth();   // lazily build the Earth mesh + texture on first draw

    std::vector<::Mesh>      meshes_;     // handle = index + 1
    std::vector<::Texture2D> textures_;   // handle = index + 1

    ::Material defaultMat_{};   // default shader, used by DrawModel
    ::Material earthMat_{};     // earth shader, used by DrawEarth
    ::Texture2D defaultTex_{};  // raylib's 1x1 white, for untextured draws

    ::Shader earthShader_{};
    int      sunDirLoc_      = -1;
    int      earthCenterLoc_ = -1;
    int      camPosLoc_      = -1;

    // Greek-capable font for overlay text (Font ID labels + telemetry). Falls
    // back to raylib's built-in ASCII font if the TTF fails to load.
    ::Font   font_{};
    bool     haveFont_ = false;

    // Last camera handed to Begin3D, kept so WorldToScreen can project labels.
    Camera3D cam3d_{};

    // Backend-owned scene objects (the simple reference look).
    RocketModel   rocket_{};
    MeshHandle    earthMesh_ = 0;
    TextureHandle earthTex_  = 0;
};

}
