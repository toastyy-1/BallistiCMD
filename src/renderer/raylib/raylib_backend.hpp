#pragma once
#include <raylib.h>
#include <vector>
#include "../render_backend.hpp"

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

    void DrawModel(MeshHandle h, const RMat4& model, const Material& mat) override;
    void DrawLines(const LineVertex* v, size_t count, float width) override;
    void DrawEarth(MeshHandle sphere, TextureHandle tex, const RMat4& model,
                   const RVec3& sun_dir, const RVec3& earth_center,
                   const RVec3& cam_pos) override;

    void DrawRect(int x, int y, int w, int h, RColor c) override;
    void DrawRectLines(int x, int y, int w, int h, RColor c) override;
    void DrawText(const char* text, int x, int y, int font_size, RColor c) override;
    void DrawFPS(int x, int y) override;

private:
    std::vector<::Mesh>      meshes_;     // handle = index + 1
    std::vector<::Texture2D> textures_;   // handle = index + 1

    ::Material defaultMat_{};   // default shader, used by DrawModel
    ::Material earthMat_{};     // earth shader, used by DrawEarth
    ::Texture2D defaultTex_{};  // raylib's 1x1 white, for untextured draws

    ::Shader earthShader_{};
    int      sunDirLoc_      = -1;
    int      earthCenterLoc_ = -1;
    int      camPosLoc_      = -1;
};

}
