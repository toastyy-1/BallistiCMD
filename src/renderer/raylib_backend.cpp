#include "raylib_backend.hpp"
#include "../constants.hpp"
#include <raymath.h>
#include <rlgl.h>

namespace renderer {

namespace {

inline Vector3 toRl(const RVec3& v) { return { v.x, v.y, v.z }; }
inline Color   toRl(RColor c)       { return { c.r, c.g, c.b, c.a }; }

const char* kEarthVS = R"(#version 330
in vec3 vertexPosition;
in vec2 vertexTexCoord;
uniform mat4 mvp;
uniform mat4 matModel;
out vec2 fragTexCoord;
out vec3 fragWorldPos;
void main() {
    fragTexCoord = vertexTexCoord;
    fragWorldPos = (matModel*vec4(vertexPosition, 1.0)).xyz;
    gl_Position  = mvp*vec4(vertexPosition, 1.0);
}
)";

const char* kEarthFS = R"(#version 330
in vec2 fragTexCoord;
in vec3 fragWorldPos;
uniform sampler2D texture0;
uniform vec4 colDiffuse;
uniform vec3 sunDir;       // direction TO the sun (world)
uniform vec3 earthCenter;  // sphere centre (world)
uniform vec3 camPos;       // camera position (world)
out vec4 finalColor;
void main() {
    vec3 N = normalize(fragWorldPos - earthCenter);
    vec3 L = normalize(sunDir);
    vec3 V = normalize(camPos - fragWorldPos);

    vec3  albedo = texture(texture0, fragTexCoord).rgb*colDiffuse.rgb;
    float diff   = max(dot(N, L), 0.0);
    vec3  color  = albedo*(0.15 + 0.85*diff);            // ambient + lambert

    // Atmospheric limb glow: strongest at the silhouette, brighter in sunlight.
    float rim = pow(1.0 - max(dot(N, V), 0.0), 3.0);
    color += vec3(0.30, 0.50, 0.95)*rim*(0.35 + 0.65*diff);

    finalColor = vec4(color, 1.0);
}
)";

} // namespace

void RaylibBackend::Init(int width, int height, const char* title) {
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(width, height, title);
    SetWindowMonitor(0);
    SetTargetFPS(60);

    tex_ = LoadTexture("src/renderer/world.jpg");
    GenTextureMipmaps(&tex_);
    SetTextureFilter(tex_, TEXTURE_FILTER_TRILINEAR);

    const float kLonOffset = 12.0f / 360.0f;  // degrees east -> UV fraction
    Mesh mesh = GenMeshSphere(EARTH_RADIUS_KM, 128, 128);
    for (int i = 0; i < mesh.vertexCount; i++) {
        float u = mesh.texcoords[i*2 + 0];
        mesh.texcoords[i*2 + 0] = mesh.texcoords[i*2 + 1] + kLonOffset;
        mesh.texcoords[i*2 + 1] = u;
    }
    UpdateMeshBuffer(mesh, 1, mesh.texcoords, mesh.vertexCount * 2 * sizeof(float), 0);
    sphere_ = LoadModelFromMesh(mesh);
    sphere_.transform = MatrixRotateX(-PI/2);

    earthShader_    = LoadShaderFromMemory(kEarthVS, kEarthFS);
    sunDirLoc_      = GetShaderLocation(earthShader_, "sunDir");
    earthCenterLoc_ = GetShaderLocation(earthShader_, "earthCenter");
    camPosLoc_      = GetShaderLocation(earthShader_, "camPos");
    sphere_.materials[0].shader = earthShader_;
    sphere_.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = tex_;
}

void RaylibBackend::Shutdown() {
    UnloadModel(sphere_);
    CloseWindow();
}

bool RaylibBackend::ShouldClose() const { return WindowShouldClose(); }

FrameInput RaylibBackend::PollInput() {
    Vector2 d = GetMouseDelta();
    return FrameInput {
        d.x, d.y,
        GetMouseWheelMove(),
        IsMouseButtonDown(MOUSE_BUTTON_LEFT),
    };
}

float  RaylibBackend::FrameTime() const { return GetFrameTime(); }
double RaylibBackend::Time() const      { return GetTime(); }

void RaylibBackend::BeginFrame(RColor clear) {
    BeginDrawing();
    ClearBackground(toRl(clear));
}

void RaylibBackend::EndFrame() { EndDrawing(); }

void RaylibBackend::SetClipPlanes(float near_plane, float far_plane) {
    rlSetClipPlanes(near_plane, far_plane);
}

void RaylibBackend::Begin3D(const RCamera& cam) {
    Camera3D c {
        toRl(cam.position),
        toRl(cam.target),
        toRl(cam.up),
        cam.fovy,
        CAMERA_PERSPECTIVE,
    };
    BeginMode3D(c);
}

void RaylibBackend::End3D() { EndMode3D(); }

void RaylibBackend::SetLineWidth(float w) { rlSetLineWidth(w); }

void RaylibBackend::DrawLine3D(const RVec3& a, const RVec3& b, RColor c) {
    ::DrawLine3D(toRl(a), toRl(b), toRl(c));
}

void RaylibBackend::DrawCylinder(const RVec3& start, const RVec3& end,
                                 float r_start, float r_end, int sides, RColor c) {
    DrawCylinderEx(toRl(start), toRl(end), r_start, r_end, sides, toRl(c));
}

void RaylibBackend::DrawTriangle3D(const RVec3& a, const RVec3& b, const RVec3& c,
                                   RColor col) {
    ::DrawTriangle3D(toRl(a), toRl(b), toRl(c), toRl(col));
}

void RaylibBackend::DrawSphere(const RVec3& center, float radius, RColor c) {
    ::DrawSphere(toRl(center), radius, toRl(c));
}

void RaylibBackend::BeginBlend(BlendMode m) {
    BeginBlendMode(m == BlendMode::Additive ? BLEND_ADDITIVE : BLEND_ALPHA);
}

void RaylibBackend::EndBlend() { EndBlendMode(); }

void RaylibBackend::SetDepthMask(bool enabled) {
    if (enabled) rlEnableDepthMask(); else rlDisableDepthMask();
}

void RaylibBackend::DrawEarth(const RVec3& center, float /*radius*/,
                              const RVec3& sun_dir, const RVec3& cam_pos) {
    Vector3 c   = toRl(center);
    Vector3 sun = toRl(sun_dir);
    Vector3 cam = toRl(cam_pos);
    SetShaderValue(earthShader_, sunDirLoc_,      &sun, SHADER_UNIFORM_VEC3);
    SetShaderValue(earthShader_, earthCenterLoc_, &c,   SHADER_UNIFORM_VEC3);
    SetShaderValue(earthShader_, camPosLoc_,      &cam, SHADER_UNIFORM_VEC3);
    DrawModel(sphere_, c, 1.0f, WHITE);
}

void RaylibBackend::DrawRect(int x, int y, int w, int h, RColor c) {
    DrawRectangle(x, y, w, h, toRl(c));
}

void RaylibBackend::DrawRectLines(int x, int y, int w, int h, RColor c) {
    DrawRectangleLines(x, y, w, h, toRl(c));
}

void RaylibBackend::DrawText(const char* text, int x, int y, int font_size, RColor c) {
    ::DrawText(text, x, y, font_size, toRl(c));
}

void RaylibBackend::DrawFPS(int x, int y) { ::DrawFPS(x, y); }

}
