#include "raylib_backend.hpp"
#include "../geometry.hpp"
#include "../../constants.hpp"
#include <raymath.h>
#include <rlgl.h>
#include <cassert>

namespace renderer {

namespace {

inline Vector3 toRl(const RVec3& v) { return { v.x, v.y, v.z }; }
inline Color   toRl(RColor c)       { return { c.r, c.g, c.b, c.a }; }

// RMat4 is column-major and raylib's Matrix field m{k} is the column-major
// element k, so a field-by-field copy is the correct conversion (the structs'
// in-memory orders differ, so this must NOT be a memcpy).
inline Matrix toRl(const RMat4& s) {
    Matrix r;
    r.m0 = s.m[0];  r.m1 = s.m[1];   r.m2  = s.m[2];   r.m3  = s.m[3];
    r.m4 = s.m[4];  r.m5 = s.m[5];   r.m6  = s.m[6];   r.m7  = s.m[7];
    r.m8 = s.m[8];  r.m9 = s.m[9];   r.m10 = s.m[10];  r.m11 = s.m[11];
    r.m12 = s.m[12]; r.m13 = s.m[13]; r.m14 = s.m[14]; r.m15 = s.m[15];
    return r;
}

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
uniform vec3 sunDir;       // direction TO the sun (view)
uniform vec3 earthCenter;  // sphere centre (view)
uniform vec3 camPos;       // camera position (view)
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

    defaultMat_ = LoadMaterialDefault();
    defaultTex_ = defaultMat_.maps[MATERIAL_MAP_DIFFUSE].texture;  // raylib's 1x1 white

    earthShader_    = LoadShaderFromMemory(kEarthVS, kEarthFS);
    sunDirLoc_      = GetShaderLocation(earthShader_, "sunDir");
    earthCenterLoc_ = GetShaderLocation(earthShader_, "earthCenter");
    camPosLoc_      = GetShaderLocation(earthShader_, "camPos");
    earthMat_        = LoadMaterialDefault();
    earthMat_.shader = earthShader_;
}

void RaylibBackend::Shutdown() {
    for (auto& m : meshes_)   UnloadMesh(m);
    for (auto& t : textures_) UnloadTexture(t);
    UnloadShader(earthShader_);
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

TextureHandle RaylibBackend::LoadTexture(const char* path) {
    ::Texture2D t = ::LoadTexture(path);
    GenTextureMipmaps(&t);
    SetTextureFilter(t, TEXTURE_FILTER_TRILINEAR);
    textures_.push_back(t);
    return (TextureHandle)textures_.size();
}

MeshHandle RaylibBackend::CreateMesh(const Mesh& m) {
    assert(m.verts.size() <= 65535 && "raylib mesh indices are 16-bit");
    ::Mesh rm = { 0 };
    rm.vertexCount   = (int)m.verts.size();
    rm.triangleCount = (int)(m.idx.size() / 3);
    rm.vertices  = (float*)MemAlloc(rm.vertexCount * 3 * sizeof(float));
    rm.normals   = (float*)MemAlloc(rm.vertexCount * 3 * sizeof(float));
    rm.texcoords = (float*)MemAlloc(rm.vertexCount * 2 * sizeof(float));
    rm.colors    = (unsigned char*)MemAlloc(rm.vertexCount * 4);
    rm.indices   = (unsigned short*)MemAlloc(m.idx.size() * sizeof(unsigned short));
    for (int i = 0; i < rm.vertexCount; ++i) {
        const Vertex& v = m.verts[i];
        rm.vertices[i*3+0] = v.pos.x; rm.vertices[i*3+1] = v.pos.y; rm.vertices[i*3+2] = v.pos.z;
        rm.normals[i*3+0]  = v.normal.x; rm.normals[i*3+1] = v.normal.y; rm.normals[i*3+2] = v.normal.z;
        rm.texcoords[i*2+0] = v.u; rm.texcoords[i*2+1] = v.v;
        rm.colors[i*4+0] = v.color.r; rm.colors[i*4+1] = v.color.g;
        rm.colors[i*4+2] = v.color.b; rm.colors[i*4+3] = v.color.a;
    }
    for (size_t i = 0; i < m.idx.size(); ++i) rm.indices[i] = (unsigned short)m.idx[i];
    UploadMesh(&rm, false);
    meshes_.push_back(rm);
    return (MeshHandle)meshes_.size();
}

void RaylibBackend::DestroyMesh(MeshHandle h) {
    if (h == 0 || h > meshes_.size()) return;
    UnloadMesh(meshes_[h - 1]);
    meshes_[h - 1] = ::Mesh{ 0 };   // tombstone; handle is not reused
}

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

void RaylibBackend::DrawModel(MeshHandle h, const RMat4& model, const Material& mat) {
    if (h == 0 || h > meshes_.size()) return;
    defaultMat_.maps[MATERIAL_MAP_DIFFUSE].color   = toRl(mat.color);
    defaultMat_.maps[MATERIAL_MAP_DIFFUSE].texture =
        mat.texture ? textures_[mat.texture - 1] : defaultTex_;

    if (mat.blend == BlendMode::Additive) BeginBlendMode(BLEND_ADDITIVE);
    if (!mat.depth_write) rlDisableDepthMask();
    DrawMesh(meshes_[h - 1], defaultMat_, toRl(model));
    if (!mat.depth_write) rlEnableDepthMask();
    if (mat.blend == BlendMode::Additive) EndBlendMode();
}

void RaylibBackend::DrawLines(const LineVertex* v, size_t count, float width) {
    rlSetLineWidth(width);
    for (size_t i = 0; i + 1 < count; i += 2)
        ::DrawLine3D(toRl(v[i].pos), toRl(v[i + 1].pos), toRl(v[i].color));
}

void RaylibBackend::DrawRocket(const RocketFrame& f) {
    rocket_.Ensure(*this, f.dims);   // build on first use, rebuild on staging
    rocket_.Draw(*this, f);
}

void RaylibBackend::ensureEarth() {
    if (earthMesh_) return;
    // Sphere in metres (radius EARTH_RADIUS_M) so the renderer's view basis
    // (metres -> km) applies to it like everything else. The 12 deg east offset
    // matches the texture alignment.
    const float kLonOffset = 12.0f / 360.0f;
    earthMesh_ = CreateMesh(geom::buildSphere((float)EARTH_RADIUS_M, 128, 128, kLonOffset));
    earthTex_  = LoadTexture("src/renderer/raylib/world.jpg");
}

void RaylibBackend::DrawEarth(const EarthFrame& f) {
    ensureEarth();
    Vector3 sun = toRl(f.sun_dir), c = toRl(f.center), cam = toRl(f.cam_pos);
    SetShaderValue(earthShader_, sunDirLoc_,      &sun, SHADER_UNIFORM_VEC3);
    SetShaderValue(earthShader_, earthCenterLoc_, &c,   SHADER_UNIFORM_VEC3);
    SetShaderValue(earthShader_, camPosLoc_,      &cam, SHADER_UNIFORM_VEC3);
    earthMat_.maps[MATERIAL_MAP_DIFFUSE].texture = textures_[earthTex_ - 1];
    DrawMesh(meshes_[earthMesh_ - 1], earthMat_, toRl(f.model));
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
