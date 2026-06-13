#define GLFW_EXPOSE_NATIVE_WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#include "bgfx_backend.hpp"
#include "../geometry.hpp"
#include "../../constants.hpp"

#include <bgfx/platform.h>
#include <bx/math.h>
#include <bx/allocator.h>
#include <bimg/decode.h>

#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

// <windows.h> (via glfw3native.h) defines a DrawText macro that clashes with our
// RenderBackend::DrawText method; drop it.
#ifdef DrawText
#undef DrawText
#endif

#include <timeapi.h>   // timeBeginPeriod (winmm)

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <fstream>
#include <vector>

#define STB_EASY_FONT_IMPLEMENTATION
#include "stb_easy_font.h"

namespace renderer {

namespace {

bx::DefaultAllocator s_allocator;

uint32_t packRgba(RColor c) {
    return (uint32_t(c.r) << 24) | (uint32_t(c.g) << 16) | (uint32_t(c.b) << 8) | uint32_t(c.a);
}

std::vector<uint8_t> readFile(const char* path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return {};
    std::streamsize n = f.tellg();
    f.seekg(0);
    std::vector<uint8_t> buf((size_t)n);
    f.read((char*)buf.data(), n);
    return buf;
}

bgfx::ShaderHandle loadShaderFile(const char* path) {
    std::vector<uint8_t> data = readFile(path);
    if (data.empty()) { std::fprintf(stderr, "bgfx: missing shader %s\n", path); return BGFX_INVALID_HANDLE; }
    const bgfx::Memory* mem = bgfx::copy(data.data(), (uint32_t)data.size());
    return bgfx::createShader(mem);
}

// A view-space direction/point as a vec4 for a bgfx uniform.
void setVec4(bgfx::UniformHandle u, const RVec3& v, float w = 0.0f) {
    float d[4] = { v.x, v.y, v.z, w };
    bgfx::setUniform(u, d);
}

} // namespace

void BgfxBackend::Init(int width, int height, const char* title) {
    width_ = width; height_ = height;
    reset_ = BGFX_RESET_VSYNC | BGFX_RESET_MSAA_X4;

    // Raise the Windows timer resolution to 1ms so the sim thread's
    // sleep_for(10ms) pacing is accurate (raylib does this implicitly; without
    // it sleeps round up to the ~15.6ms default tick and the sim runs slow).
    timeBeginPeriod(1);

    glfwInit();
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);   // bgfx owns the context
    window_ = glfwCreateWindow(width_, height_, title, nullptr, nullptr);
    glfwSetWindowUserPointer(window_, this);
    glfwSetScrollCallback(window_, [](GLFWwindow* w, double, double yoff) {
        ((BgfxBackend*)glfwGetWindowUserPointer(w))->wheel_ += (float)yoff;
    });

    bgfx::renderFrame();   // signal single-threaded (submit on this thread)
    bgfx::Init init;
    init.type = bgfx::RendererType::OpenGL;
    init.platformData.nwh = glfwGetWin32Window(window_);
    init.resolution.width  = (uint32_t)width_;
    init.resolution.height = (uint32_t)height_;
    init.resolution.reset  = reset_;
    bgfx::init(init);

    // Preserve submit order (the additive plume must draw after the opaque hull).
    bgfx::setViewMode(0, bgfx::ViewMode::Sequential);
    bgfx::setViewMode(1, bgfx::ViewMode::Sequential);

    layout_.begin()
        .add(bgfx::Attrib::Position,  3, bgfx::AttribType::Float)
        .add(bgfx::Attrib::Normal,    3, bgfx::AttribType::Float)
        .add(bgfx::Attrib::TexCoord0, 2, bgfx::AttribType::Float)
        .add(bgfx::Attrib::Color0,    4, bgfx::AttribType::Uint8, true)
        .end();

    generic_   = bgfx::createProgram(loadShaderFile("src/renderer/bgfx/shaders/vs_generic.bin"),
                                     loadShaderFile("src/renderer/bgfx/shaders/fs_generic.bin"), true);
    earthProg_ = bgfx::createProgram(loadShaderFile("src/renderer/bgfx/shaders/vs_earth.bin"),
                                     loadShaderFile("src/renderer/bgfx/shaders/fs_earth.bin"), true);

    s_tex_         = bgfx::createUniform("s_tex",        bgfx::UniformType::Sampler);
    u_tint_        = bgfx::createUniform("u_tint",       bgfx::UniformType::Vec4);
    s_color_       = bgfx::createUniform("s_color",      bgfx::UniformType::Sampler);
    s_bump_        = bgfx::createUniform("s_bump",       bgfx::UniformType::Sampler);
    s_night_       = bgfx::createUniform("s_night",      bgfx::UniformType::Sampler);
    u_sunDir_      = bgfx::createUniform("u_sunDir",     bgfx::UniformType::Vec4);
    u_earthCenter_ = bgfx::createUniform("u_earthCenter",bgfx::UniformType::Vec4);
    u_camPos_      = bgfx::createUniform("u_camPos",     bgfx::UniformType::Vec4);
    u_dispScale_   = bgfx::createUniform("u_dispScale",  bgfx::UniformType::Vec4);

    const uint8_t whitePix[4] = { 255, 255, 255, 255 };
    white_ = bgfx::createTexture2D(1, 1, false, 1, bgfx::TextureFormat::RGBA8, 0,
                                   bgfx::copy(whitePix, 4));

    startTime_ = lastTime_ = glfwGetTime();
}

void BgfxBackend::Shutdown() {
    for (auto& m : meshes_) { if (bgfx::isValid(m.vbh)) bgfx::destroy(m.vbh); if (bgfx::isValid(m.ibh)) bgfx::destroy(m.ibh); }
    for (auto& t : textures_) if (bgfx::isValid(t)) bgfx::destroy(t);
    if (bgfx::isValid(earthColor_)) bgfx::destroy(earthColor_);
    if (bgfx::isValid(earthBump_))  bgfx::destroy(earthBump_);
    if (bgfx::isValid(earthNight_)) bgfx::destroy(earthNight_);
    if (bgfx::isValid(white_))      bgfx::destroy(white_);
    if (bgfx::isValid(generic_))    bgfx::destroy(generic_);
    if (bgfx::isValid(earthProg_))  bgfx::destroy(earthProg_);
    for (bgfx::UniformHandle u : { s_tex_, u_tint_, s_color_, s_bump_, s_night_,
                                   u_sunDir_, u_earthCenter_, u_camPos_, u_dispScale_ })
        if (bgfx::isValid(u)) bgfx::destroy(u);
    bgfx::shutdown();
    if (window_) glfwDestroyWindow(window_);
    glfwTerminate();
    timeEndPeriod(1);
}

bool BgfxBackend::ShouldClose() const { return glfwWindowShouldClose(window_); }

FrameInput BgfxBackend::PollInput() {
    glfwPollEvents();

    double now = glfwGetTime();
    frameDt_ = now - lastTime_;
    lastTime_ = now;
    if (frameDt_ > 1e-6) fps_ = fps_ * 0.9 + (1.0 / frameDt_) * 0.1;

    double x, y;
    glfwGetCursorPos(window_, &x, &y);
    if (!havePrev_) { prevX_ = x; prevY_ = y; havePrev_ = true; }
    float dx = float(x - prevX_), dy = float(y - prevY_);
    prevX_ = x; prevY_ = y;

    bool left = glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
    float w = wheel_; wheel_ = 0.0f;
    return FrameInput{ dx, dy, w, left };
}

float  BgfxBackend::FrameTime() const { return (float)frameDt_; }
double BgfxBackend::Time() const      { return glfwGetTime() - startTime_; }

TextureHandle BgfxBackend::LoadTexture(const char* path) {
    std::vector<uint8_t> data = readFile(path);
    if (data.empty()) return 0;
    bimg::ImageContainer* ic = bimg::imageParse(&s_allocator, data.data(), (uint32_t)data.size());
    if (!ic) return 0;
    bgfx::TextureHandle th = bgfx::createTexture2D(
        (uint16_t)ic->m_width, (uint16_t)ic->m_height, ic->m_numMips > 1, ic->m_numLayers,
        (bgfx::TextureFormat::Enum)ic->m_format, BGFX_SAMPLER_NONE, bgfx::copy(ic->m_data, ic->m_size));
    bimg::imageFree(ic);
    textures_.push_back(th);
    return (TextureHandle)textures_.size();
}

MeshHandle BgfxBackend::CreateMesh(const Mesh& m) {
    const bgfx::Memory* vmem = bgfx::copy(m.verts.data(), (uint32_t)(m.verts.size() * sizeof(Vertex)));
    // 32-bit indices throughout, so the Earth can be tessellated past 64k verts.
    const bgfx::Memory* imem = bgfx::copy(m.idx.data(), (uint32_t)(m.idx.size() * sizeof(uint32_t)));
    GpuMesh g;
    g.vbh = bgfx::createVertexBuffer(vmem, layout_);
    g.ibh = bgfx::createIndexBuffer(imem, BGFX_BUFFER_INDEX32);
    meshes_.push_back(g);
    return (MeshHandle)meshes_.size();
}

void BgfxBackend::DestroyMesh(MeshHandle h) {
    if (h == 0 || h > meshes_.size()) return;
    GpuMesh& g = meshes_[h - 1];
    if (bgfx::isValid(g.vbh)) { bgfx::destroy(g.vbh); g.vbh = BGFX_INVALID_HANDLE; }
    if (bgfx::isValid(g.ibh)) { bgfx::destroy(g.ibh); g.ibh = BGFX_INVALID_HANDLE; }
}

void BgfxBackend::BeginFrame(RColor clear) {
    // Handle window resize.
    int fw, fh;
    glfwGetFramebufferSize(window_, &fw, &fh);
    if (fw > 0 && fh > 0 && (fw != width_ || fh != height_)) {
        width_ = fw; height_ = fh;
        bgfx::reset((uint32_t)fw, (uint32_t)fh, reset_);
    }

    bgfx::setViewClear(0, BGFX_CLEAR_COLOR | BGFX_CLEAR_DEPTH, packRgba(clear), 1.0f, 0);
    bgfx::setViewRect(0, 0, 0, (uint16_t)width_, (uint16_t)height_);
    bgfx::touch(0);

    // View 1: 2D overlay, screen-space ortho (top-left origin, y down).
    float ortho[16];
    bx::mtxOrtho(ortho, 0.0f, (float)width_, (float)height_, 0.0f, 0.0f, 100.0f, 0.0f,
                 bgfx::getCaps()->homogeneousDepth);
    bgfx::setViewClear(1, BGFX_CLEAR_NONE, 0, 1.0f, 0);
    bgfx::setViewRect(1, 0, 0, (uint16_t)width_, (uint16_t)height_);
    bgfx::setViewTransform(1, nullptr, ortho);
}

void BgfxBackend::EndFrame() { bgfx::frame(); }

void BgfxBackend::SetClipPlanes(float near_plane, float far_plane) {
    near_ = near_plane; far_ = far_plane;
}

void BgfxBackend::Begin3D(const RCamera& cam) {
    cam_ = cam;
    float view[16], proj[16];
    bx::mtxLookAt(view, bx::Vec3{ cam.position.x, cam.position.y, cam.position.z },
                        bx::Vec3{ cam.target.x,   cam.target.y,   cam.target.z },
                        bx::Vec3{ cam.up.x,       cam.up.y,       cam.up.z },
                  bx::Handedness::Right);
    float aspect = height_ > 0 ? (float)width_ / (float)height_ : 1.0f;
    // bx::mtxProj applies toRad() internally, so fovy is in DEGREES here.
    bx::mtxProj(proj, cam.fovy, aspect, near_, far_,
                bgfx::getCaps()->homogeneousDepth, bx::Handedness::Right);
    bgfx::setViewTransform(0, view, proj);
    bgfx::setViewRect(0, 0, 0, (uint16_t)width_, (uint16_t)height_);
}

void BgfxBackend::End3D() {}

void BgfxBackend::DrawModel(MeshHandle h, const RMat4& model, const Material& mat) {
    if (h == 0 || h > meshes_.size()) return;
    const GpuMesh& g = meshes_[h - 1];
    if (!bgfx::isValid(g.vbh)) return;

    bgfx::setTransform(model.m);
    bgfx::setVertexBuffer(0, g.vbh);
    bgfx::setIndexBuffer(g.ibh);
    float tint[4] = { mat.color.r / 255.0f, mat.color.g / 255.0f, mat.color.b / 255.0f, mat.color.a / 255.0f };
    bgfx::setUniform(u_tint_, tint);
    bgfx::setTexture(0, s_tex_, mat.texture ? textures_[mat.texture - 1] : white_);

    uint64_t state = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_DEPTH_TEST_LESS;
    if (mat.depth_write) state |= BGFX_STATE_WRITE_Z;
    state |= (mat.blend == BlendMode::Additive) ? BGFX_STATE_BLEND_ADD : BGFX_STATE_BLEND_ALPHA;
    bgfx::setState(state);
    bgfx::submit(0, generic_);
}

void BgfxBackend::DrawLines(const LineVertex* v, size_t count, float /*width*/) {
    if (count < 2) return;                       // bgfx GL has no portable line width (~1px)
    uint32_t n = (uint32_t)count;
    if (bgfx::getAvailTransientVertexBuffer(n, layout_) < n) return;
    bgfx::TransientVertexBuffer tvb;
    bgfx::allocTransientVertexBuffer(&tvb, n, layout_);
    Vertex* dst = (Vertex*)tvb.data;
    for (uint32_t i = 0; i < n; ++i)
        dst[i] = Vertex{ v[i].pos, {0, 0, 0}, 0.0f, 0.0f, v[i].color };

    bgfx::setVertexBuffer(0, &tvb);
    float tint[4] = { 1, 1, 1, 1 };
    bgfx::setUniform(u_tint_, tint);
    bgfx::setTexture(0, s_tex_, white_);
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_WRITE_Z
                   | BGFX_STATE_DEPTH_TEST_LESS | BGFX_STATE_PT_LINES);
    bgfx::submit(0, generic_);
}

void BgfxBackend::DrawRocket(const RocketFrame& f) {
    rocket_.Ensure(*this, f.dims);
    rocket_.Draw(*this, f);
}

void BgfxBackend::ensureEarth() {
    if (earthMesh_) return;
    const float kLonOffset = 12.0f / 360.0f;
    // High tessellation so vertex displacement has the resolution to show real
    // relief. 2048 rings/sectors -> ~4.2M verts / ~25M tris, 32-bit indices.
    earthMesh_  = CreateMesh(geom::buildSphere((float)EARTH_RADIUS_M, 2048, 2048, kLonOffset));
    auto loadDDS = [&](const char* path) -> bgfx::TextureHandle {
        std::vector<uint8_t> data = readFile(path);
        if (data.empty()) { std::fprintf(stderr, "bgfx: missing %s\n", path); return BGFX_INVALID_HANDLE; }
        bimg::ImageContainer* ic = bimg::imageParse(&s_allocator, data.data(), (uint32_t)data.size());
        if (!ic) { std::fprintf(stderr, "bgfx: parse failed %s\n", path); return BGFX_INVALID_HANDLE; }
        const uint64_t flags = BGFX_SAMPLER_V_CLAMP | BGFX_SAMPLER_MIN_ANISOTROPIC | BGFX_SAMPLER_MAG_ANISOTROPIC;
        bgfx::TextureHandle th = bgfx::createTexture2D(
            (uint16_t)ic->m_width, (uint16_t)ic->m_height, ic->m_numMips > 1, ic->m_numLayers,
            (bgfx::TextureFormat::Enum)ic->m_format, flags, bgfx::copy(ic->m_data, ic->m_size));
        bimg::imageFree(ic);
        return th;
    };
    earthColor_ = loadDDS("src/renderer/bgfx/Earth-Color-Map-32768x16384.dds");
    earthBump_  = loadDDS("src/renderer/bgfx/Earth-Bump-Map-32768x16384.dds");
    earthNight_ = loadDDS("src/renderer/bgfx/Earth-Night-Map-32768x16384.dds");
}

void BgfxBackend::DrawEarth(const EarthFrame& f) {
    ensureEarth();
    if (earthMesh_ == 0) return;
    const GpuMesh& g = meshes_[earthMesh_ - 1];

    setVec4(u_sunDir_,      f.sun_dir);
    setVec4(u_earthCenter_, f.center);
    setVec4(u_camPos_,      f.cam_pos);
    float disp[4] = { 80000.0f, 0, 0, 0 };   // metres of height exaggeration
    bgfx::setUniform(u_dispScale_, disp);
    bgfx::setTexture(0, s_color_, earthColor_);
    bgfx::setTexture(1, s_bump_,  earthBump_);   // also read by the vertex shader
    bgfx::setTexture(2, s_night_, earthNight_);

    bgfx::setTransform(f.model.m);
    bgfx::setVertexBuffer(0, g.vbh);
    bgfx::setIndexBuffer(g.ibh);
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_WRITE_Z
                   | BGFX_STATE_DEPTH_TEST_LESS);
    bgfx::submit(0, earthProg_);
}

// --- 2D overlay (view 1) ----------------------------------------------------

void BgfxBackend::submit2DTris(const Vertex* v, uint32_t count, RColor tint) {
    if (count < 3) return;
    if (bgfx::getAvailTransientVertexBuffer(count, layout_) < count) return;
    bgfx::TransientVertexBuffer tvb;
    bgfx::allocTransientVertexBuffer(&tvb, count, layout_);
    std::memcpy(tvb.data, v, count * sizeof(Vertex));

    bgfx::setVertexBuffer(0, &tvb);
    float t[4] = { tint.r / 255.0f, tint.g / 255.0f, tint.b / 255.0f, tint.a / 255.0f };
    bgfx::setUniform(u_tint_, t);
    bgfx::setTexture(0, s_tex_, white_);
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_BLEND_ALPHA);
    bgfx::submit(1, generic_);
}

void BgfxBackend::DrawRect(int x, int y, int w, int h, RColor c) {
    float x0 = (float)x, y0 = (float)y, x1 = (float)(x + w), y1 = (float)(y + h);
    auto V = [](float px, float py) { return Vertex{ {px, py, 0}, {0,0,0}, 0, 0, kWhite }; };
    Vertex quad[6] = { V(x0,y0), V(x1,y0), V(x1,y1), V(x0,y0), V(x1,y1), V(x0,y1) };
    submit2DTris(quad, 6, c);
}

void BgfxBackend::DrawRectLines(int x, int y, int w, int h, RColor c) {
    DrawRect(x, y, w, 1, c);
    DrawRect(x, y + h - 1, w, 1, c);
    DrawRect(x, y, 1, h, c);
    DrawRect(x + w - 1, y, 1, h, c);
}

void BgfxBackend::DrawText(const char* text, int x, int y, int font_size, RColor c) {
    static char quads[64000];   // stb vertex buffer: 16 bytes/vertex, 4 verts/quad
    int nq = stb_easy_font_print((float)x, (float)y, (char*)text, nullptr, quads, sizeof(quads));
    if (nq <= 0) return;

    const float scale = font_size / 8.0f;   // stb base glyphs are ~8px tall
    struct SV { float x, y, z; unsigned char col[4]; };
    const SV* sv = (const SV*)quads;

    std::vector<Vertex> tris;
    tris.reserve(nq * 6);
    auto put = [&](const SV& s) {
        tris.push_back(Vertex{ { x + (s.x - x) * scale, y + (s.y - y) * scale, 0 }, {0,0,0}, 0, 0, kWhite });
    };
    for (int q = 0; q < nq; ++q) {
        const SV& v0 = sv[q*4+0]; const SV& v1 = sv[q*4+1];
        const SV& v2 = sv[q*4+2]; const SV& v3 = sv[q*4+3];
        put(v0); put(v1); put(v2);
        put(v0); put(v2); put(v3);
    }
    submit2DTris(tris.data(), (uint32_t)tris.size(), c);
}

void BgfxBackend::DrawFPS(int x, int y) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%d FPS", (int)(fps_ + 0.5));
    DrawText(buf, x, y, 16, RColor{ 0, 228, 48, 255 });
}

}
