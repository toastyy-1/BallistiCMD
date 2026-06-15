#if defined(_WIN32)
#  define GLFW_EXPOSE_NATIVE_WIN32
#  define WIN32_LEAN_AND_MEAN
#  define NOMINMAX
#elif defined(__APPLE__)
#  define GLFW_EXPOSE_NATIVE_COCOA
#elif defined(__linux__)
#  define GLFW_EXPOSE_NATIVE_X11
#endif

#include "bgfx_backend.hpp"
#include "../geometry.hpp"
#include "../../constants.hpp"

#include <bgfx/platform.h>
#include <bx/math.h>
#include <bx/allocator.h>
#include <bimg/decode.h>

#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#ifdef DrawText
#undef DrawText
#endif

#if defined(_WIN32)
#include <timeapi.h>
#endif

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

#if defined(_WIN32)
    timeBeginPeriod(1);
#endif

    if (!glfwInit()) {
        std::fprintf(stderr, "bgfx: failed to initialize GLFW\n");
        std::abort();
    }
    glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);   // bgfx owns the context
    window_ = glfwCreateWindow(width_, height_, title, nullptr, nullptr);
    if (!window_) {
        std::fprintf(stderr, "bgfx: failed to create GLFW window\n");
        glfwTerminate();
        std::abort();
    }
    glfwSetWindowUserPointer(window_, this);
    glfwSetScrollCallback(window_, [](GLFWwindow* w, double, double yoff) {
        ((BgfxBackend*)glfwGetWindowUserPointer(w))->wheel_ += (float)yoff;
    });

    bgfx::renderFrame();   // signal single-threaded (submit on this thread)
    bgfx::Init init;
    init.type = bgfx::RendererType::OpenGL;
#if defined(_WIN32)
    init.platformData.nwh = glfwGetWin32Window(window_);
#elif defined(__APPLE__)
    init.platformData.nwh = glfwGetCocoaWindow(window_);
#elif defined(__linux__)
    init.platformData.ndt = glfwGetX11Display();
    init.platformData.nwh = (void*)(uintptr_t)glfwGetX11Window(window_);
#endif
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
    cloudProg_ = bgfx::createProgram(loadShaderFile("src/renderer/bgfx/shaders/vs_cloud.bin"),
                                     loadShaderFile("src/renderer/bgfx/shaders/fs_cloud.bin"), true);
    // Terrain LOD patch reuses the earth fragment shader, so it shades identically.
    patchProg_ = bgfx::createProgram(loadShaderFile("src/renderer/bgfx/shaders/vs_patch.bin"),
                                     loadShaderFile("src/renderer/bgfx/shaders/fs_earth.bin"), true);
    atmosProg_ = bgfx::createProgram(loadShaderFile("src/renderer/bgfx/shaders/vs_atmos.bin"),
                                     loadShaderFile("src/renderer/bgfx/shaders/fs_atmos.bin"), true);
    flareProg_ = bgfx::createProgram(loadShaderFile("src/renderer/bgfx/shaders/vs_flare.bin"),
                                     loadShaderFile("src/renderer/bgfx/shaders/fs_flare.bin"), true);
    rocketProg_ = bgfx::createProgram(loadShaderFile("src/renderer/bgfx/shaders/vs_rocket.bin"),
                                      loadShaderFile("src/renderer/bgfx/shaders/fs_rocket.bin"), true);
    heatProg_  = bgfx::createProgram(loadShaderFile("src/renderer/bgfx/shaders/vs_heat.bin"),
                                     loadShaderFile("src/renderer/bgfx/shaders/fs_heat.bin"), true);

    s_tex_         = bgfx::createUniform("s_tex",        bgfx::UniformType::Sampler);
    u_tint_        = bgfx::createUniform("u_tint",       bgfx::UniformType::Vec4);
    u_depth_       = bgfx::createUniform("u_depth",      bgfx::UniformType::Vec4);
    u_light_       = bgfx::createUniform("u_light",      bgfx::UniformType::Vec4);
    u_heat_        = bgfx::createUniform("u_heat",       bgfx::UniformType::Vec4);
    u_earth_       = bgfx::createUniform("u_earth",      bgfx::UniformType::Vec4);
    s_color_       = bgfx::createUniform("s_color",      bgfx::UniformType::Sampler);
    s_bump_        = bgfx::createUniform("s_bump",       bgfx::UniformType::Sampler);
    s_night_       = bgfx::createUniform("s_night",      bgfx::UniformType::Sampler);
    s_emiss_       = bgfx::createUniform("s_emiss",      bgfx::UniformType::Sampler);
    s_rough_       = bgfx::createUniform("s_rough",      bgfx::UniformType::Sampler);
    u_sunDir_      = bgfx::createUniform("u_sunDir",     bgfx::UniformType::Vec4);
    u_earthCenter_ = bgfx::createUniform("u_earthCenter",bgfx::UniformType::Vec4);
    u_camPos_      = bgfx::createUniform("u_camPos",     bgfx::UniformType::Vec4);
    u_dispScale_   = bgfx::createUniform("u_dispScale",  bgfx::UniformType::Vec4);
    s_cloud_       = bgfx::createUniform("s_cloud",      bgfx::UniformType::Sampler);
    u_cloudAlpha_  = bgfx::createUniform("u_cloudAlpha", bgfx::UniformType::Vec4);
    u_cloudDisp_   = bgfx::createUniform("u_cloudDisp",  bgfx::UniformType::Vec4);
    u_atmos_       = bgfx::createUniform("u_atmos",      bgfx::UniformType::Vec4);
    u_rayFwd_      = bgfx::createUniform("u_rayFwd",     bgfx::UniformType::Vec4);
    u_rayRight_    = bgfx::createUniform("u_rayRight",   bgfx::UniformType::Vec4);
    u_rayUp_       = bgfx::createUniform("u_rayUp",      bgfx::UniformType::Vec4);
    u_patchC_      = bgfx::createUniform("u_patchC",     bgfx::UniformType::Vec4);
    u_patchE_      = bgfx::createUniform("u_patchE",     bgfx::UniformType::Vec4);
    u_patchN_      = bgfx::createUniform("u_patchN",     bgfx::UniformType::Vec4);
    u_patchCam_    = bgfx::createUniform("u_patchCam",   bgfx::UniformType::Vec4);
    u_patchTrue_   = bgfx::createUniform("u_patchTrue",  bgfx::UniformType::Vec4);

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
    if (bgfx::isValid(earthCloud_)) bgfx::destroy(earthCloud_);
    if (bgfx::isValid(earthRough_)) bgfx::destroy(earthRough_);
    if (bgfx::isValid(earthEmiss_)) bgfx::destroy(earthEmiss_);
    if (bgfx::isValid(white_))      bgfx::destroy(white_);
    if (bgfx::isValid(generic_))    bgfx::destroy(generic_);
    if (bgfx::isValid(earthProg_))  bgfx::destroy(earthProg_);
    if (bgfx::isValid(cloudProg_))  bgfx::destroy(cloudProg_);
    if (bgfx::isValid(patchProg_))  bgfx::destroy(patchProg_);
    if (bgfx::isValid(atmosProg_))  bgfx::destroy(atmosProg_);
    if (bgfx::isValid(flareProg_))  bgfx::destroy(flareProg_);
    if (bgfx::isValid(rocketProg_)) bgfx::destroy(rocketProg_);
    if (bgfx::isValid(heatProg_))   bgfx::destroy(heatProg_);
    for (bgfx::UniformHandle u : { s_tex_, u_tint_, u_depth_, u_light_, u_heat_, u_earth_, s_color_, s_bump_, s_night_, s_rough_, s_emiss_,
                                   u_sunDir_, u_earthCenter_, u_camPos_, u_dispScale_,
                                   s_cloud_, u_cloudAlpha_, u_cloudDisp_, u_atmos_,
                                   u_rayFwd_, u_rayRight_, u_rayUp_,
                                   u_patchC_, u_patchE_, u_patchN_, u_patchCam_, u_patchTrue_ })
        if (bgfx::isValid(u)) bgfx::destroy(u);
    bgfx::shutdown();
    if (window_) glfwDestroyWindow(window_);
    glfwTerminate();
#if defined(_WIN32)
    timeEndPeriod(1);
#endif
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

    auto down = [&](int k){ return glfwGetKey(window_, k) == GLFW_PRESS; };
    float mx = (down(GLFW_KEY_D) ? 1.0f : 0.0f) - (down(GLFW_KEY_A) ? 1.0f : 0.0f);
    float my = (down(GLFW_KEY_E) ? 1.0f : 0.0f) - (down(GLFW_KEY_Q) ? 1.0f : 0.0f);
    float mz = (down(GLFW_KEY_W) ? 1.0f : 0.0f) - (down(GLFW_KEY_S) ? 1.0f : 0.0f);
    bool boost = down(GLFW_KEY_LEFT_SHIFT) || down(GLFW_KEY_RIGHT_SHIFT);
    bool recenter = down(GLFW_KEY_F);
    return FrameInput{ dx, dy, w, left, mx, my, mz, boost, recenter };
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
    float depth[4] = { far_, 0, 0, 0 };
    bgfx::setUniform(u_depth_, depth);
    // Program select: emissive -> additive ablation shell; lit -> PBR+grime;
    // everything else (lines, plume, markers, 2D) -> flat generic.
    float light[4] = { sunDirView_.x, sunDirView_.y, sunDirView_.z, mat.lit ? 1.0f : 0.0f };
    bgfx::setUniform(u_light_, light);
    if (mat.lit || mat.emissive) {
        float cam[4] = { camPosView_.x, camPosView_.y, camPosView_.z, 0 };
        bgfx::setUniform(u_camPos_, cam);
        float heat[4] = { heatDir_.x, heatDir_.y, heatDir_.z, heatAmt_ };
        bgfx::setUniform(u_heat_, heat);
    }
    if (mat.lit) {
        // Earth params + textures for the rocket's analytic reflection + earthshine.
        float e[4] = { earthCenterView_.x, earthCenterView_.y, earthCenterView_.z, (float)EARTH_RADIUS_KM };
        bgfx::setUniform(u_earth_, e);
        bgfx::setTexture(0, s_color_, bgfx::isValid(earthColor_) ? earthColor_ : white_);
        bgfx::setTexture(1, s_night_, bgfx::isValid(earthNight_) ? earthNight_ : white_);
        bgfx::setTexture(2, s_cloud_, bgfx::isValid(earthCloud_) ? earthCloud_ : white_);
    } else {
        bgfx::setTexture(0, s_tex_, mat.texture ? textures_[mat.texture - 1] : white_);
    }

    bgfx::ProgramHandle prog = mat.emissive ? heatProg_ : (mat.lit ? rocketProg_ : generic_);
    uint64_t state = BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_DEPTH_TEST_LESS;
    if (mat.depth_write) state |= BGFX_STATE_WRITE_Z;
    if (mat.emissive) {
        state |= BGFX_STATE_BLEND_ADD | BGFX_STATE_CULL_CW;   // glow shell, single layer
    } else if (mat.blend == BlendMode::Additive) {
        state |= BGFX_STATE_BLEND_ADD;          // plume: both sides, no cull
    } else {
        state |= BGFX_STATE_BLEND_ALPHA;
        if (mat.cull) state |= BGFX_STATE_CULL_CW;   // closed solids; off for open bell / fins
    }
    bgfx::setState(state);
    bgfx::submit(0, prog);
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
    float depth[4] = { far_, 0, 0, 0 };
    bgfx::setUniform(u_depth_, depth);
    float unlit[4] = { 0, 0, 0, 0 };          // lines are flat-coloured
    bgfx::setUniform(u_light_, unlit);
    bgfx::setTexture(0, s_tex_, white_);
    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_WRITE_Z
                   | BGFX_STATE_DEPTH_TEST_LESS | BGFX_STATE_PT_LINES);
    bgfx::submit(0, generic_);
}

void BgfxBackend::DrawRocket(const RocketFrame& f) {
    heatDir_ = f.vel_dir;   // cached for the lit (PBR) DrawModel heating glow
    heatAmt_ = f.heating;
    rocket_.Ensure(*this, f.dims);
    rocket_.Draw(*this, f);
}

void BgfxBackend::ensureEarth() {
    if (earthMesh_) return;
    // Half-turn: aligns the Greenwich-centred equirectangular map (u=0.5 -> 0 deg)
    // with the ECI frame the rocket is placed in, so launch sites land correctly
    // (e.g. Knoxville at -83.94 deg, not 108 deg E / China). Keep this in sync with
    // the same offset hardcoded in fs_earth, fs_rocket and vs_patch.
    const float kLonOffset = 0.5f;
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
    earthEmiss_ = loadDDS("src/renderer/bgfx/Earth-Night-Emission-Map-32768x16384.dds");  // white = light source strength
    earthCloud_ = loadDDS("src/renderer/bgfx/Earth-Cloud-Map-32768x16384.dds");
    earthRough_ = loadDDS("src/renderer/bgfx/Earth-Roughness-Map-32768x16384.dds");

    // Sphere for the cloud shells. Denser than a plain textured sphere would need
    // so the per-vertex noise displacement resolves smoothly. 32-bit indices.
    cloudMesh_  = CreateMesh(geom::buildSphere((float)EARTH_RADIUS_M, 512, 512, kLonOffset));

    // Camera-following terrain LOD patch: a dense flat grid the vertex shader
    // wraps onto the sphere under the camera and displaces at full map resolution.
    patchMesh_  = CreateMesh(geom::buildGrid(192));
}

void BgfxBackend::DrawEarth(const EarthFrame& f) {
    ensureEarth();
    if (earthMesh_ == 0) return;
    const GpuMesh& g = meshes_[earthMesh_ - 1];
    sunDirView_ = f.sun_dir;   // cache for lit DrawModel (the rocket), drawn later
    camPosView_ = f.cam_pos;
    earthCenterView_ = f.center;

    // Atmosphere FIRST: full-screen analytic single scattering, drawn before the
    // opaque earth so the earth's real (displaced) silhouette overwrites the inner
    // part. That keeps the limb glow exactly at the visible edge and hides the
    // sphere-clip discontinuity behind the planet -- no false horizon floating in
    // the disk. For each pixel we reconstruct the view ray, intersect the
    // atmosphere/planet spheres, and integrate Rayleigh+Mie in front of the planet,
    // so it is correct from any altitude.
    if (bgfx::getAvailTransientVertexBuffer(3, layout_) >= 3) {
        RVec3 fwd   = rmath::normalize(rmath::sub(cam_.target, cam_.position));
        RVec3 right = rmath::normalize(rmath::cross(fwd, cam_.up));
        RVec3 up    = rmath::cross(right, fwd);
        float tanH  = tanf(cam_.fovy * 0.5f * (float)(M_PI / 180.0));
        float aspect = height_ > 0 ? (float)width_ / (float)height_ : 1.0f;

        setVec4(u_camPos_, f.cam_pos);
        setVec4(u_earthCenter_, f.center);
        setVec4(u_sunDir_, f.sun_dir);
        // x: planet radius, y: atmosphere radius, z: scale height, w: exposure (km).
        // y: Kármán line (~100 km). z: Rayleigh scale height (~8.5 km). w: exposure
        // (raised to compensate for the thinner, denser-falloff air column).
        float at[4] = { (float)EARTH_RADIUS_KM, (float)EARTH_RADIUS_KM + 100.0f, 8.5f, 0.05f };
        bgfx::setUniform(u_atmos_, at);
        setVec4(u_rayFwd_, fwd);
        float rr[4] = { right.x*tanH*aspect, right.y*tanH*aspect, right.z*tanH*aspect, 0 };
        bgfx::setUniform(u_rayRight_, rr);
        float ru[4] = { up.x*tanH, up.y*tanH, up.z*tanH, 0 };
        bgfx::setUniform(u_rayUp_, ru);

        bgfx::TransientVertexBuffer tvb;
        bgfx::allocTransientVertexBuffer(&tvb, 3, layout_);
        Vertex* d = (Vertex*)tvb.data;
        d[0] = Vertex{ { -1, -1, 0 }, {0,0,0}, 0, 0, kWhite };   // full-screen triangle
        d[1] = Vertex{ {  3, -1, 0 }, {0,0,0}, 0, 0, kWhite };
        d[2] = Vertex{ { -1,  3, 0 }, {0,0,0}, 0, 0, kWhite };
        bgfx::setTexture(0, s_night_, bgfx::isValid(earthNight_) ? earthNight_ : white_);  // light-pollution colour
        bgfx::setTexture(1, s_emiss_, bgfx::isValid(earthEmiss_) ? earthEmiss_ : white_);  // light-pollution strength
        bgfx::setVertexBuffer(0, &tvb);
        bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_BLEND_ADD);
        bgfx::submit(0, atmosProg_);
    }

    // Camera altitude (km). When low, the high-detail terrain patch REPLACES the
    // global sphere as the surface (they're nearly coincident -- drawing both
    // z-fights into noise), so decide before the earth draw whether to skip it.
    float dcx = f.cam_pos.x - f.center.x, dcy = f.cam_pos.y - f.center.y, dcz = f.cam_pos.z - f.center.z;
    float camAlt = sqrtf(dcx*dcx + dcy*dcy + dcz*dcz) - (float)EARTH_RADIUS_KM;
    // Patch is the surface near the ground; by 90 km it has geomorphed (vs_patch's
    // altitude LOD bias) down to the global sphere's resolution, so the handoff is
    // seamless. Above that the global sphere takes over.
    bool  patchActive = patchMesh_ && camAlt < 90.0f;

    setVec4(u_sunDir_,      f.sun_dir);
    setVec4(u_earthCenter_, f.center);
    setVec4(u_camPos_,      f.cam_pos);
    float disp[4] = { 8849.0f, 0, 0, 0 };    // true scale: Everest above sea level (m)
    bgfx::setUniform(u_dispScale_, disp);
    float depth[4] = { far_, 0, 0, 0 };
    bgfx::setUniform(u_depth_, depth);
    bgfx::setTexture(0, s_color_, earthColor_);
    bgfx::setTexture(1, s_bump_,  earthBump_);   // also read by the vertex shader
    bgfx::setTexture(2, s_night_, earthNight_);
    bgfx::setTexture(3, s_rough_, earthRough_);
    bgfx::setTexture(4, s_cloud_, earthCloud_);  // cloud shadows on the ground

    if (!patchActive) {                          // global sphere = far field / high up
        bgfx::setTransform(f.model.m);
        bgfx::setVertexBuffer(0, g.vbh);
        bgfx::setIndexBuffer(g.ibh);
        bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_WRITE_Z
                       | BGFX_STATE_DEPTH_TEST_LESS | BGFX_STATE_CULL_CW);
        bgfx::submit(0, earthProg_);
    }

    // --- Terrain LOD patch (near-surface): a density-graded cap that follows the
    // camera, covering the visible ground to the horizon (dense underfoot, coarse
    // toward the limb) and displacing the height map at full resolution. SOLE
    // surface while active (no global sphere -> no z-fight), reuses fs_earth for
    // identical shading, and built in a floating-origin frame so the fine geometry
    // stays precise/stable at planet scale. See vs_patch.sc.
    if (patchActive) {
        const GpuMesh& pm = meshes_[patchMesh_ - 1];
        float Rkm = (float)EARTH_RADIUS_KM;

        // True sub-camera direction (view space) and altitude. These drive the
        // floating-origin reconstruction in the shader, so they must stay exact.
        RVec3 Cvt    = rmath::normalize(rmath::sub(f.cam_pos, f.center));
        float altt   = fmaxf(camAlt, 0.02f);

        // World-anchored cap: build the patch geometry from a sub-camera point that
        // is SNAPPED to the height map's texel grid (and an altitude snapped to a
        // geometric ladder), not from the live camera. The grid is camera-locked in
        // *topology*, so without this its vertices slide across the height field as
        // you move and the surface swims. Snapping freezes every vertex's world
        // position for sub-texel camera motion (zero swim); on a texel crossing the
        // cap shifts by exactly one texel of a texel-resolution field, which is
        // imperceptible. The shader reconstructs wpos = center + dir*(R+disp) from
        // the snapped dir, so it is independent of the true camera between snaps.
        constexpr float kTexel    = 3.14159265f / 16384.0f;  // rad/texel (16384 rows over pi)
        constexpr int   kSnapTex  = 1;                        // snap granularity, in texels
        constexpr float kAltLadder = 0.06f;                  // altitude snap: ~4% geometric steps
        const float snap = kSnapTex * kTexel;

        // Snap the sub-camera point on the lon/colat grid (== the height-map grid).
        // view->body is (x,-z,y) (inverse of viewBasis), matching vs_patch.
        RVec3 bC   = { Cvt.x, -Cvt.z, Cvt.y };
        float lon  = atan2f(bC.y, bC.x);
        float cola = acosf(fmaxf(-1.0f, fminf(1.0f, bC.z)));
        lon  = roundf(lon  / snap) * snap;
        cola = roundf(cola / snap) * snap;
        cola = fmaxf(snap, fminf(3.14159265f - snap, cola));
        float sc = sinf(cola);
        RVec3 bS = { sc * cosf(lon), sc * sinf(lon), cosf(cola) };
        RVec3 Cv = { bS.x, bS.z, -bS.y };                    // snapped centre (view space)

        // Snapped altitude (ceil to a geometric ladder so the cap is stable AND
        // always over-covers the true horizon -- no limb gap).
        float alts = (altt > 0.05f)
                   ? exp2f(ceilf(log2f(altt) / kAltLadder) * kAltLadder) : altt;

        // Geographic tangent frame at the snapped centre (view +Y = north pole).
        RVec3 pole = fabsf(Cv.y) < 0.99f ? RVec3{0,1,0} : RVec3{1,0,0};
        RVec3 Ev   = rmath::normalize(rmath::cross(pole, Cv));
        RVec3 Nv   = rmath::cross(Cv, Ev);
        // Cap reaches past the geometric horizon (snapped alt) so it covers all
        // visible ground; margin also absorbs the altitude snap step.
        float horizon = acosf(Rkm / (Rkm + alts));
        float tanHa   = tanf(fminf(horizon * 1.08f, 1.4f));

        float pc[4]   = { Cv.x, Cv.y, Cv.z, tanHa };
        float pe[4]   = { Ev.x, Ev.y, Ev.z, Rkm };
        float pn[4]   = { Nv.x, Nv.y, Nv.z, alts };
        float pcam[4] = { f.cam_pos.x, f.cam_pos.y, f.cam_pos.z, 8.849f };  // disp scale (km)
        float ptr[4]  = { Cvt.x, Cvt.y, Cvt.z, altt };       // true centre/alt: reconstruction
        bgfx::setUniform(u_patchC_, pc);
        bgfx::setUniform(u_patchE_, pe);
        bgfx::setUniform(u_patchN_, pn);
        bgfx::setUniform(u_patchCam_, pcam);
        bgfx::setUniform(u_patchTrue_, ptr);
        bgfx::setTransform(rmath::identity().m);   // positions already in world km
        bgfx::setVertexBuffer(0, pm.vbh);
        bgfx::setIndexBuffer(pm.ibh);
        bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_WRITE_Z
                       | BGFX_STATE_DEPTH_TEST_LESS);   // no cull: graded cap faces camera
        bgfx::submit(0, patchProg_);
    }

    // Cloud shells: concentric cloud-map layers above the terrain. Drawn
    // inner->outer (back-to-front for the visible near hemisphere) with depth
    // test on (mountains occlude clouds) but no depth write (shells blend). The
    // slight radius offsets give a parallax / volumetric feel as the camera moves.
    // Fade the shells out as the camera descends into the 2-12 km layer: seen from
    // below/inside, the translucent spheres wash out the ground and you can't tell
    // cloud from terrain. Full clouds above ~35 km, gone by ~10 km.
    float cloudFade = (camAlt - 10.0f) / 25.0f;
    cloudFade = cloudFade < 0.0f ? 0.0f : (cloudFade > 1.0f ? 1.0f : cloudFade);
    if (cloudMesh_ && cloudFade > 0.001f) {
        const GpuMesh& cm  = meshes_[cloudMesh_ - 1];
        const int   kShells    = 5;
        const float baseKm     = 2.0f;    // cloud base ~2 km (low cumulus)
        const float gapKm      = 2.5f;    // shells span 2..12 km -> tops at the tropopause
        const float shellAlpha = 0.22f * cloudFade;  // per-shell opacity (x altitude fade);
                                          // base 0.22 thin enough the ground shadow shows
                                          // through (keep in sync with fs_earth shadow curve)
        const float dispAmp    = 3000.0f; // noise amplitude (m); ~> gap so shells merge
        const float dispFreq   = 10.0f;    // noise feature scale over the sphere
        for (int i = 0; i < kShells; ++i) {
            float factor = 1.0f + ((baseKm + i * gapKm) * 1000.0f) / (float)EARTH_RADIUS_M;
            RMat4 m = rmath::mul(f.model, rmath::scale(factor));
            setVec4(u_sunDir_, f.sun_dir);
            setVec4(u_earthCenter_, f.center);
            float ca[4] = { shellAlpha, 0, 0, 0 };
            bgfx::setUniform(u_cloudAlpha_, ca);
            float cd[4] = { dispAmp, dispFreq, i * 17.0f, 0 };   // per-shell seed
            bgfx::setUniform(u_cloudDisp_, cd);
            bgfx::setUniform(u_depth_, depth);
            bgfx::setTexture(0, s_cloud_, earthCloud_);
            bgfx::setTexture(1, s_night_, bgfx::isValid(earthNight_) ? earthNight_ : white_);  // city-light colour
            bgfx::setTexture(2, s_emiss_, bgfx::isValid(earthEmiss_) ? earthEmiss_ : white_);  // city-light strength
            bgfx::setTransform(m.m);
            bgfx::setVertexBuffer(0, cm.vbh);
            bgfx::setIndexBuffer(cm.ibh);
            bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A
                           | BGFX_STATE_DEPTH_TEST_LESS | BGFX_STATE_BLEND_ALPHA | BGFX_STATE_CULL_CW);
            bgfx::submit(0, cloudProg_);
        }
    }

    // --- Sun + lens flare (screen space, additive, on top of the 3D scene) ---
    if (bgfx::isValid(flareProg_)) {
        RVec3 fwd   = rmath::normalize(rmath::sub(cam_.target, cam_.position));
        RVec3 right = rmath::normalize(rmath::cross(fwd, cam_.up));
        RVec3 up    = rmath::cross(right, fwd);
        float tanH  = tanf(cam_.fovy * 0.5f * (float)(M_PI / 180.0));
        float aspect = height_ > 0 ? (float)width_ / (float)height_ : 1.0f;

        RVec3 S  = f.sun_dir;                       // unit, view space
        float sf = rmath::dot(S, fwd);              // > 0: sun in front of camera
        auto smooth01 = [](float a, float b, float x) {
            float t = (x - a) / (b - a); t = t < 0 ? 0 : (t > 1 ? 1 : t); return t*t*(3.0f - 2.0f*t);
        };
        if (sf > 0.02f) {
            // Sun screen position (NDC), inverse of the ray reconstruction.
            float nx = (rmath::dot(S, right) / sf) / (tanH * aspect);
            float ny = (rmath::dot(S, up)    / sf) / tanH;

            // Soft occlusion: fade as the sun passes behind the Earth's disk.
            RVec3 toC = rmath::sub(f.center, f.cam_pos);
            float dC  = rmath::length(toC);
            float cosA = dC > 1e-3f ? rmath::dot(S, RVec3{ toC.x/dC, toC.y/dC, toC.z/dC }) : -1.0f;
            float sunAng   = acosf(fmaxf(-1.0f, fminf(1.0f, cosA)));
            float earthAng = asinf(fminf(1.0f, (float)EARTH_RADIUS_KM / fmaxf(dC, (float)EARTH_RADIUS_KM)));
            float occ      = smooth01(earthAng * 0.95f, earthAng * 1.10f, sunAng);
            float edge     = fmaxf(fabsf(nx), fabsf(ny));
            float onScreen = 1.0f - smooth01(1.3f, 2.4f, edge);
            float master   = occ * onScreen;

            if (master > 0.003f) {
                std::vector<Vertex> fv;
                fv.reserve(9 * 6);
                auto add = [&](float t, float size, RColor c, float inten, float power, float ring) {
                    float cx = nx * (1.0f - t), cy = ny * (1.0f - t);   // along sun -> screen centre
                    float a  = fminf(1.0f, inten * master);
                    RColor col = { c.r, c.g, c.b, (unsigned char)(a * 255.0f) };
                    float sx = size / aspect, sy = size;
                    Vertex v0{ {cx-sx, cy-sy, 0}, {power, ring, 0}, -1, -1, col };
                    Vertex v1{ {cx+sx, cy-sy, 0}, {power, ring, 0},  1, -1, col };
                    Vertex v2{ {cx+sx, cy+sy, 0}, {power, ring, 0},  1,  1, col };
                    Vertex v3{ {cx-sx, cy+sy, 0}, {power, ring, 0}, -1,  1, col };
                    fv.push_back(v0); fv.push_back(v1); fv.push_back(v2);
                    fv.push_back(v0); fv.push_back(v2); fv.push_back(v3);
                };
                // Sun: warm glow + white-hot core.
                add(0.00f, 0.30f, { 255, 235, 200, 255 }, 0.60f,  2.0f, 0.0f);
                add(0.00f, 0.05f, { 255, 255, 255, 255 }, 1.00f, 10.0f, 0.0f);
                // Chromatic ghosts along the sun -> centre axis.
                add(0.30f, 0.06f, { 120, 160, 255, 255 }, 0.15f,  3.0f, 0.0f);
                add(0.50f, 0.11f, { 255, 180,  90, 255 }, 0.12f,  2.0f, 0.0f);
                add(0.62f, 0.04f, { 150, 255, 150, 255 }, 0.12f,  3.0f, 0.0f);
                add(0.80f, 0.15f, { 255, 110,  90, 255 }, 0.09f,  1.5f, 0.0f);
                add(1.10f, 0.08f, { 130, 150, 255, 255 }, 0.12f,  2.0f, 0.0f);
                add(1.30f, 0.20f, { 255, 210, 140, 255 }, 0.07f,  1.2f, 0.0f);
                // Halo ring around the screen centre.
                add(1.00f, 0.55f, { 150, 180, 255, 255 }, 0.06f,  1.0f, 0.78f);

                uint32_t n = (uint32_t)fv.size();
                if (bgfx::getAvailTransientVertexBuffer(n, layout_) >= n) {
                    bgfx::TransientVertexBuffer tvb;
                    bgfx::allocTransientVertexBuffer(&tvb, n, layout_);
                    std::memcpy(tvb.data, fv.data(), n * sizeof(Vertex));
                    bgfx::setVertexBuffer(0, &tvb);
                    bgfx::setState(BGFX_STATE_WRITE_RGB | BGFX_STATE_WRITE_A | BGFX_STATE_BLEND_ADD);
                    bgfx::submit(1, flareProg_);   // view 1 = over the 3D scene, under the UI
                }
            }
        }
    }
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
    float depth[4] = { far_, 0, 0, 0 };   // unused by 2D (no depth test) but keeps u_depth defined
    bgfx::setUniform(u_depth_, depth);
    float unlit[4] = { 0, 0, 0, 0 };      // 2D overlay is flat-coloured
    bgfx::setUniform(u_light_, unlit);
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
