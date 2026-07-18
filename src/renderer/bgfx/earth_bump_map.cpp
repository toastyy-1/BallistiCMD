#include "earth_bump_map.hpp"

#include <bx/allocator.h>
#include <bimg/decode.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>

namespace renderer {

namespace {

bx::DefaultAllocator s_bumpAllocator;

std::vector<uint8_t> readFile(const char* path) {
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) return {};
    std::streamsize n = f.tellg();
    f.seekg(0);
    std::vector<uint8_t> buf((size_t)n);
    f.read((char*)buf.data(), n);
    return buf;
}

long wrap_column(long x, long w) {
    x %= w;
    if (x < 0) x += w;
    return x;
}

long clamp_row(long y, long h) {
  long y_clamped = y >= h ? h - 1 : y;
  return y < 0 ? 0L : y_clamped;
}

}

void EarthBumpMap::Load(const char* path) {
    std::vector<uint8_t> data = readFile(path);
    if (data.empty()) { std::fprintf(stderr, "EarthBumpMap: missing %s\n", path); return; }

    bimg::ImageContainer* ic = bimg::imageParse(&s_bumpAllocator, data.data(), (uint32_t)data.size());
    if (!ic) { std::fprintf(stderr, "EarthBumpMap: parse failed %s\n", path); return; }

    const uint64_t flags = BGFX_SAMPLER_V_CLAMP | BGFX_SAMPLER_MIN_ANISOTROPIC | BGFX_SAMPLER_MAG_ANISOTROPIC;
    tex_ = bgfx::createTexture2D(
        (uint16_t)ic->m_width, (uint16_t)ic->m_height, ic->m_numMips > 1, ic->m_numLayers,
        (bgfx::TextureFormat::Enum)ic->m_format, flags, bgfx::copy(ic->m_data, ic->m_size));

    if (bimg::ImageMip mip; bimg::imageGetRawData(*ic, 0, 0, ic->m_data, ic->m_size, mip)) {
        w_ = mip.m_width;
        h_ = mip.m_height;
        heights_.resize((size_t)w_ * h_);
        const uint32_t kStrip = 512;
        for (uint32_t y = 0; y < h_; y += kStrip) {
            uint32_t hs = std::min(kStrip, h_ - y);
            const uint8_t* src = mip.m_data + (size_t)y * w_;
            uint8_t*       dst = heights_.data() + (size_t)y * w_;
            bimg::imageDecodeToR8(&s_bumpAllocator, dst, src, w_, hs, 1, w_, mip.m_format);
        }
    } else {
        std::fprintf(stderr, "EarthBumpMap: imageGetRawData failed %s\n", path);   // couldn't find mip 0; leave heights empty.
    }

    bimg::imageFree(ic);
}

void EarthBumpMap::Destroy() {
    if (bgfx::isValid(tex_)) bgfx::destroy(tex_);
    tex_ = BGFX_INVALID_HANDLE;
    heights_.clear();
    heights_.shrink_to_fit();
    w_ = h_ = 0;
}

double EarthBumpMap::sampleHeight01(const Vec3& r) const {
    if (heights_.empty()) return 0.0;
    Vec3 dir = r.normalized();

    double u = std::atan2(dir.y, dir.x) * (0.5 / M_PI) + 0.5;
    double v = std::acos(std::clamp(dir.z, -1.0, 1.0)) / M_PI;

    double fract_u = u * (double)w_ - 0.5;
    double fract_v = v * (double)h_ - 0.5;
    double floor_u = std::floor(fract_u);
    double floor_v = std::floor(fract_v);
    double blend_weight_u = fract_u - floor_u;
    double blend_weight_v = fract_v - floor_v;

    long W = (long)w_;
    long H = (long)h_;
    long x0 = wrap_column((long)floor_u, W);
    long x1 = wrap_column((long)floor_u + 1, W);
    long y0 = clamp_row((long)floor_v, H);
    long y1 = clamp_row((long)floor_v + 1, H);

    auto at = [&](long x, long y) { return heights_[(size_t)y * w_ + x] / 255.0; };
    double top = at(x0, y0) * (1.0 - blend_weight_u) + at(x1, y0) * blend_weight_u;
    double bot = at(x0, y1) * (1.0 - blend_weight_u) + at(x1, y1) * blend_weight_u;
    return top * (1.0 - blend_weight_v) + bot * blend_weight_v;
}

double EarthBumpMap::Elevation(const Vec3& r) const {
    return sampleHeight01(r) * kMaxElevation;
}

double EarthBumpMap::SurfaceRadius(const Vec3& r) const {
    return EARTH_RADIUS + Elevation(r);
}

double EarthBumpMap::SurfaceRadius(double lat_deg, double lon_deg) const {
    double lat = lat_deg * (M_PI / 180.0);
    double lon = lon_deg * (M_PI / 180.0);
    double cl = std::cos(lat);
    Vec3 dir{ cl * std::cos(lon), cl * std::sin(lon), std::sin(lat) };
    return SurfaceRadius(dir);
}

double EarthBumpMap::Altitude(const Vec3& eci_m) const {
    return eci_m.norm() - SurfaceRadius(eci_m);
}

}
