$input a_position
$output v_texcoord0, v_wpos, v_objpos

#include <bgfx_shader.sh>

// Camera-following terrain LOD patch. A flat [-1,1]^2 grid (a_position.xy = grid
// coords) becomes a density-graded spherical cap centred under the camera: dense
// underfoot, reaching the horizon at the edge, displacing the height map at full
// resolution. While active it is the SOLE Earth surface (the global sphere is not
// drawn), so there is no z-fighting, and it is built in a floating-origin frame
// (positions relative to the camera) so fine geometry doesn't shimmer at planet
// scale. It outputs the earth's varyings so fs_earth shades it identically.
SAMPLER2D(s_bump, 1);          // height map (same texture as the earth)
uniform vec4 u_depth;          // x: far plane (logarithmic depth)
uniform vec4 u_patchC;         // xyz: view-space dir under camera, w: tan(half-angle)
uniform vec4 u_patchE;         // xyz: view-space east tangent,  w: earth radius (km)
uniform vec4 u_patchN;         // xyz: view-space north tangent, w: camera altitude (km)
uniform vec4 u_patchCam;       // xyz: camera position (world km), w: displacement scale (km)

void main() {
    vec3  Cv        = u_patchC.xyz;
    float tanHa     = u_patchC.w;
    float Rkm       = u_patchE.w;
    float alt       = u_patchN.w;
    float dispScale = u_patchCam.w;

    // Density-graded grid: the exponent packs vertices toward the centre (under
    // the camera) while the edges still stretch out to the horizon.
    float gx = a_position.x, gy = a_position.y;
    float wx = sign(gx) * pow(abs(gx), 2.5);
    float wy = sign(gy) * pow(abs(gy), 2.5);

    // View-space direction from the planet centre to this vertex (gnomonic: offset
    // in the tangent plane by tan(angle), then renormalise onto the sphere).
    vec3 dir = normalize(Cv + u_patchE.xyz * (wx * tanHa) + u_patchN.xyz * (wy * tanHa));

    // Body (mesh) direction for UV + frame-stable noise. viewBasis maps body
    // (x,y,z) -> view (x,z,-y), so its inverse is (x,-z,y).
    vec3 bdir = vec3(dir.x, -dir.z, dir.y);
    vec2 uv = vec2(atan(bdir.y, bdir.x) * (0.5/3.14159265) + 0.5,
                   acos(clamp(bdir.z, -1.0, 1.0)) / 3.14159265);

    // Band-limit the displacement to the local vertex spacing (mip-correct height
    // sampling). The grid is camera-locked, so without this its vertices slide over
    // the height map as you move and the under-sampled surface oscillates. arcN is
    // the world spacing between adjacent vertices along each axis (the graded grid
    // makes it vary); 1223 m is the map's texel size. The altitude term additionally
    // coarsens the patch toward the global sphere's resolution near the handoff, so
    // the switch isn't a hard pop.
    float Rm   = Rkm * 1000.0;
    float ox   = wx * tanHa, oy = wy * tanHa;
    float arcx = Rm * (2.5 * pow(abs(gx), 1.5) * tanHa) / (1.0 + ox*ox) * (2.0/192.0);
    float arcy = Rm * (2.5 * pow(abs(gy), 1.5) * tanHa) / (1.0 + oy*oy) * (2.0/192.0);
    float lod  = max(log2(max(max(arcx, arcy) / 1223.0, 1.0)),
                     smoothstep(40.0, 90.0, alt) * 5.0);
    float h    = texture2DLod(s_bump, uv, lod).x;
    float disp = h * dispScale;   // km

    // Floating-origin world position (km). Every term is small near the camera --
    // R*(dir-Cv) is the arc offset from the sub-camera point -- so there is no
    // catastrophic cancellation and the fine relief stays stable frame to frame.
    vec3 wpos = u_patchCam.xyz + (dir - Cv) * Rkm + dir * disp - Cv * alt;

    v_wpos      = wpos;                          // world (shifted) km
    v_objpos    = bdir * (Rkm * 1000.0);         // body metres (frame-stable noise domain)
    v_texcoord0 = uv;
    gl_Position = mul(u_modelViewProj, vec4(wpos, 1.0));   // patch transform is identity
    gl_Position.z = (log2(max(1e-6, 1.0 + gl_Position.w)) * (2.0/log2(u_depth.x + 1.0)) - 1.0) * gl_Position.w;
}
