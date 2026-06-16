$input a_position, a_normal, a_texcoord0
$output v_texcoord0, v_wpos

#include <bgfx_shader.sh>

uniform vec4 u_depth;       // x: far plane (logarithmic depth)
uniform vec4 u_cloudDisp;   // x: amplitude (m), y: frequency, z: per-shell seed

// --- Ashima/Gustavson 3D simplex noise (public domain) ----------------------
vec4 mod289(vec4 x) { return x - floor(x * (1.0/289.0)) * 289.0; }
vec3 mod289(vec3 x) { return x - floor(x * (1.0/289.0)) * 289.0; }
vec4 permute(vec4 x) { return mod289(((x*34.0)+1.0)*x); }
vec4 taylorInvSqrt(vec4 r) { return 1.79284291400159 - 0.85373472095314 * r; }

float snoise(vec3 v) {
    const vec2 C = vec2(1.0/6.0, 1.0/3.0);
    const vec4 D = vec4(0.0, 0.5, 1.0, 2.0);
    vec3 i  = floor(v + dot(v, C.yyy));
    vec3 x0 = v - i + dot(i, C.xxx);
    vec3 g = step(x0.yzx, x0.xyz);
    vec3 l = 1.0 - g;
    vec3 i1 = min(g.xyz, l.zxy);
    vec3 i2 = max(g.xyz, l.zxy);
    vec3 x1 = x0 - i1 + C.xxx;
    vec3 x2 = x0 - i2 + C.yyy;
    vec3 x3 = x0 - D.yyy;
    i = mod289(i);
    vec4 p = permute(permute(permute(
               i.z + vec4(0.0, i1.z, i2.z, 1.0))
             + i.y + vec4(0.0, i1.y, i2.y, 1.0))
             + i.x + vec4(0.0, i1.x, i2.x, 1.0));
    float n_ = 1.0/7.0;
    vec3 ns = n_ * D.wyz - D.xzx;
    vec4 j = p - 49.0 * floor(p * ns.z * ns.z);
    vec4 x_ = floor(j * ns.z);
    vec4 y_ = floor(j - 7.0 * x_);
    vec4 x = x_ * ns.x + ns.yyyy;
    vec4 y = y_ * ns.x + ns.yyyy;
    vec4 h = 1.0 - abs(x) - abs(y);
    vec4 b0 = vec4(x.xy, y.xy);
    vec4 b1 = vec4(x.zw, y.zw);
    vec4 s0 = floor(b0)*2.0 + 1.0;
    vec4 s1 = floor(b1)*2.0 + 1.0;
    vec4 sh = -step(h, vec4(0.0));
    vec4 a0 = b0.xzyw + s0.xzyw*sh.xxyy;
    vec4 a1 = b1.xzyw + s1.xzyw*sh.zzww;
    vec3 p0 = vec3(a0.xy, h.x);
    vec3 p1 = vec3(a0.zw, h.y);
    vec3 p2 = vec3(a1.xy, h.z);
    vec3 p3 = vec3(a1.zw, h.w);
    vec4 norm = taylorInvSqrt(vec4(dot(p0,p0), dot(p1,p1), dot(p2,p2), dot(p3,p3)));
    p0 *= norm.x; p1 *= norm.y; p2 *= norm.z; p3 *= norm.w;
    vec4 m = max(0.6 - vec4(dot(x0,x0), dot(x1,x1), dot(x2,x2), dot(x3,x3)), 0.0);
    m = m * m;
    return 42.0 * dot(m*m, vec4(dot(p0,x0), dot(p1,x1), dot(p2,x2), dot(p3,x3)));
}

// Fractal Brownian motion: a few octaves for turbulent cloud relief.
float fbm(vec3 p) {
    float f = 0.0, amp = 0.5;
    for (int i = 0; i < 3; i++) { f += amp * snoise(p); p *= 2.03; amp *= 0.5; }
    return f;
}

void main() {
    // Perturb the shell along its (radial) normal by noise so the concentric
    // layers interleave into a uniform mass instead of stacked sheets. The seed
    // differs per shell so they don't nest cleanly.
    float n = fbm(a_normal * u_cloudDisp.y + vec3(u_cloudDisp.z, u_cloudDisp.z, u_cloudDisp.z));
    vec3  disp = a_position + a_normal * (n * u_cloudDisp.x);

    vec4 wpos   = mul(u_model[0], vec4(disp, 1.0));
    v_wpos      = wpos.xyz;
    gl_Position = mul(u_modelViewProj, vec4(disp, 1.0));
    gl_Position.z = (log2(max(1e-6, 1.0 + gl_Position.w)) * (2.0/log2(u_depth.x + 1.0)) - 1.0) * gl_Position.w;
    v_texcoord0 = a_texcoord0;
}
