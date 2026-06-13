$input v_texcoord0, v_color0, v_normal, v_wpos, v_objpos

#include <bgfx_shader.sh>

uniform vec4 u_light;    // xyz: sun direction (view)
uniform vec4 u_camPos;   // xyz: camera (view)
uniform vec4 u_heat;     // xyz: travel direction (view); w: aerodynamic heating [0,1]

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
float fbm(vec3 p) {
    float f = 0.0, a = 0.5;
    for (int i = 0; i < 4; i++) { f += a * snoise(p); p *= 2.03; a *= 0.5; }
    return f * 0.5 + 0.5;   // -> [0,1]-ish
}

void main() {
    vec3  albedo    = v_color0.rgb;
    float metallic  = v_texcoord0.x;
    float roughness = clamp(v_texcoord0.y, 0.05, 1.0);

    // --- Procedural grime (object space, so it sticks to the hull) ---
    vec3 p = v_objpos;
    float blotch = fbm(p * 1.4);
    float ang    = atan(p.y, p.x);
    float streak = fbm(vec3(ang * 1.6, p.z * 0.45, 4.0));   // soot streaks down the body
    float grime  = smoothstep(0.65, 1.0, 0.55*blotch + 0.25*streak);
    albedo   *= mix(1.0, 0.80, grime);                      // subtle soot/stains
    roughness = clamp(mix(roughness, 0.7, grime * 0.35), 0.05, 1.0);
    albedo   *= 0.96 + 0.04 * fbm(p * 6.0);                 // faint unevenness

    // --- Cook-Torrance PBR (single sun + cheap sky ambient) ---
    vec3 N = normalize(v_normal);
    vec3 V = normalize(u_camPos.xyz - v_wpos);
    vec3 L = normalize(u_light.xyz);
    vec3 H = normalize(L + V);
    float NdL = max(dot(N, L), 0.0);
    float NdV = max(dot(N, V), 1e-4);
    float NdH = max(dot(N, H), 0.0);
    float VdH = max(dot(V, H), 0.0);

    float a = roughness * roughness, a2 = a * a;
    float D = a2 / (3.14159265 * pow(NdH*NdH*(a2 - 1.0) + 1.0, 2.0));
    float k = (roughness + 1.0)*(roughness + 1.0) / 8.0;
    float G = (NdV/(NdV*(1.0 - k) + k)) * (NdL/(NdL*(1.0 - k) + k));
    vec3  F0 = mix(vec3(0.04, 0.04, 0.04), albedo, metallic);
    vec3  F  = F0 + (1.0 - F0) * pow(1.0 - VdH, 5.0);
    vec3  spec = (D * G) * F / (4.0 * NdV * NdL + 1e-4);
    vec3  kd = (1.0 - F) * (1.0 - metallic);

    vec3 sun = vec3(1.35, 1.33, 1.28);
    vec3 Lo  = (kd * albedo / 3.14159265 + spec) * sun * NdL;

    // Ambient: dielectric diffuse + metal picks up a tinted sky reflection so it
    // is not pure black where the sun does not hit.
    vec3 sky = vec3(0.34, 0.40, 0.52);
    vec3 amb = albedo * sky * 0.45 * (1.0 - 0.6*metallic) + F0 * sky * 0.55;

    vec3 color = Lo + amb;

    // --- Aerodynamic heating: windward faces glow incandescent (ablation) ---
    float h = u_heat.w;
    if (h > 0.001) {
        float wind = max(dot(N, normalize(u_heat.xyz)), 0.0);   // facing into the airflow
        float glow = pow(wind, 1.5) * h;
        // Blackbody-ish ramp: dull red -> orange -> white as heating rises.
        vec3 hot = mix(vec3(0.7, 0.06, 0.0), vec3(1.0, 0.45, 0.08), smoothstep(0.0, 0.45, h));
        hot = mix(hot, vec3(1.0, 0.92, 0.75), smoothstep(0.5, 1.0, h));
        color += hot * glow * 2.2;
    }

    gl_FragColor = vec4(color, 1.0);
}
