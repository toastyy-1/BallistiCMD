$input v_texcoord0, v_color0, v_normal, v_wpos, v_objpos

#include <bgfx_shader.sh>

uniform vec4 u_light;    // xyz: sun direction (view)
uniform vec4 u_camPos;   // xyz: camera (view)
uniform vec4 u_heat;     // xyz: travel direction (view); w: aerodynamic heating [0,1]
uniform vec4 u_earth;    // xyz: Earth centre (view); w: radius (km) -- for reflections
SAMPLER2D(s_color, 0);   // Earth albedo (analytic reflection)
SAMPLER2D(s_night, 1);   // Earth city lights (analytic reflection)
SAMPLER2D(s_cloud, 2);   // Earth clouds (cloud shadow on the rocket)

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

// Lit Earth colour seen along a view-space direction from the planet centre
// (for the analytic reflection). Inverts the (x,z,-y) view basis to ECI to get
// the equirectangular UV, then applies the same day/night blend as the Earth.
vec3 earthRefl(vec3 dv, vec3 L) {
    vec3 e = vec3(dv.x, -dv.z, dv.y);                       // view -> ECI (+Z north)
    float u  = atan(e.y, e.x) * (0.5 / 3.14159265) + (12.0 / 360.0);
    float vt = acos(clamp(e.z, -1.0, 1.0)) / 3.14159265;
    vec2 uv = vec2(u, vt);
    vec3 ec = texture2D(s_color, uv).rgb;
    vec3 nc = texture2D(s_night, uv).rgb; nc = nc * nc * 1.3;
    float d = dot(dv, L);
    float t = smoothstep(-0.15, 0.15, d);
    return mix(nc, ec * (0.2 + 0.8 * max(d, 0.0)), t);
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

    // Cloud shadow: if a cloud lies between this point and the sun, dim the sun
    // term (ray to the sun exits the cloud layer above -> sample cover there).
    float Rc = u_earth.w + 60.0;
    vec3  oc2 = v_wpos - u_earth.xyz;
    float b2 = dot(oc2, L);
    float disc2 = b2*b2 - (dot(oc2, oc2) - Rc*Rc);
    if (disc2 > 0.0) {
        float tc = -b2 + sqrt(disc2);
        if (tc > 0.0) {
            vec3 cd = normalize((v_wpos + L*tc) - u_earth.xyz);
            vec3 ce = vec3(cd.x, -cd.z, cd.y);
            vec2 cuv = vec2(atan(ce.y, ce.x) * (0.5/3.14159265) + 12.0/360.0,
                            acos(clamp(ce.z, -1.0, 1.0)) / 3.14159265);
            Lo *= 1.0 - 0.7 * texture2D(s_cloud, cuv).x;
        }
    }

    // --- Analytic ray-traced reflection: the metal mirrors the actual Earth ---
    vec3  Rdir = reflect(-V, N);
    vec3  oc   = v_wpos - u_earth.xyz;
    float bb   = dot(oc, Rdir);
    float cc   = dot(oc, oc) - u_earth.w * u_earth.w;
    float disc = bb*bb - cc;
    vec3  env;
    if (disc > 0.0 && (-bb - sqrt(disc)) > 0.0) {
        vec3 hit = v_wpos + Rdir * (-bb - sqrt(disc));     // reflection hits the planet
        env = earthRefl(normalize(hit - u_earth.xyz), L);
    } else {
        env = vec3(0.0, 0.0, 0.0);                         // reflection sees space
        env += vec3(1.0, 0.97, 0.9) * pow(max(dot(Rdir, L), 0.0), 600.0) * 4.0;  // sun glint
    }
    vec3 kS = F0 + (max(vec3(1.0 - roughness, 1.0 - roughness, 1.0 - roughness), F0) - F0)
                   * pow(1.0 - NdV, 5.0);
    vec3 envRefl = env * kS;

    // Ambient: a little dielectric sky fill + the ray-traced environment reflection.
    vec3 sky = vec3(0.34, 0.40, 0.52);
    vec3 amb = albedo * sky * 0.32 * (1.0 - 0.6*metallic) + envRefl;

    vec3 color = Lo + amb;

    // Earthshine: single-bounce fill from the planet below (bluish bounce light).
    vec3  toE = normalize(u_earth.xyz - v_wpos);
    float es  = max(dot(N, toE), 0.0);
    color += albedo * vec3(0.16, 0.26, 0.36) * (es * 0.5);

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
