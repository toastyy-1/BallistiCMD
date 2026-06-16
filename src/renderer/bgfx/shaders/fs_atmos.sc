$input v_texcoord0

#include <bgfx_shader.sh>

SAMPLER2D(s_night, 0);       // city-light colour (light pollution hue)
SAMPLER2D(s_emiss, 1);       // white = city-light strength
uniform vec4 u_sunDir;       // xyz: view-space direction TO the sun
uniform vec4 u_earthCenter;  // xyz: planet centre (view-space km)
uniform vec4 u_camPos;       // xyz: camera (view-space km)
uniform vec4 u_atmos;        // x: planet radius, y: atmosphere radius, z: scale height, w: exposure (km)
uniform vec4 u_rayFwd;       // camera forward
uniform vec4 u_rayRight;     // camera right * tan(fovy/2) * aspect
uniform vec4 u_rayUp;        // camera up    * tan(fovy/2)

// Nearest positive root of |o + t d - c|^2 = r^2, plus the far root in .y.
// Returns t in .x/.y; .x < 0 means no (forward) hit.
vec2 raySphere(vec3 o, vec3 d, vec3 c, float r) {
    vec3  oc = o - c;
    float b  = dot(oc, d);
    float cc = dot(oc, oc) - r*r;
    float disc = b*b - cc;
    if (disc < 0.0) return vec2(-1.0, -1.0);
    float s = sqrt(disc);
    return vec2(-b - s, -b + s);
}

void main() {
    vec3 ro = u_camPos.xyz;
    vec3 rd = normalize(u_rayFwd.xyz + u_rayRight.xyz * v_texcoord0.x + u_rayUp.xyz * v_texcoord0.y);
    vec3 C  = u_earthCenter.xyz;
    vec3 L  = normalize(u_sunDir.xyz);
    float Rp = u_atmos.x, Ra = u_atmos.y, H = u_atmos.z;

    // Segment of the ray inside the atmosphere, clipped to the front of the planet.
    vec2 atm = raySphere(ro, rd, C, Ra);
    if (atm.y <= 0.0) { gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0); return; }  // misses atmosphere
    float t0 = max(atm.x, 0.0);
    float t1 = atm.y;
    vec2 pl = raySphere(ro, rd, C, Rp);
    if (pl.x > 0.0) t1 = min(t1, pl.x);   // stop at the planet surface
    float seg = t1 - t0;
    if (seg <= 0.0) { gl_FragColor = vec4(0.0, 0.0, 0.0, 1.0); return; }

    // Integrate sun-lit air density along the segment (single scattering). In the
    // same pass accumulate night-side light pollution: where the air parcel is in
    // shadow, the city lights directly below it scatter back toward the camera, so
    // the night-side limb air picks up the cities' colour. Strength comes from the
    // clean emission mask, hue from the colour night map; sampled at a coarse mip
    // because the glow is diffuse (also keeps the full-screen pass cheap).
    const int N = 8;
    const float Hc = 12.0;        // light-pollution scale height (km): hugs the low air
    float od = 0.0;
    vec3  city = vec3(0.0);
    for (int i = 0; i < N; i++) {
        float t = t0 + (float(i) + 0.5) / float(N) * seg;
        vec3  p = ro + rd * t;
        vec3  dp = normalize(p - C);
        float h = length(p - C) - Rp;
        float lit = smoothstep(-0.10, 0.25, dot(dp, L));
        od += exp(-max(h, 0.0) / H) * lit;

        float night = 1.0 - smoothstep(-0.10, 0.10, dot(dp, L));
        if (night > 0.001) {
            vec3 e  = vec3(dp.x, -dp.z, dp.y);                 // view -> ECI (+Z north)
            vec2 uv = vec2(atan(e.y, e.x) * (0.5/3.14159265) + 0.5,
                           acos(clamp(e.z, -1.0, 1.0)) / 3.14159265);
            float em  = texture2DLod(s_emiss, uv, 4.0).x;     // strength
            vec3  hue = texture2DLod(s_night, uv, 4.0).rgb;   // colour
            city += hue * em * exp(-max(h, 0.0) / Hc) * night;
        }
    }
    od   *= (seg / float(N)) / H;    // optical depth in scale-height units
    city *= (seg / float(N)) / Hc;   // night-side column glow

    // Phase functions: Rayleigh (broad, blue) + Mie (forward lobe, warm).
    float mu = dot(rd, L);
    float Pr = 0.75 * (1.0 + mu * mu);
    float g  = 0.76;
    float Pm = (1.0 - g*g) / (4.0*3.14159 * pow(max(1.0 + g*g - 2.0*g*mu, 1e-3), 1.5));

    vec3 rayleigh = vec3(0.20, 0.45, 1.00) * Pr;
    vec3 mie      = vec3(1.00, 0.85, 0.70) * Pm * 0.3;
    vec3 col = (rayleigh + mie) * od * u_atmos.w;

    col += city * 1.5;   // 1.5: light-pollution gain (tunable)

    gl_FragColor = vec4(col, 1.0);   // additive
}
