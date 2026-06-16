$input v_texcoord0, v_wpos, v_objpos

#include <bgfx_shader.sh>

SAMPLER2D(s_color, 0);   // daytime albedo
SAMPLER2D(s_bump,  1);   // height / relief
SAMPLER2D(s_night, 2);   // city lights
SAMPLER2D(s_rough, 3);   // roughness (low over water -> glossy ocean glint)
SAMPLER2D(s_cloud, 4);   // cloud cover (for cloud shadows on the ground)
uniform vec4 u_sunDir;       // xyz: view-space direction TO the sun
uniform vec4 u_earthCenter;  // xyz: view-space sphere centre (km)
uniform vec4 u_camPos;       // xyz: view-space camera (km)

// --- cheap 3D gradient noise + fBm, for procedural surface-detail amplification.
// The base maps top out at ~1.2 km/texel, so anything finer than that is invented
// here (keyed off the real height map) and only shown up close.
vec3 hash33(vec3 p) {
    p = vec3(dot(p, vec3(127.1, 311.7, 74.7)),
             dot(p, vec3(269.5, 183.3, 246.1)),
             dot(p, vec3(113.5, 271.9, 124.6)));
    return fract(sin(p) * 43758.5453123) * 2.0 - 1.0;
}
// Periodic gradient noise: wrapping the integer lattice with mod(period) keeps the
// hash inputs small, so float precision holds even though the domain is a body-fixed
// planet coordinate of ~1e7 magnitude. That body-fixed (frame-invariant) domain is
// what stops the detail from swimming as the camera and Earth move; the price is
// repetition every `period` cells, set large enough not to be noticeable.
float gnoise(vec3 p, float period) {
    vec3 i = floor(p), f = fract(p);
    vec3 u = f*f*(3.0 - 2.0*f);
    vec3 a = mod(i,        period);   // cell corners, wrapped to [0,period)
    vec3 b = mod(i + 1.0,  period);
    return mix(mix(mix(dot(hash33(vec3(a.x,a.y,a.z)), f-vec3(0,0,0)),
                       dot(hash33(vec3(b.x,a.y,a.z)), f-vec3(1,0,0)), u.x),
                   mix(dot(hash33(vec3(a.x,b.y,a.z)), f-vec3(0,1,0)),
                       dot(hash33(vec3(b.x,b.y,a.z)), f-vec3(1,1,0)), u.x), u.y),
               mix(mix(dot(hash33(vec3(a.x,a.y,b.z)), f-vec3(0,0,1)),
                       dot(hash33(vec3(b.x,a.y,b.z)), f-vec3(1,0,1)), u.x),
                   mix(dot(hash33(vec3(a.x,b.y,b.z)), f-vec3(0,1,1)),
                       dot(hash33(vec3(b.x,b.y,b.z)), f-vec3(1,1,1)), u.x), u.y), u.z);
}
float fbm3(vec3 p) {
    float f = 0.0, a = 0.5, period = 512.0;
    for (int k = 0; k < 4; k++) { f += a * gnoise(p, period); p *= 2.0; a *= 0.5; period *= 2.0; }
    return f;
}

// Smooth (B-spline) bicubic upsample in 4 bilinear taps. Magnifying the single
// global map with hardware bilinear shows hard texel cells; this rounds them off
// into smooth gradients so the ground stops looking blocky up close.
vec4 cubicW(float v) {
    vec4 n = vec4(1.0, 2.0, 3.0, 4.0) - v;
    vec4 s = n * n * n;
    float x = s.x;
    float y = s.y - 4.0 * s.x;
    float z = s.z - 4.0 * s.y + 6.0 * s.x;
    float w = 6.0 - x - y - z;
    return vec4(x, y, z, w) * (1.0 / 6.0);
}
vec3 texBicubic(sampler2D tex, vec2 uv, vec2 texSize) {
    vec2 invSize = 1.0 / texSize;
    uv = uv * texSize - 0.5;
    vec2 f = fract(uv);
    uv -= f;
    vec4 xc = cubicW(f.x), yc = cubicW(f.y);
    vec4 c  = uv.xxyy + vec4(-0.5, 1.5, -0.5, 1.5);
    vec4 s  = vec4(xc.x + xc.y, xc.z + xc.w, yc.x + yc.y, yc.z + yc.w);
    vec4 o  = c + vec4(xc.y, xc.w, yc.y, yc.w) / s;
    o *= invSize.xxyy;
    vec3 s0 = texture2D(tex, o.xz).xyz;
    vec3 s1 = texture2D(tex, o.yz).xyz;
    vec3 s2 = texture2D(tex, o.xw).xyz;
    vec3 s3 = texture2D(tex, o.yw).xyz;
    float sx = s.x / (s.x + s.y);
    float sy = s.z / (s.z + s.w);
    return mix(mix(s3, s2, sx), mix(s1, s0, sx), sy);
}

void main() {
    vec3 N = normalize(v_wpos - u_earthCenter.xyz);
    vec3 L = normalize(u_sunDir.xyz);
    vec3 V = normalize(u_camPos.xyz - v_wpos);

    // Bump relief: perturb N from the height-map gradient. The renderer rotates
    // ECI +Z (north) to view +Y, so the pole axis is +Y; build an east/north
    // tangent frame around N to apply the gradient. The step spans several texels
    // (one texel is ~1km, gradients there are tiny) and the strength is large
    // because the map is global-scale elevation; both are tuned for visible relief.
    vec2  texel = vec2(1.0/32768.0, 1.0/16384.0) * 4.0;
    float hL = texture2D(s_bump, v_texcoord0 - vec2(texel.x, 0.0)).x;
    float hR = texture2D(s_bump, v_texcoord0 + vec2(texel.x, 0.0)).x;
    float hD = texture2D(s_bump, v_texcoord0 - vec2(0.0, texel.y)).x;
    float hU = texture2D(s_bump, v_texcoord0 + vec2(0.0, texel.y)).x;
    vec3  east  = normalize(cross(vec3(0.0, 1.0, 0.0), N));
    vec3  north = cross(N, east);
    vec3  Np    = normalize(N + (east*(hL - hR) + north*(hD - hU)) * 40.0);

    vec3  albedo = texture2D(s_color, v_texcoord0).xyz;

    // Procedural detail amplification: close to the surface the maps are far too
    // coarse (~1.2 km/texel, ~20 km/quad), so (1) de-block the magnified base map
    // with a smooth bicubic upsample and (2) synthesise high-frequency relief and
    // ground texture that isn't in the data. All faded in by camera proximity so
    // orbit views never shimmer. This perturbs the *shading* normal and albedo
    // only -- the geometry stays smooth (real geometry LOD is a later phase).
    float distSurf = length(u_camPos.xyz - v_wpos);          // km to this point
    float detail   = smoothstep(200.0, 5.0, distSurf);       // 0 far .. 1 near
    if (detail > 0.001) {
        // Smooth out the blocky magnified texture (blend in only as we approach).
        albedo = mix(albedo, texBicubic(s_color, v_texcoord0, vec2(32768.0, 16384.0)), detail);

        float hC   = texture2D(s_bump, v_texcoord0).x;
        float land = smoothstep(0.0008, 0.02, hC);           // oceans stay smooth
        float amp  = detail * land;
        if (amp > 0.001) {
            // Noise domain is the body-fixed (object-space) position, so the pattern
            // is locked to the planet and does not swim as the camera/Earth move. The
            // periodic gnoise keeps precision at this large magnitude. Gradient is
            // taken along object-space tangents and applied to the view-space normal
            // (the two east/north frames describe the same physical directions).
            vec3  Nobj  = normalize(v_objpos);
            vec3  oEast = normalize(cross(vec3(0.0, 0.0, 1.0), Nobj));  // body +Z = north pole
            vec3  oNorth= cross(Nobj, oEast);
            vec3  P     = v_objpos * 0.0033;                  // ~0.3 km base features
            float e     = 0.4;                                // finite-difference step
            float macro = fbm3(P * 0.15);                     // ~2 km patches
            float n0    = fbm3(P);
            float nf    = fbm3(P * 4.7);                      // fine grain
            float se    = fbm3(P + oEast  * e) - n0;
            float sn    = fbm3(P + oNorth * e) - n0;
            // Fake fine relief by tilting the shading normal along the noise gradient.
            Np = normalize(Np - (east*se + north*sn) * 2.2 * amp);
            // Multi-scale brightness break-up (keeps local hue) so the ground reads as
            // textured terrain across scales instead of one muddy colour.
            float v = macro * 0.5 + n0 * 0.35 + nf * 0.15;
            albedo  = max(albedo * (1.0 + v * 0.38 * amp), vec3(0.0));
        }
    }

    float lit    = max(dot(Np, L), 0.0);                 // relief shading
    float term   = dot(N, L);                            // smooth terminator
    float t      = smoothstep(-0.15, 0.15, term);        // 0 = night .. 1 = day

    vec3  day = albedo * (0.15 + 0.85*lit);

    // Cloud shadows: cast a ray from this ground point toward the sun; where it
    // crosses the cloud layer and there's cover, darken the lit ground. Sampling
    // at the cloud altitude gives the correct offset (longer shadows at low sun),
    // so the shadow falls beside the cloud instead of hiding under it.
    vec3  ocC = v_wpos - u_earthCenter.xyz;
    float Rc  = length(ocC) + 7.0;                          // ~7 km: mid-troposphere cloud deck
    float bC  = dot(ocC, L);                                // (km, matching the rendered shells)
    float tc  = -bC + sqrt(max(bC*bC - (dot(ocC, ocC) - Rc*Rc), 0.0));
    vec3  cp  = normalize((v_wpos + L*tc) - u_earthCenter.xyz);
    vec3  ce  = vec3(cp.x, -cp.z, cp.y);                     // view -> ECI
    vec2  cuv = vec2(atan(ce.y, ce.x) * (0.5/3.14159265) + 0.5,
                     acos(clamp(ce.z, -1.0, 1.0)) / 3.14159265);
    // Match the cloud's apparent opacity: it is drawn as 5 shells at alpha 0.22
    // (keep this in sync with shellAlpha in bgfx_backend.cpp), so its coverage
    // follows 1-(1-0.22*c)^5, which lifts faint texels into visible cloud and makes
    // the shadow cover the cloud's full footprint (every non-black texel casts it).
    // The gate only fades the shadow across the terminator into the night side; it
    // is kept tight (0..0.05) so low-sun shadows -- which stretch out to the side
    // and are the ones actually visible from orbit -- are not cut off.
    float c       = clamp(texture2D(s_cloud, cuv).x, 0.0, 1.0);
    float cloudSh = 1.0 - pow(1.0 - 0.22 * c, 5.0);
    day *= 1.0 - 0.85 * cloudSh * smoothstep(0.0, 0.05, term);

    // PBR specular (Cook-Torrance GGX): the roughness map makes oceans glossy
    // (sharp sun glint) and land matte. F0 ~ 0.02 (dielectric water).
    float rough = clamp(1.0 - texture2D(s_rough, v_texcoord0).x, 0.2, 1.0);
    vec3  H   = normalize(L + V);
    float NdH = max(dot(Np, H), 0.0);
    float NdV = max(dot(Np, V), 1e-4);
    float NdL = max(dot(Np, L), 0.0);
    float VdH = max(dot(V, H), 0.0);
    float a = rough * rough, a2 = a * a;
    float Dg = a2 / (3.14159265 * pow(NdH*NdH*(a2 - 1.0) + 1.0, 2.0));
    float kk = a * 0.5;
    float Gg = (NdV/(NdV*(1.0 - kk) + kk)) * (NdL/(NdL*(1.0 - kk) + kk));
    float Fr = 0.02 + 0.98 * pow(1.0 - VdH, 5.0);
    float spec = Dg * Gg * Fr / (4.0 * NdV * NdL + 1e-4);
    day += vec3(1.0, 0.97, 0.90) * (spec * NdL * 2.2);   // ocean sun glint

    float rim = pow(1.0 - max(dot(N, V), 0.0), 3.0);     // atmospheric limb glow
    day += vec3(0.30, 0.50, 0.95) * rim * (0.35 + 0.65*max(term, 0.0));

    // City lights on the dark side, at lower exposure: squaring crushes the dim
    // ocean/atmosphere baked into the map (which otherwise out-shines the dark
    // daytime ocean) while keeping the bright city lights.
    vec3 night = texture2D(s_night, v_texcoord0).xyz;
    night = night * night * 1.3;
    vec3 color = mix(night, day, t);                     // blend across the twilight band

    gl_FragColor = vec4(color, 1.0);
}
