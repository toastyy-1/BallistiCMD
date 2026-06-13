$input v_texcoord0, v_wpos

#include <bgfx_shader.sh>

SAMPLER2D(s_color, 0);   // daytime albedo
SAMPLER2D(s_bump,  1);   // height / relief
SAMPLER2D(s_night, 2);   // city lights
SAMPLER2D(s_rough, 3);   // roughness (low over water -> glossy ocean glint)
uniform vec4 u_sunDir;       // xyz: view-space direction TO the sun
uniform vec4 u_earthCenter;  // xyz: view-space sphere centre (km)
uniform vec4 u_camPos;       // xyz: view-space camera (km)

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
    float lit    = max(dot(Np, L), 0.0);                 // relief shading
    float term   = dot(N, L);                            // smooth terminator
    float t      = smoothstep(-0.15, 0.15, term);        // 0 = night .. 1 = day

    vec3  day = albedo * (0.15 + 0.85*lit);

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
