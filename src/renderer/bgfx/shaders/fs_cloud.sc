$input v_texcoord0, v_wpos

#include <bgfx_shader.sh>

SAMPLER2D(s_cloud, 0);         // white = cloud density
SAMPLER2D(s_night, 1);         // colour of the city lights below (hue)
SAMPLER2D(s_emiss, 2);         // white = light-source strength below
uniform vec4 u_sunDir;         // xyz: view-space direction TO the sun
uniform vec4 u_earthCenter;    // xyz: view-space sphere centre
uniform vec4 u_cloudAlpha;     // x: per-shell opacity

void main() {
    float c   = texture2D(s_cloud, v_texcoord0).x;
    vec3  N   = normalize(v_wpos - u_earthCenter.xyz);
    vec3  L   = normalize(u_sunDir.xyz);
    float ndl = dot(N, L);
    vec3  col = vec3(0.15 + 0.85 * max(ndl, 0.0));    // sun-shaded; dark on the night side

    // City lights underlight the cloud base on the night side: strength from the
    // clean emission mask, hue from the colour night map (so only real light
    // sources glow, not the airglow baked into the night map). Fades in exactly
    // where the sun is down, mirroring the earth's day/night terminator.
    float night = 1.0 - smoothstep(-0.15, 0.15, ndl);
    if (night > 0.001) {
        float e   = texture2D(s_emiss, v_texcoord0).x;   // light-source strength
        vec3  hue = texture2D(s_night, v_texcoord0).rgb; // proper colour
        col += hue * (e * night * 2.5);                  // 2.5: underlight gain (tunable)
    }
    gl_FragColor = vec4(col, c * u_cloudAlpha.x);
}
