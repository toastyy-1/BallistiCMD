$input v_texcoord0, v_wpos

#include <bgfx_shader.sh>

SAMPLER2D(s_cloud, 0);         // white = cloud density
uniform vec4 u_sunDir;         // xyz: view-space direction TO the sun
uniform vec4 u_earthCenter;    // xyz: view-space sphere centre
uniform vec4 u_cloudAlpha;     // x: per-shell opacity

void main() {
    float c = texture2D(s_cloud, v_texcoord0).x;
    vec3  N = normalize(v_wpos - u_earthCenter.xyz);
    vec3  L = normalize(u_sunDir.xyz);
    float lit = 0.15 + 0.85 * max(dot(N, L), 0.0);   // sun-shaded; dark on the night side
    gl_FragColor = vec4(vec3(lit), c * u_cloudAlpha.x);
}
