$input a_position, a_normal, a_texcoord0
$output v_wpos

#include <bgfx_shader.sh>

uniform vec4 u_depth;   // x: far plane (logarithmic depth)

void main() {
    vec4 wpos   = mul(u_model[0], vec4(a_position, 1.0));   // shell already scaled by model
    v_wpos      = wpos.xyz;
    gl_Position = mul(u_modelViewProj, vec4(a_position, 1.0));
    gl_Position.z = (log2(max(1e-6, 1.0 + gl_Position.w)) * (2.0/log2(u_depth.x + 1.0)) - 1.0) * gl_Position.w;
}
