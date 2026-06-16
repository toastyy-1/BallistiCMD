$input a_position, a_normal, a_texcoord0, a_color0
$output v_texcoord0, v_color0, v_normal, v_wpos, v_objpos

#include <bgfx_shader.sh>

uniform vec4 u_depth;   // x: far plane, for logarithmic depth

void main() {
    vec4 wp = mul(u_model[0], vec4(a_position, 1.0));
    v_wpos  = wp.xyz;                                  // view-space (km)
    gl_Position = mul(u_modelViewProj, vec4(a_position, 1.0));
    gl_Position.z = (log2(max(1e-6, 1.0 + gl_Position.w)) * (2.0/log2(u_depth.x + 1.0)) - 1.0) * gl_Position.w;
    v_texcoord0 = a_texcoord0;                         // x: metallic, y: roughness
    v_color0    = a_color0;                            // albedo
    v_normal    = mul(u_model[0], vec4(a_normal, 0.0)).xyz;
    v_objpos    = a_position;                          // body space (metres), for grime
}
