$input a_position, a_normal, a_texcoord0
$output v_texcoord0, v_wpos

#include <bgfx_shader.sh>

SAMPLER2D(s_bump, 1);          // height map, also sampled here for displacement
uniform vec4 u_dispScale;      // x: vertex displacement scale (metres)

void main() {
    // Actual geometry displacement: push the vertex out along its (radial)
    // normal by the sampled height. Needs an explicit LOD in the vertex stage.
    float h = texture2DLod(s_bump, a_texcoord0, 0.0).x;
    vec3  disp = a_position + a_normal * (h * u_dispScale.x);

    vec4 wpos   = mul(u_model[0], vec4(disp, 1.0));
    v_wpos      = wpos.xyz;                              // view-space (km)
    gl_Position = mul(u_modelViewProj, vec4(disp, 1.0));
    v_texcoord0 = a_texcoord0;
}
