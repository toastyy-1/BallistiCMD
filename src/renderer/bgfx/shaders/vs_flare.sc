$input a_position, a_normal, a_texcoord0, a_color0
$output v_texcoord0, v_color0, v_wpos

#include <bgfx_shader.sh>

// Screen-space sprite: positions arrive in NDC; texcoord is the local quad coord
// (-1..1) for the radial shape; normal carries (falloff power, ring radius).
void main() {
    gl_Position = vec4(a_position.xy, 0.0, 1.0);
    v_texcoord0 = a_texcoord0;
    v_color0    = a_color0;
    v_wpos      = a_normal;
}
