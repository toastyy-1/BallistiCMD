$input a_position
$output v_texcoord0

#include <bgfx_shader.sh>

// Full-screen triangle: positions arrive already in clip space (-1..3); pass the
// NDC xy to the fragment shader for per-pixel view-ray reconstruction.
void main() {
    gl_Position = vec4(a_position.xy, 0.0, 1.0);
    v_texcoord0 = a_position.xy;
}
