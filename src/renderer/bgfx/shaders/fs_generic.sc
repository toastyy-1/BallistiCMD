$input v_texcoord0, v_color0

#include <bgfx_shader.sh>

SAMPLER2D(s_tex, 0);
uniform vec4 u_tint;

void main() {
    // Untextured draws bind a 1x1 white texture, so this reduces to tint*vcolor.
    gl_FragColor = texture2D(s_tex, v_texcoord0) * u_tint * v_color0;
}
