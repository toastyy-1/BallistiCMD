$input v_texcoord0, v_color0, v_normal

#include <bgfx_shader.sh>

SAMPLER2D(s_tex, 0);
uniform vec4 u_tint;
uniform vec4 u_light;   // xyz: sun direction (view); w: lit flag (0 = flat/unlit)

void main() {
    // Untextured draws bind a 1x1 white texture, so this reduces to tint*vcolor.
    vec4 base = texture2D(s_tex, v_texcoord0) * u_tint * v_color0;

    if (u_light.w > 0.5) {
        vec3  N = normalize(v_normal);
        float d = max(dot(N, normalize(u_light.xyz)), 0.0);
        base.rgb *= (0.28 + 0.72 * d);   // ambient + lambert (sun-lit surfaces)
    }

    gl_FragColor = base;
}
