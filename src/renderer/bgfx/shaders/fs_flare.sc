$input v_texcoord0, v_color0, v_wpos

#include <bgfx_shader.sh>

void main() {
    float r     = length(v_texcoord0);   // 0 = centre, 1 = edge
    float power = v_wpos.x;               // disk/glow falloff
    float ring  = v_wpos.y;              // >0 -> draw a ring band at this radius

    float i;
    if (ring > 0.0) {
        float d = r - ring;
        i = exp(-d * d * 70.0);          // thin halo band
    } else {
        i = pow(max(1.0 - r, 0.0), power);
    }

    // v_color0.a carries the per-element intensity; additive output.
    gl_FragColor = vec4(v_color0.rgb * (i * v_color0.a), 1.0);
}
