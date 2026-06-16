$input v_normal

#include <bgfx_shader.sh>

uniform vec4 u_heat;   // xyz: travel direction (view); w: aerodynamic heating [0,1]

void main() {
    float h    = u_heat.w;
    float wind = max(dot(normalize(v_normal), normalize(u_heat.xyz)), 0.0);   // windward
    float glow = pow(wind, 2.0) * h;

    // Blackbody-ish ramp: dull red -> orange -> white as heating rises.
    vec3 hot = mix(vec3(0.85, 0.10, 0.0), vec3(1.0, 0.5, 0.12), smoothstep(0.0, 0.45, h));
    hot = mix(hot, vec3(1.0, 0.95, 0.85), smoothstep(0.5, 1.0, h));

    gl_FragColor = vec4(hot * glow, 2.0);   // additive
}
