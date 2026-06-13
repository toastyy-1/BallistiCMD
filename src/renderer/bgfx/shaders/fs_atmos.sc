$input v_wpos

#include <bgfx_shader.sh>

uniform vec4 u_sunDir;       // xyz: view-space direction TO the sun
uniform vec4 u_earthCenter;  // xyz: view-space sphere centre (km)
uniform vec4 u_camPos;       // xyz: view-space camera (km)
uniform vec4 u_atmos;        // x: surface radius, z: outward scale height, w: inward scale height (km)

void main() {
    // Impact parameter: closest distance from the planet centre to this fragment's
    // view ray. Glow is densest just above the surface (b ~ R) and falls off
    // exponentially outward into space, with a gentler inward falloff for the
    // thin haze seen over the disk near the limb.
    vec3  cam = u_camPos.xyz;
    vec3  D   = normalize(v_wpos - cam);
    float b   = length(cross(D, u_earthCenter.xyz - cam));   // km

    float R = u_atmos.x, Hout = u_atmos.z, Hin = u_atmos.w;
    float d = b - R;
    float glow = (d >= 0.0) ? exp(-d / Hout) : exp(d / Hin);

    // Lit on the day side, fading through the terminator into night.
    vec3  N   = normalize(v_wpos - u_earthCenter.xyz);
    float day = smoothstep(-0.25, 0.25, dot(N, normalize(u_sunDir.xyz)));
    float intensity = glow * (0.12 + 0.88 * day);

    vec3 col = vec3(0.33, 0.55, 1.0);                // Rayleigh-ish blue
    gl_FragColor = vec4(col * intensity, intensity); // additive
}
