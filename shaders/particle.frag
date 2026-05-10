#version 330 core
out vec4 FragColor;

in float v_speed;

uniform vec3  u_color;
uniform bool  u_velocity_coloring;
uniform float u_max_speed;

void main() {
    vec2 coord = gl_PointCoord - vec2(0.5);
    if (dot(coord, coord) > 0.25) discard;

    vec3 color = u_color;
    if (u_velocity_coloring) {
        float t = clamp(v_speed / u_max_speed, 0.0, 1.0);
        // low speed → u_color, high speed → red
        color = mix(u_color, vec3(1.0, 0.1, 0.05), t);
    }

    FragColor = vec4(color, 1.0);
}
