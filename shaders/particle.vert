#version 330 core
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_vel;

uniform mat4  u_mat_view_proj;
uniform float u_point_size;

out float v_speed;

void main() {
    gl_Position  = u_mat_view_proj * vec4(a_pos, 1.0);
    gl_PointSize = u_point_size;
    v_speed       = length(a_vel);
}
