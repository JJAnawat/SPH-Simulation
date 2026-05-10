#version 330 core
layout(location = 0) in vec3 aPos;

uniform mat4 u_mat_view_proj;

void main() {
    gl_Position = u_mat_view_proj * vec4(aPos, 1.0);
}
