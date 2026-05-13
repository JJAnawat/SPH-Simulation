#version 330 core
layout(location = 0) in vec3 a_pos;

uniform mat4 u_mat_view_proj;
uniform mat4 u_mat_model;

out VS_OUT {
    vec3 FragPos;
    vec3 Normal;
} vs_out;

void main() {
    vs_out.FragPos = vec3(u_mat_model * vec4(a_pos, 1.0));
    
    // Transform normal to world space using inverse transpose of model matrix
    // This properly handles rotation and non-uniform scaling
    vs_out.Normal = normalize(mat3(transpose(inverse(u_mat_model))) * a_pos);
    
    gl_Position = u_mat_view_proj * vec4(vs_out.FragPos, 1.0);
}
