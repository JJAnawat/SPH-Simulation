#version 330 core
out vec4 FragColor;

in VS_OUT {
    vec3 FragPos;
    vec3 Normal;
} fs_in;

uniform vec3 u_color;
uniform vec3 u_light_pos;

void main() {
    // Simple Phong lighting
    vec3 norm = normalize(fs_in.Normal);
    vec3 lightDir = normalize(u_light_pos - fs_in.FragPos);
    vec3 viewDir = normalize(-fs_in.FragPos);
    
    // Ambient
    float ambientStrength = 0.3;
    vec3 ambient = ambientStrength * u_color;
    
    // Diffuse
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * u_color;
    
    // Specular
    float specularStrength = 0.5;
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32.0);
    vec3 specular = specularStrength * spec * vec3(1.0);
    
    vec3 result = ambient + diffuse + specular;
    FragColor = vec4(result, 1.0);
}
