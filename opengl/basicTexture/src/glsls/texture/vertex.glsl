#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 2) in vec3 aTexture;
out vec2 texture_coor;
void main() {
    gl_Position = vec4(aPos, 1.0);
    texture_coor = vec2(aTexture.xy);
}