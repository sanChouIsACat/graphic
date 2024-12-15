#version 330 core
out vec4 FragColor;

in vec2 texture_coor;
uniform sampler2D texture_sample;

void main() {
    FragColor = texture(texture_sample, vec2(texture_coor.x,1-texture_coor.y));
}
