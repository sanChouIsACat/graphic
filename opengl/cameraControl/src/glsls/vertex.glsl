#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 2) in vec3 aTexture;
uniform mat4 perspective;
uniform mat4 view;
out vec2 texture_coor;
void main() {
  // vec4 modeled_coord = vec4(aPos, 1.0);
  vec4 mid = perspective * view * vec4(aPos, 1.0);
  // mid = mid / mid.z;
  gl_Position = mid;
  texture_coor = vec2(aTexture.xy);
}