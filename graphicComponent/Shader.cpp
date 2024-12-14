#include "Shader.hpp"
#include "types.hpp"
g_sharder::Shader &
g_sharder::Shader::set_vertex_shader_algo(VertexShaderAlgo algo) {
  this->vertex_shader_algo = algo;
  return *this;
}

g_sharder::Shader &
g_sharder::Shader::set_fragment_shader_algo(FragmentShaderAlgo algo) {
  this->fragment_shader_algo = algo;
  return *this;
}

Eigen::Vector3f
g_sharder::Shader::shade_pixel(const FragmentShaderPayload &payload) const {
  return fragment_shader_algo(payload, env_payload);
}
