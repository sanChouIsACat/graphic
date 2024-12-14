#pragma once
#include "Shader.hpp"
#include <eigen3/Eigen/Eigen>
namespace g_sharder {
Eigen::Vector3f phone_sharder_algo(const FragmentShaderPayload &payload,
                                   const EnvPayload &env);
Eigen::Vector3f normal_fragment_shader(const FragmentShaderPayload &payload);
} // namespace g_sharder