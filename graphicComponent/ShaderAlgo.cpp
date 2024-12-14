#include "ShaderAlgo.hpp"
#include "algebra.hpp"
#include "algorithm"
using namespace Eigen;
Eigen::Vector3f
g_sharder::phone_sharder_algo(const FragmentShaderPayload &payload,
                              const EnvPayload &env) {
  RGB res = Vector3f::Zero();
  POINT_EGDE_3D normed_normal = payload.normal.normalized();

  for (const auto &light : env.lights) {
    float n = 0.0f;
    POINT_EGDE_3D light_vec = light.position - payload.view_pos;
    // add one to avoid infinite value at nearest poztion(let light point be
    // distance 1)
    float distance = 1 + light_vec.norm();
    POINT_EGDE_3D view_vec = payload.view_pos;
    view_vec.w() = 0;
    POINT_EGDE_3D half_vec = light_vec + view_vec;
    light_vec.normalize();
    half_vec.normalize();

    // diffuse reflection
    n += normed_normal.dot(light_vec);

    // specular reflection
    n += std::pow(half_vec.dot(normed_normal), env.p);
    if (n < 0) {
      continue;
    }

    res += light.rgb * n / (distance * 0.05);
  }
  res += env.env_light;
  res += payload.color_reflection_coefficient;
  return res;
}

// directly copied from homework3
Eigen::Vector3f
g_sharder::normal_fragment_shader(const FragmentShaderPayload &payload) {
  Eigen::Vector3f return_color = (payload.normal.head<3>().normalized() +
                                  Eigen::Vector3f(1.0f, 1.0f, 1.0f)) /
                                 2.f;
  Eigen::Vector3f result;
  result << return_color.x() * 255, return_color.y() * 255,
      return_color.z() * 255;
  return result;
}
