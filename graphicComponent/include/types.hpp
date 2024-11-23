#pragma once
#include <eigen3/Eigen/Eigen>

using POINT_EGDE_3D = Eigen::Vector4f;
using POINT_EGDE_2D = Eigen::Vector3f;
using RGB = Eigen::Vector3f;
using RGB_INT = Eigen::Vector3i;
namespace type_comparer {
    struct Vector3fHash {
        size_t operator()(const Eigen::Vector3f& v) const {
            size_t h1 = std::hash<float>{}(v.x());
            size_t h2 = std::hash<float>{}(v.y());
            size_t h3 = std::hash<float>{}(v.z());
            return h1 ^ (h2 << 1) ^ (h3 << 2);  // 合并哈希值
        }
    };

    // 自定义比较函数对象，用于 Eigen::Vector3f 类型
    struct Vector3fEqual {
        bool operator()(const Eigen::Vector3f& lhs, const Eigen::Vector3f& rhs) const {
            return lhs.isApprox(rhs);  // 使用 Eigen 的 isApprox 进行近似比较
        }
    };
}


namespace g_sharder {
    struct Light {
        RGB rgb;
        POINT_EGDE_3D position;
        Light(const RGB& rgb, const POINT_EGDE_3D& position) :rgb(rgb), position(position) {};
        Light(const Light& b) {
            this->rgb = b.rgb;
            this->position = b.position;
        }

        Light(Light&& b) noexcept {
            this->rgb = std::move(b.rgb);
            this->position = std::move(b.position);
        }
    };
    struct EnvPayload {
        std::vector<Light> lights;
        RGB env_light;
        float p = 10;

        EnvPayload(const std::vector<Light>& lights, const RGB& env_light, float p = 10)
            : lights(lights), env_light(env_light), p(p) {};
        EnvPayload(const EnvPayload& b) : lights(b.lights), env_light(b.env_light), p(b.p) {}

        EnvPayload(EnvPayload&& b) noexcept : lights(std::move(b.lights)), env_light(std::move(b.env_light)), p(b.p) {}
    };
    struct FragmentShaderPayload
    {
        // assume texture pixel's RGB value is under standard env, which object is exposed to (255,255,255) light at distance 1
        FragmentShaderPayload(const RGB& col, const POINT_EGDE_3D& nor,const POINT_EGDE_3D& view_pos, const POINT_EGDE_2D& tc) :
            color_reflection_coefficient(col), normal(nor),view_pos(view_pos), tex_coords(tc) {}

        FragmentShaderPayload(RGB&& col, POINT_EGDE_3D&& nor, POINT_EGDE_3D&& view_pos, POINT_EGDE_2D&& tc) :
            color_reflection_coefficient(col), normal(nor), view_pos(view_pos), tex_coords(tc) {}


        POINT_EGDE_3D view_pos;
        RGB color_reflection_coefficient;
        POINT_EGDE_3D normal;
        POINT_EGDE_2D tex_coords;
    };

    struct VertexShaderPayload
    {
        POINT_EGDE_3D position;
    };

    using FragmentShaderAlgo = std::function<Eigen::Vector3f(const FragmentShaderPayload&, const EnvPayload&)>;
    using VertexShaderAlgo = std::function<Eigen::Vector3f(const VertexShaderPayload&, const EnvPayload&)>;
}

