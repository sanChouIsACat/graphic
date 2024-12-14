#pragma once
#include <eigen3/Eigen/Eigen>
using POINT_EGDE_3D = Eigen::Vector4f;
using POINT_EGDE_2D = Eigen::Vector3f;
using RGB = Eigen::Vector3f;
using RGB_INT = Eigen::Vector3i;
namespace GTypes {
    // old code use global type defs
    using POINT_EGDE_3D = POINT_EGDE_3D;
    using POINT_EGDE_2D = POINT_EGDE_2D;
    using RGB = RGB;
    using RGB_INT = RGB_INT;
    struct Plane
    {
        POINT_EGDE_3D basis1;
        POINT_EGDE_3D basis2;
        POINT_EGDE_3D origin;
    };
    // from 1 to 4, rotate by couterclockwise
    struct AABB
    {
        POINT_EGDE_3D top_1;
        POINT_EGDE_3D top_2;
        POINT_EGDE_3D top_3;
        POINT_EGDE_3D top_4;
        POINT_EGDE_3D bottom_1;
        POINT_EGDE_3D bottom_2;
        POINT_EGDE_3D bottom_3;
        POINT_EGDE_3D bottom_4;
        AABB() = default;
        // Constructor to initialize AABB
        AABB(const POINT_EGDE_3D& minPoint, const POINT_EGDE_3D& maxPoint) {
            // Bottom face
            bottom_1 = POINT_EGDE_3D(minPoint.x(), minPoint.y(), minPoint.z(), 1); // (minX, minY, minZ)
            bottom_2 = POINT_EGDE_3D(maxPoint.x(), minPoint.y(), minPoint.z(), 1); // (maxX, minY, minZ)
            bottom_3 = POINT_EGDE_3D(maxPoint.x(), maxPoint.y(), minPoint.z(), 1); // (maxX, maxY, minZ)
            bottom_4 = POINT_EGDE_3D(minPoint.x(), maxPoint.y(), minPoint.z(), 1); // (minX, maxY, minZ)

            // Top face
            top_1 = POINT_EGDE_3D(minPoint.x(), minPoint.y(), maxPoint.z(), 1);    // (minX, minY, maxZ)
            top_2 = POINT_EGDE_3D(maxPoint.x(), minPoint.y(), maxPoint.z(), 1);    // (maxX, minY, maxZ)
            top_3 = POINT_EGDE_3D(maxPoint.x(), maxPoint.y(), maxPoint.z(), 1);    // (maxX, maxY, maxZ)
            top_4 = POINT_EGDE_3D(minPoint.x(), maxPoint.y(), maxPoint.z(), 1);    // (minX, maxY, maxZ)
        }
        AABB(const POINT_EGDE_3D& t1, const POINT_EGDE_3D& t2, const POINT_EGDE_3D& t3, const POINT_EGDE_3D& t4,
            const POINT_EGDE_3D& b1, const POINT_EGDE_3D& b2, const POINT_EGDE_3D& b3, const POINT_EGDE_3D& b4)
            : top_1(t1), top_2(t2), top_3(t3), top_4(t4),
            bottom_1(b1), bottom_2(b2), bottom_3(b3), bottom_4(b4) {}
    };
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
    struct Light {
        RGB rgb;
        POINT_EGDE_3D position;
        POINT_EGDE_3D dir;
        Light(const RGB& rgb,
            const POINT_EGDE_3D& position,
            const POINT_EGDE_3D& dir) :rgb(rgb), position(position), dir(dir) {};
        Light(const Light& b) {
            this->rgb = b.rgb;
            this->position = b.position;
            this->dir = dir;
        }

        Light(Light&& b) noexcept {
            this->rgb = std::move(b.rgb);
            this->position = std::move(b.position);
            this->dir = std::move(b.dir);
        }
    };
}



namespace g_sharder {
    
    struct EnvPayload {
        std::vector<GTypes::Light> lights;
        RGB env_light;
        float p = 10;

        EnvPayload(const std::vector<GTypes::Light>& lights, const RGB& env_light, float p = 10)
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

