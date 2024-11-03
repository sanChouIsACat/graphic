//
// Created by goksu on 4/6/19.
//

#pragma once

#include <eigen3/Eigen/Eigen>
#include <optional>
#include <algorithm>
#include "Shader.hpp"
#include "algo.hpp"
#include "Triangle.hpp"

using namespace Eigen;

namespace rst
{
    enum class Buffers
    {
        Color = 1,
        Depth = 2
    };

    inline Buffers operator|(Buffers a, Buffers b)
    {
        return Buffers((int)a | (int)b);
    }

    inline Buffers operator&(Buffers a, Buffers b)
    {
        return Buffers((int)a & (int)b);
    }

    enum class Primitive
    {
        Line,
        Triangle
    };

    /*
     * For the curious : The draw function takes two buffer id's as its arguments. These two structs
     * make sure that if you mix up with their orders, the compiler won't compile it.
     * Aka : Type safety
     * */
    struct pos_buf_id
    {
        int pos_id = 0;
    };

    struct ind_buf_id
    {
        int ind_id = 0;
    };

    struct col_buf_id
    {
        int col_id = 0;
    };

    class rasterizer
    {
    public:
        template<typename U>
		rasterizer(int w, int h, U&& shader) :
            width(w), height(h), shader(std::forward<U>(shader)) {
            frame_buf.resize(w * h);
            depth_buf.resize(w * h);
        }
        pos_buf_id load_positions(const std::vector<Eigen::Vector3f>& positions);
        ind_buf_id load_indices(const std::vector<Eigen::Vector3i>& indices);
        col_buf_id load_colors(const std::vector<Eigen::Vector3f>& colors);
        col_buf_id load_normals(const std::vector<Eigen::Vector3f>& normals);

        void set_line_draw_algo(
            std::function<void(const Eigen::Vector3f&,
                const Eigen::Vector3f&,
                std::function<void(const Eigen::Vector3f&)>)> draw_algo);
        void set_model(const Eigen::Matrix4f& m);
        void set_view(const Eigen::Matrix4f& v);
        void set_projection(const Eigen::Matrix4f& p);

        
        template<typename T>
        void set_shader(T&& shader) {
            static_assert(std::is_same_v<g_sharder::Shader, std::decay_t<T>>, "using g_sharder::Shader");
            this->shader = std::forward<T>(shader); 
        }
        void set_pixel(const Vector2i& point, const Eigen::Vector3f& color);

        void clear(Buffers buff);

        void draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type);
        void draw(std::vector<Triangle*>& TriangleList);

        std::vector<Eigen::Vector3f>& frame_buffer() { return frame_buf; }

    private:
        void rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector4f, 3>& world_pos);

        // VERTEX SHADER -> MVP -> Clipping -> /.W -> VIEWPORT -> DRAWLINE/DRAWTRI -> FRAGSHADER

    private:
        Eigen::Matrix4f model;
        Eigen::Matrix4f view;
        Eigen::Matrix4f projection;
        std::function<void(const Eigen::Vector3f&,
            const Eigen::Vector3f&,
            std::function<void(const Eigen::Vector3f&)>)> draw_line;

        int normal_id = -1;

        std::map<int, std::vector<Eigen::Vector3f>> pos_buf;
        std::map<int, std::vector<Eigen::Vector3i>> ind_buf;
        std::map<int, std::vector<Eigen::Vector3f>> col_buf;
        std::map<int, std::vector<Eigen::Vector3f>> nor_buf;

        const g_sharder::Shader& shader;

        std::vector<Eigen::Vector3f> frame_buf;
        std::vector<float> depth_buf;
        int get_index(int x, int y);

        int width, height;

        int next_id = 0;
        int get_next_id() { return next_id++; }
    };
}
