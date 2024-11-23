//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include "rasterizer.hpp"
#include "interpolation.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>
#include "logger.hpp"
#include "algebra.hpp"
#include "types.hpp"
#include <intrin.h> 

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_normals(const std::vector<Eigen::Vector3f>& normals)
{
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(int x, int y, const Vector4f* _v){
    Vector3f v[3];
    for(int i=0;i<3;i++)
        v[i] = {_v[i].x(),_v[i].y(), 1.0};
    Vector3f f0,f1,f2;
    f0 = v[1].cross(v[0]);
    f1 = v[2].cross(v[1]);
    f2 = v[0].cross(v[2]);
    Vector3f p(x,y,1.);
    if((p.dot(f0)*f0.dot(v[2])>0) && (p.dot(f1)*f1.dot(v[0])>0) && (p.dot(f2)*f2.dot(v[1])>0))
        return true;
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v){
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(std::vector<Triangle *> &TriangleList) {

    const Eigen::Matrix4f mv = view * model;
    const Eigen::Matrix4f mvp = projection * mv;
    const Eigen::Matrix4f inv_trans = (mv).inverse().transpose();
    for (const auto& t:TriangleList)
    {
        Triangle newtri = *t;

        //std::array<Eigen::Vector4f, 3> mm {
        //        (view * model * t->v[0]),
        //        (view * model * t->v[1]),
        //        (view * model * t->v[2])
        //};

        std::array<POINT_EGDE_3D, 3> viewspace_pos = {
            mv * t->v[0],
            mv * t->v[1],
            mv * t->v[2]
        };

        //std::transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) {
        //    return v.template head<3>();
        //});

        Eigen::Vector4f v[] = {
                mvp * t->v[0],
                mvp * t->v[1],
                mvp * t->v[2]
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec.x()/=vec.w();
            vec.y()/=vec.w();
            vec.z()/=vec.w();
            vec.w() = 1;
        }

        POINT_EGDE_3D n[] = {
                inv_trans * t->normal[0],
                inv_trans * t->normal[1],
                inv_trans * t->normal[2]
        };

        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            //vert.z() = vert.z() * f1 + f2;
        }
        

        for (int i = 0; i < 3; ++i)
        {
            //screen space coordinates
            newtri.setVertex(i, v[i]);
        }

        for (int i = 0; i < 3; ++i)
        {
            //view space normal
            newtri.setNormal(i, n[i]);
        }

        newtri.setColor(0, 148,121.0,92.0);
        newtri.setColor(1, 148,121.0,92.0);
        newtri.setColor(2, 148,121.0,92.0);
        
        // Also pass view space vertice position
        rasterize_triangle(newtri, viewspace_pos);
    }
}

static Eigen::Vector3f interpolate(float alpha, float beta, float gamma, const Eigen::Vector3f& vert1, const Eigen::Vector3f& vert2, const Eigen::Vector3f& vert3, float weight)
{
    return (alpha * vert1 + beta * vert2 + gamma * vert3) / weight;
}

static Eigen::Vector2f interpolate(float alpha, float beta, float gamma, const Eigen::Vector2f& vert1, const Eigen::Vector2f& vert2, const Eigen::Vector2f& vert3, float weight)
{
    auto u = (alpha * vert1[0] + beta * vert2[0] + gamma * vert3[0]);
    auto v = (alpha * vert1[1] + beta * vert2[1] + gamma * vert3[1]);

    u /= weight;
    v /= weight;

    return Eigen::Vector2f(u, v);
}

//Screen space rasterization

void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector4f, 3>& view_pos) 
{
    auto [min_x, min_y, max_x, max_y] = GAlgo::getRoundingBox(t);
    for (int i = min_x; i <= std::ceil(max_x); i++)
    {
        for (int j = min_y; j < std::ceil(max_y); j++)
        {
            if (!interpolation::insideTriangle((float)i, (float)j, t.v)) {
                continue;
            }
            std::array<float,3> depths;
            for (int i = 0; i < 3; i++) {
				depths[i] = t.v[i].z();
            }
            //if (i == 330 && j == 405) {
            //    __debugbreak();
            //}
            auto [a, b, c, depth, view_coor_normal, view_coor_point, text_coors] = 
                GAlgo::BarycentricProperties(i, j, t.v, depths, t.normal, view_pos, t.tex_coords);
            if (depth <= depth_buf[i * width + j]) {
                continue;
            }
            depth_buf[i * width + j] = depth;
            
            RGB pixel = t.tex->getColor(text_coors[0], text_coors[1]);
            //RGB pixel{ 100,100,100 };
			g_sharder::FragmentShaderPayload shader_payload(pixel, view_coor_normal, view_coor_point, text_coors);
			RGB color = shader.shade_pixel(shader_payload);
            //G_LOGGER_INFO("x[%d] y[%d] ORIGIN RGB[%s]", i, j, EigenStructToString(pixel.transpose()).c_str());
            //G_LOGGER_INFO("x[%d] y[%d] RGB[%s]", i, j, EigenStructToString(color.transpose()).c_str());
			set_pixel(Vector2i(i, j), color);
        }
    }
}

//void rst::rasterizer::rasterize_triangle(const Triangle& t, const std::array<Eigen::Vector4f, 3>& view_pos) {
//    auto v = t.toVector4();
//    float min_x = std::numeric_limits<float>::max();
//    float min_y = std::numeric_limits<float>::max();
//    float max_x = std::numeric_limits<float>::min();
//    float max_y = std::numeric_limits<float>::min();
//
//    for (const Vector4f& edge : v) {
//
//        float x = edge.x();
//        float y = edge.y();
//        min_x = min_x < x ? min_x : x;
//        min_y = min_y < y ? min_y : y;
//        max_x = max_x > x ? max_x : x;
//        max_y = max_y > y ? max_y : y;
//    }
//
//    for (int i = min_x; i < (int)max_x; i++)
//    {
//        for (int j = min_y; j < max_y; j++)
//        {
//            if (interpolation::insideTriangle(i, j, v)) {
//                auto [alpha, beta, gamma] = computeBarycentric2D(i, j, t.v);
//                //float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
//                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
//                //z_interpolated *= w_reciprocal;
//                int idx = width * i + j;
//
//                if (depth_buf[idx] < z_interpolated) {
//                    depth_buf[idx] = z_interpolated;
//                    set_pixel(Vector2i(i, j), Vector3f{255,255,255});
//                }
//            }
//        }
//    }
//
//    // TODO : Find out the bounding box of current triangle.
//    // iterate through the pixel and find if the current pixel is inside the triangle
//
//    // If so, use the following code to get the interpolated z value.
//
//
//    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
//}

void rst::rasterizer::set_line_draw_algo(std::function<void(const Eigen::Vector3f&, const Eigen::Vector3f&, std::function<void(const Eigen::Vector3f&)>)> draw_algo)
{
    this->draw_line = draw_algo;
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), - std::numeric_limits<float>::infinity());
    }
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-y)*width + x;
}

void rst::rasterizer::set_pixel(const Vector2i &point, const Eigen::Vector3f &color)
{
    //old index: auto ind = point.y() + point.x() * width;
    int ind = (height-point.y())*width + (width - point.x());
    frame_buf[ind] = color;
}

