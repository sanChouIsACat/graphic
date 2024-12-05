#include <iostream>
#include <opencv2/opencv.hpp>

#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"
#include "ShaderAlgo.hpp"
#include "view.hpp"
#include "transformation.hpp"

using namespace Eigen;
constexpr double PI = 3.141592653589793238462643383279502884197;
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float angle)
{
    return transformation::generateArbitraryRotateMatrix(POINT_EGDE_3D{ 0,1,0,0 }, POINT_EGDE_3D{ 0,0,0,1 }, angle);
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
    float zNear, float zFar)
{
    float half_len = std::tanf(eye_fov) * zNear;
    Vector4f down_left = Vector4f(half_len, half_len, zNear, 1);
    Vector4f top_right = Vector4f(-half_len, -half_len, zNear, 1);

    return view::generatePerspectiveProjection(down_left, top_right, zFar);
}

static Eigen::Vector3f reflect(const Eigen::Vector3f& vec, const Eigen::Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return (2 * costheta * axis - vec).normalized();
}


int main(int argc, const char** argv)
{
    std::vector<Triangle*> TriangleList;
    float angle = (float)35 / 360 * 2 * PI;
    bool command_line = false;

    std::string filename = "output.png";
    objl::Loader Loader;
    std::string obj_path = "./models/spot/";

    // load texture 
    auto texture_path = "spot_texture.png";
    Texture texture(obj_path + texture_path);
    // Load .obj File
    bool loadout = Loader.LoadFile(obj_path + "spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j, POINT_EGDE_3D(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,0 - mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j, POINT_EGDE_3D(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,0 - mesh.Vertices[i+j].Normal.Z, 1.0));
                t->setTexCoord(j, POINT_EGDE_2D(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y, 1.0));
                t->setTexture(texture);
            }
            TriangleList.push_back(t);
        }
    }
    // set sharder
    std::vector<Light> light_defition = {
    { Vector3f{500, 500, 500} ,Vector4f{-20, 20, 0 ,1} }
    };
    g_sharder::EnvPayload env{
        light_defition,
        Vector3f{10, 10, 10}
    };
    g_sharder::Shader sharder { env };
    sharder.set_fragment_shader_algo(g_sharder::phone_sharder_algo);

    //set rasterizer
    rst::rasterizer r(700, 700, std::move(sharder));

    
   // if (argc >= 2)
   // {
   //     command_line = true;
   //     filename = std::string(argv[1]);

   //     if (argc == 3 && std::string(argv[2]) == "texture")
   //     {
   //         std::cout << "Rasterizing using the texture shader\n";
			////sharder.set_fragment_shader_algo(g_sharder::texture_fragment_shader);
   ////         active_shader = texture_fragment_shader;
   //         texture_path = "spot_texture.png";
   //         r.set_texture(Texture(obj_path + texture_path));
   //     }
   //     else if (argc == 3 && std::string(argv[2]) == "normal")
   //     {
   //         std::cout << "Rasterizing using the normal shader\n";
   //         //active_shader = normal_fragment_shader;
   //     }
   //     else if (argc == 3 && std::string(argv[2]) == "phong")
   //     {
   //         std::cout << "Rasterizing using the phong shader\n";
   //         //active_shader = phong_fragment_shader;
   //     }
   //     else if (argc == 3 && std::string(argv[2]) == "bump")
   //     {
   //         std::cout << "Rasterizing using the bump shader\n";
   //         //active_shader = bump_fragment_shader;
   //     }
   //     else if (argc == 3 && std::string(argv[2]) == "displacement")
   //     {
   //         std::cout << "Rasterizing using the bump shader\n";
   //         //active_shader = displacement_fragment_shader;
   //     }
   // }

    Eigen::Vector3f eye_pos = {0,0,1.5};

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 45, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, -0.1, -50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}
