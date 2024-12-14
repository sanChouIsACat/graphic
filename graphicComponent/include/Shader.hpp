//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_SHADER_H
#define RASTERIZER_SHADER_H
#include <eigen3/Eigen/Eigen>
#include "Texture.hpp"
#include "types.hpp"
namespace g_sharder {

    class Shader {
    public:
        Shader& set_vertex_shader_algo(VertexShaderAlgo algo);
        Shader& set_fragment_shader_algo(FragmentShaderAlgo algo);
		Eigen::Vector3f shade_pixel(const FragmentShaderPayload& payload) const;
        
        template<typename U>
        Shader(U&& env, typename std::enable_if_t<std::is_same_v<std::decay_t<U>, EnvPayload>>* = 0) :
            env_payload(std::forward<U>(env)) {};

        template<typename T>
		typename std::enable_if_t<std::is_same_v<std::decay_t<T>, std::vector<GTypes::Light>>, Shader>
            set_lights(T&& light) {
			this->env_payload.lights = std::forward<T>(light);
		}
        template<typename T>
        typename std::enable_if_t<std::is_same_v<std::decay_t<T>, EnvPayload>, Shader>
            set_env_light(T&& env) {
            this->env_payload = env;
        }
    private:
        EnvPayload env_payload;
        FragmentShaderAlgo fragment_shader_algo;
        VertexShaderAlgo vertex_shader_algo;
    };
}


#endif //RASTERIZER_SHADER_H
