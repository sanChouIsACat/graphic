#include "thread"
#include <fstream>
#include <glad/glad.h>

// #include <glad/glad.h>
#include <iostream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#include "Holder/FillHelper.hpp"
#include "Holder/ModelTextureHolder.hpp"
#include "Holder/ResourcePathHolder.hpp"
#include "Holder/SharderCompiler.hpp"
#include "Holder/WindowHolder.hpp"

#include "stb_image.h"
#include <filesystem>
#include <windows.h>

#include "DiContainer/DiContainerImplement.hpp"
#include "camera.hpp"
#include "camera/openglCameraControl.hpp"
#include "linearCameraControl.hpp"
#include "logger.hpp"
#include "transformation.hpp"
#include "types.hpp"
#include "view.hpp"
#include <eigen3/Eigen/Eigen>
constexpr unsigned int width = 800;
constexpr unsigned int height = 600;
int main() {
  // init window
  Holder::WindowHolder windowHolder;
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cerr << "Failed to initialize GLAD" << std::endl;
    return -1;
  }
  glViewport(0, 0, 800, 600);

  // init sharder
  Holder::ShaderCompiler compiler;
  auto [program, error_log] = compiler.toCurrentDir()
                                  .shader(GL_VERTEX_SHADER, "vertex.glsl")
                                  .shader(GL_FRAGMENT_SHADER, "fragment.glsl")
                                  .program();

  // init camera
  GComponent::Camera camera;
  GTypes::AABB aabb{GTypes::POINT_EGDE_3D{-1, -1, -1, 1},
                    GTypes::POINT_EGDE_3D{1, 1, 1, 1}};
  OpenglCamera::openglCameraControl<GComponent::LinearCameraControl>
      cameraControl{windowHolder.window,
                    program,
                    aabb,
                    width,
                    height,
                    5000,
                    5000,
                    camera};

  // load model
  Holder::ModelTextureHolder modelHolder{"spot\\spot_triangulated_good.obj",
                                         "spot\\spot_texture.png"};
  if (!error_log.empty()) {
    std::cout << error_log;
    return -1;
  }

  // init opengl options
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
  glUseProgram(program);

  // init view and perspective transform
  Eigen::Matrix4f p = view::generatePerspectiveProjection(
      GTypes::POINT_EGDE_3D{-1, -1, 1, 1}, GTypes::POINT_EGDE_3D{1, 1, -1, 1},
      -1);
  Eigen::Matrix4f v = Eigen::Matrix4f::Identity();
  Holder::GlFillHelper::fillUniform(glUniformMatrix4fv, "perspective", program,
                                    1, false, p.data());
  Holder::GlFillHelper::fillUniform(glUniformMatrix4fv, "view", program, 1,
                                    false, v.data());

  // loop
  while (!glfwWindowShouldClose(windowHolder.window)) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    modelHolder.bind();
    glUseProgram(program);
    glDrawElements(GL_TRIANGLES, modelHolder.modelHolder.indCount,
                   GL_UNSIGNED_INT, 0);
    glfwSwapBuffers(windowHolder.window);
    glfwPollEvents();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}
