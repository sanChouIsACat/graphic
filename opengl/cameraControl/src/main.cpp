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

#include "camera.hpp"
#include "linearCameraControl.hpp"
#include "logger.hpp"
#include "transformation.hpp"
#include "types.hpp"
#include "view.hpp"
#include <eigen3/Eigen/Eigen>

#include "logger.hpp"

GLuint globalProgram;
long long getCurrentMilliseconds() {
  // 获取当前时间点
  auto now = std::chrono::steady_clock::now();

  // 将当前时间点转换为毫秒
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch());

  // 返回毫秒数
  return duration.count();
}
void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
  glViewport(0, 0, width, height);
  GComponent::LinearCameraControl &cameraControl =
      *(GComponent::LinearCameraControl *)glfwGetWindowUserPointer(window);
  cameraControl.resize(width, height);
}

void key_callback(GLFWwindow *window, int key, int scancode, int action,
                  int mods) {
  GComponent::LinearCameraControl &cameraControl =
      *(GComponent::LinearCameraControl *)glfwGetWindowUserPointer(window);
  if (action == GLFW_PRESS || action == GLFW_REPEAT) {
    if (key == GLFW_KEY_ESCAPE) {
      printf("Escape key pressed! Exiting...\n");
      glfwSetWindowShouldClose(window, GLFW_TRUE); // 关闭窗口
    } else if (key == GLFW_KEY_A) {
      cameraControl.keyBoardX(-1, getCurrentMilliseconds());
    } else if (key == GLFW_KEY_D) {
      cameraControl.keyBoardX(1, getCurrentMilliseconds());
    } else if (key == GLFW_KEY_S) {
      cameraControl.keyBoardZ(-1, getCurrentMilliseconds());
    } else if (key == GLFW_KEY_W) {
      cameraControl.keyBoardZ(1, getCurrentMilliseconds());
    }
    Eigen::Matrix4f newView = cameraControl.camera.getViewTransformMatrix();

    Holder::GlFillHelper::fillUniform(glUniformMatrix4fv, "view", globalProgram,
                                      1, false, newView.data());
  } else if (action == GLFW_RELEASE) {
  }
}
void cursor_position_callback(GLFWwindow *window, double xpos, double ypos) {
  GComponent::LinearCameraControl &cameraControl =
      *(GComponent::LinearCameraControl *)glfwGetWindowUserPointer(window);
  // 打印鼠标的当前坐标
  cameraControl.mouse(xpos, ypos);
  Eigen::Matrix4f newView = cameraControl.camera.getViewTransformMatrix();
  Holder::GlFillHelper::fillUniform(glUniformMatrix4fv, "view", globalProgram,
                                    1, false, newView.data());
}

void cursor_enter_callback(GLFWwindow *window, int entered) {
  GComponent::LinearCameraControl &cameraControl =
      *(GComponent::LinearCameraControl *)glfwGetWindowUserPointer(window);
  if (entered) {
    double xpos, ypos;
    glfwGetCursorPos(window, &xpos, &ypos);
    cameraControl.setEnterCoords(xpos, ypos);
  }
}
constexpr unsigned int width = 800;
constexpr unsigned int height = 600;
int main() {
  // view control
  GComponent::Camera camera;
  GTypes::AABB aabb{GTypes::POINT_EGDE_3D{-1, -1, -1, 1},
                    GTypes::POINT_EGDE_3D{1, 1, 1, 1}};
  GComponent::LinearCameraControl cameraControl{aabb, width, height,
                                                5000, 5000,  camera};

  // cameraControl.keyBoardX();
  // window
  Holder::WindowHolder windowHolder;
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cerr << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  glfwMakeContextCurrent(windowHolder.window);
  glViewport(0, 0, 800, 600);
  glfwSetFramebufferSizeCallback(windowHolder.window,
                                 framebuffer_size_callback);
  glfwSetKeyCallback(windowHolder.window, key_callback);
  glfwSetCursorPosCallback(windowHolder.window, cursor_position_callback);
  glfwSetCursorEnterCallback(windowHolder.window, cursor_enter_callback);
  glfwSetWindowUserPointer(windowHolder.window, &cameraControl);
  // opengl
  Holder::ModelTextureHolder modelHolder{"spot\\spot_triangulated_good.obj",
                                         "spot\\spot_texture.png"};
  Holder::ShaderCompiler compiler;

  auto [program, error_log] = compiler.toCurrentDir()
                                  .shader(GL_VERTEX_SHADER, "vertex.glsl")
                                  .shader(GL_FRAGMENT_SHADER, "fragment.glsl")
                                  .program();
  globalProgram = program;
  if (!error_log.empty()) {
    std::cout << error_log;
    return -1;
  }

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
  glUseProgram(program);
  // transform matrix
  Eigen::Matrix4f p = view::generatePerspectiveProjection(
      GTypes::POINT_EGDE_3D{-1, -1, -0.01, 1},
      GTypes::POINT_EGDE_3D{1, 1, -1, 1}, -1);
  // G_LOGGER_INFO("p: %s", EigenStructToString(p).c_str());
  Eigen::Matrix4f v = Eigen::Matrix4f::Identity();

  Holder::GlFillHelper::fillUniform(glUniformMatrix4fv, "perspective", program,
                                    1, false, v.data());
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
