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
#include "Holder/ModelTextureHolder.hpp"
#include "Holder/ResourcePathHolder.hpp"
#include "Holder/SharderCompiler.hpp"
#include "Holder/WindowHolder.hpp"

#include "stb_image.h"
#include <filesystem>
#include <windows.h>

void framebuffer_size_callback(GLFWwindow *window, int width, int height) {
  glViewport(0, 0, width, height);
}

void processInput(GLFWwindow *window) {
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    glfwSetWindowShouldClose(window, true);
}

int main() {
  Holder::WindowHolder windowHolder;
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cerr << "Failed to initialize GLAD" << std::endl;
    return -1;
  }

  Holder::ModelTextureHolder modelHolder{"spot\\spot_triangulated_good.obj",
                                         "spot\\spot_texture.png"};
  Holder::ShaderCompiler compiler;

  auto [program, error_log] =
      compiler.toCurrentDir()
          .shader(GL_VERTEX_SHADER, "texture\\vertex.glsl")
          .shader(GL_FRAGMENT_SHADER, "texture\\fragment.glsl")
          .program();
  if (!error_log.empty()) {
    std::cout << error_log;
    return -1;
  }
  glfwMakeContextCurrent(windowHolder.window);
  glViewport(0, 0, 800, 600);
  glfwSetFramebufferSizeCallback(windowHolder.window,
                                 framebuffer_size_callback);

  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);

  glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
  while (!glfwWindowShouldClose(windowHolder.window)) {
    processInput(windowHolder.window);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram(program);
    modelHolder.bind();
    glDrawElements(GL_TRIANGLES, modelHolder.modelHolder.indCount,
                   GL_UNSIGNED_INT, 0);
    glfwSwapBuffers(windowHolder.window);
    glfwPollEvents();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}
