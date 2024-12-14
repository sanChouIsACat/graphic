#include "Holder/WindowHolder.hpp"

Holder::WindowHolder::WindowHolder() {
  glfwInit();
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  window = glfwCreateWindow(800, 600, "OpenGL Triangle", NULL, NULL);
  glfwMakeContextCurrent(window);
}

Holder::WindowHolder::~WindowHolder() { glfwTerminate(); }
