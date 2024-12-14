#include "Holder/SharderCompiler.hpp"
#include "Holder/ResourcePathHolder.hpp"
#include "algorithm"
#include <Windows.h>

namespace Holder {
ShaderCompiler &ShaderCompiler::toCurrentDir() {
  ResourcePathHolder resourcePathHolder;
  this->current_glsls_resource_path = resourcePathHolder.getGlslsPath();
  return *this;
}

ShaderCompiler &ShaderCompiler::shader(GLenum type,
                                       const std::string &filePath) {

  shaderFiles.emplace_back(type, this->current_glsls_resource_path + filePath);
  return *this;
}

std::tuple<GLuint, std::string> ShaderCompiler::program() {
  if (isCompiled) {
    return {programHandle, "Program already compiled."};
  }

  programHandle = glCreateProgram();
  std::vector<GLuint> compiledShaders;
  std::string errorLog;

  for (const auto &[type, filePath] : shaderFiles) {
    // Check if file exists and read its contents
    // std::replace(filePath.begin(), filePath.end(), '/', '\\');
    std::ifstream shaderFile(filePath);
    if (!shaderFile.is_open()) {
      errorLog = "Failed to open shader file: " + filePath + "\n";
      break;
    }

    std::stringstream buffer;
    buffer << shaderFile.rdbuf();
    std::string source = buffer.str();
    shaderFile.close();

    GLuint shader = glCreateShader(type);
    const char *sourceCStr = source.c_str();
    glShaderSource(shader, 1, &sourceCStr, nullptr);
    glCompileShader(shader);

    GLint success;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
      char log[512];
      glGetShaderInfoLog(shader, 512, nullptr, log);
      errorLog = "Shader Compilation Error in file " + filePath + ": \n" +
                 std::string(log) + "\n";
      glDeleteShader(shader);
      break;
    }

    glAttachShader(programHandle, shader);
    compiledShaders.push_back(shader);
  }

  if (!errorLog.empty()) {
    // Clean up shaders and program on failure
    for (GLuint shader : compiledShaders) {
      glDeleteShader(shader);
    }
    glDeleteProgram(programHandle);
    programHandle = 0;
    return {0, errorLog};
  }

  glLinkProgram(programHandle);
  GLint linkSuccess;
  glGetProgramiv(programHandle, GL_LINK_STATUS, &linkSuccess);
  if (!linkSuccess) {
    char log[512];
    glGetProgramInfoLog(programHandle, 512, nullptr, log);
    errorLog = "Program Linking Error: \n" + std::string(log);

    for (GLuint shader : compiledShaders) {
      glDeleteShader(shader);
    }
    glDeleteProgram(programHandle);
    programHandle = 0;
    return {0, errorLog};
  }

  for (GLuint shader : compiledShaders) {
    glDeleteShader(shader); // Shaders can be deleted after linking
  }

  isCompiled = true;
  return {programHandle, ""}; // Success, no error message
}

ShaderCompiler::~ShaderCompiler() {
  if (programHandle != 0) {
    glDeleteProgram(programHandle);
  }
}

} // namespace Holder
