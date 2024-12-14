#ifndef SHADERCOMPILER_H
#define SHADERCOMPILER_H

#include "glad/glad.h"
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>
#include <vector>

namespace Holder {

class ShaderCompiler {
private:
  std::string current_glsls_resource_path;
  std::vector<std::pair<GLenum, std::string>>
      shaderFiles; // Stores type and file paths
  GLuint programHandle = 0;
  bool isCompiled = false;

public:
  ShaderCompiler &toCurrentDir();
  ShaderCompiler &shader(GLenum type, const std::string &filePath);
  std::tuple<GLuint, std::string> program();
  ~ShaderCompiler();
};

} // namespace Holder

#endif // SHADERCOMPILER_H
