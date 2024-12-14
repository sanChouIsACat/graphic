#pragma once
#include <string>
namespace Holder {
class ResourcePathHolder {
private:
  std::string folder_path;
  std::string complete_model_path;
  std::string complete_glsls_path;

  /* data */
public:
  ResourcePathHolder();
  std::string getModelPath();
  std::string getGlslsPath();
};

} // namespace Holder
