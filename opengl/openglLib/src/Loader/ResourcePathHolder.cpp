#include "Holder/ResourcePathHolder.hpp"
#include <Windows.h>
#include <algorithm>
#include <filesystem>

#ifndef MODEL_RESOURCE_PATH
#error "add model resource def in cmake
#endif
#ifndef GLSLS_RESOURCE_PATH
#error "add glsls resource def in cmake
#endif

#define STRINGIFY(x) #x
#define TO_STRING(x) STRINGIFY(x)
Holder::ResourcePathHolder::ResourcePathHolder() {
  char path[MAX_PATH];
  GetModuleFileName(NULL, path, MAX_PATH);
  std::filesystem::path exePath(path);
  auto tmp = exePath.parent_path().parent_path().string() + "\\";
  this->complete_glsls_path = tmp + GLSLS_RESOURCE_PATH + "\\";
  this->complete_model_path = tmp + MODEL_RESOURCE_PATH + "\\";
}

std::string Holder::ResourcePathHolder::getModelPath() {
  return complete_model_path;
}

std::string Holder::ResourcePathHolder::getGlslsPath() {
  return complete_glsls_path;
}
