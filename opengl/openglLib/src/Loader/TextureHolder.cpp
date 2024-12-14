#include "Holder/TextureHolder.hpp"
#include "Holder/FillHelper.hpp"
#include "boost/format.hpp"
#include "glad/glad.h"
#include "stb_image.h"

namespace Holder {
TextureHolder::TextureHolder(const std::string &texturePath) {

  int width, height, nrChannels;
  unsigned char *data =
      stbi_load(texturePath.c_str(), &width, &height, &nrChannels, 0);
  if (!data) {
    error_log =
        (boost::format("unable to open texture of path [%1%]") % texturePath)
            .str();
    stbi_image_free(data);
    return;
  }
  glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);
  GlFillHelper fillHelper;
  fillHelper.fillSomeThing(glGenTextures, 1, &texture)
      .fillSomeThing(glBindTexture, GL_TEXTURE_2D, texture)
      .fillSomeThing(glTexImage2D, GL_TEXTURE_2D, 0, GL_RGB, width, height, 0,
                     GL_RGB, GL_UNSIGNED_BYTE, data)
      .fillSomeThing(glGenerateMipmap, GL_TEXTURE_2D)
      .fillSomeThing(glBindTexture, GL_TEXTURE_2D, 0);
  error_log = fillHelper.error_log;
  stbi_image_free(data);
}

TextureHolder::~TextureHolder() { glDeleteTextures(1, &texture); }
} // namespace Holder
