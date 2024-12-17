#include "boost/format.hpp"
#include "glad/glad.h"
#include <functional>
#include <string>

namespace Holder {
class GlFillHelper {
private:
  bool errored = false;
  unsigned int i;

public:
  std::string error_log;

private:
  static std::string getLastGlError() {
    GLenum errorNo = glGetError();
    if (errorNo != GL_NO_ERROR) {
      return (boost::format("error happened with error num [%1%]") % errorNo)
          .str();
    }
    return "";
  }

public:
  template <typename K, typename... Args>
  GlFillHelper &fillSomeThing(K *functionP, Args... args) {
    if (errored) {
      i++;
      return *this;
    }
    functionP(std::forward<Args>(args)...);
    GLenum errorNo = glGetError();
    if (errorNo != GL_NO_ERROR) {
      error_log =
          (boost::format(
               "error happened at function [%1%] with error num [%2%]") %
           i % errorNo)
              .str();
    }
    i++;
    return *this;
  }
  template <typename K, typename... Args>
  std::string static fillUniform(K function_p, const std::string &name,
                                 const GLuint programID, Args... args) {
    GLint location = glGetUniformLocation(programID, name.c_str());
    function_p(location, std::forward<Args>(args)...);
    // TODO: return error msg
    return getLastGlError();
  }
};
} // namespace Holder
