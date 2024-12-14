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
};
} // namespace Holder
