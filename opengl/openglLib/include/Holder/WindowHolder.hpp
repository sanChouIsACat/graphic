#include "GLFW/glfw3.h"

namespace Holder {
class WindowHolder {
private:
  /* data */
public:
  GLFWwindow *window;
  WindowHolder();
  ~WindowHolder();
};

} // namespace Holder
