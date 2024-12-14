#include "interpolation.hpp"
#include "logger.hpp"

using namespace Eigen;

void interpolation::tanx_line_draw(
    const Eigen::Vector3f &begin, const Eigen::Vector3f &end,
    std::function<void(const Eigen::Vector3f &, const Eigen::Vector3f &)>
        set_pixel) {

  Vector3f &left = (Vector3f &)begin;
  Vector3f &right = (Vector3f &)end;
  // make sure begin is left than end
  if (begin.x() > end.x()) {
    left = end;
    right = begin;
  }

  float x1 = begin.x();
  float y1 = begin.y();
  float x2 = end.x();
  float y2 = end.y();

  // for horizontal lines
  if (x1 == x2) {
    for (int i = std::min(y1, y2); i <= std::max(y1, y2); i++) {
      Eigen::Vector3f point = Eigen::Vector3f(x1, i, 1.0f);
      set_pixel(point, Eigen::Vector3f(255, 255, 255));
    }
    return;
  }

  float ratio = (y2 - y1) / (x2 - x1);
  int cur = y1;
  if (ratio < 0) {
    for (int i = x1; i < x2; i++) {
      int next = y1 + ratio * (i + 1 - x1);
      int middle = (int)(cur + next) / 2;
      int length = cur - next;
      /*
       * should draw additional pixals to connect the line.
       * cautions: all pixals that has the same y coordiante as y2 should be
       * drawn too.
       */
      for (int j = cur; j >= next && j >= (int)y2; j--) {
        Eigen::Vector3f point = Eigen::Vector3f(i, j, 1.0f);
        int value = 255;
        set_pixel(point, Eigen::Vector3f(value, value, value));
      }
      cur = next;
    }
  } else {
    // the same as previous. Just reverse the relation
    for (int i = x1; i < x2; i++) {
      int next = y1 + ratio * (i + 1 - x1);
      for (int j = cur; j <= next && j <= std::ceil(y2); j++) {
        if (std::abs(ratio) >= 4) {
          G_LOGGER_TRACE("i: {%d}, j: {%d}", i, j);
        }
        Eigen::Vector3f point = Eigen::Vector3f(i, j, 1.0f);
        set_pixel(point, Eigen::Vector3f(255, 255, 255));
      }
      cur = next;
    }
  }

  G_LOGGER_TRACE("the final pixal: {%f,%f}", x2, y2);
  set_pixel(Eigen::Vector3f{x2, y2, 1}, Eigen::Vector3f(255, 255, 255));
}

void interpolation::standard_line_draw(
    const Eigen::Vector3f &begin, const Eigen::Vector3f &end,
    std::function<void(const Eigen::Vector3f &, const Eigen::Vector3f &)>
        set_pixel) {
  auto x1 = begin.x();
  auto y1 = begin.y();
  auto x2 = end.x();
  auto y2 = end.y();

  Eigen::Vector3f line_color = {255, 255, 255};

  int x, y, dx, dy, dx1, dy1, px, py, xe, ye, i;

  dx = x2 - x1;
  dy = y2 - y1;
  dx1 = fabs(dx);
  dy1 = fabs(dy);
  px = 2 * dy1 - dx1;
  py = 2 * dx1 - dy1;

  if (dy1 <= dx1) {
    if (dx >= 0) {
      x = x1;
      y = y1;
      xe = x2;
    } else {
      x = x2;
      y = y2;
      xe = x1;
    }
    Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
    set_pixel(point, line_color);
    for (i = 0; x < xe; i++) {
      x = x + 1;
      if (px < 0) {
        px = px + 2 * dy1;
      } else {
        if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) {
          y = y + 1;
        } else {
          y = y - 1;
        }
        px = px + 2 * (dy1 - dx1);
      }
      //            delay(0);
      Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
      set_pixel(point, line_color);
    }
  } else {
    if (dy >= 0) {
      x = x1;
      y = y1;
      ye = y2;
    } else {
      x = x2;
      y = y2;
      ye = y1;
    }
    Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
    set_pixel(point, line_color);
    for (i = 0; y < ye; i++) {
      y = y + 1;
      if (py <= 0) {
        py = py + 2 * dx1;
      } else {
        if ((dx < 0 && dy < 0) || (dx > 0 && dy > 0)) {
          x = x + 1;
        } else {
          x = x - 1;
        }
        py = py + 2 * (dx1 - dy1);
      }
      //            delay(0);
      Eigen::Vector3f point = Eigen::Vector3f(x, y, 1.0f);
      set_pixel(point, line_color);
    }
  }
}