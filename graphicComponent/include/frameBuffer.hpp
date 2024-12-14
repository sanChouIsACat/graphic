#pragma once
#include "types.hpp"
#include <vector>
namespace GComponent {
class FrameBuffer {
protected:
  std::vector<RGB> frame_buf;
  int width, height;

public:
  FrameBuffer(int w, int h) : width(w), height(h) { frame_buf.resize(w * h); }
  void set_pixel(const POINT_EGDE_2D &point, const RGB &color) {
    if (point.y() >= width || point.x() >= height || point.x() < 0 ||
        point.y() < 0) {
      return;
    }
    // old index: auto ind = point.y() + point.x() * width;
    int ind = point.y() * width + point.x();
    frame_buf[ind] = color;
  }
  int getWidth() { return width; }
  int getHeight() { return height; }
};
} // namespace GComponent
