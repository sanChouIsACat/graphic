#include "bezierCurve.hpp"
#include "logger.hpp"

void GAlgo::BezierCurve::drawCurve(
    float sample_rate, const std::vector<POINT_EGDE_2D> &control_point,
    std::function<RGB(float t)> color_f) {
  if (control_point.empty()) {
    return;
  }
  // power of bezier curve
  int n = control_point.size() - 1;
  int buffer_size = n + 1;

  // resize buffer if necessary
  if (buffer_size != buffer.size()) {
    buffer.resize(buffer_size);
    for (int i = 0; i <= n; i++) {
      buffer[i] = combination(n, i);
    }
  }

  for (float i = 0; i <= 1; i += sample_rate) {
    float x = 0;
    float y = 0;
    for (int j = 0; j <= n; j++) {
      float control_point_x = control_point[j].x();
      float control_point_y = control_point[j].y();
      float time_coefficient = std::pow(1 - i, n - j) * std::pow(i, j);
      x += buffer[j] * time_coefficient * control_point[j].x();
      y += buffer[j] * time_coefficient * control_point[j].y();
    }
    set_pixel_f(std::round(x), std::round(y), color_f(i));
  }
}

long long GAlgo::BezierCurve::combination(int n, int k) {
  if (k > n)
    return 0;
  if (k == 0 || k == n)
    return 1;
  k = std::min(k, n - k);
  long long result = 1;
  for (int i = 1; i <= k; ++i) {
    result = result * (n - k + i) / i;
  }
  return result;
}
