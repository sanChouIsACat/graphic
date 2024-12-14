#include "algo.hpp"
using namespace GTypes;
RGB GAlgo::generateRainbowColor(float t) {
  // 将 t 映射到 HSV 色相 (Hue) 范围 [0, 360]
  float hue = 360.0f * t;

  // HSV 转 RGB 的逻辑
  float c = 1.0f; // 饱和度为 1.0，亮度为 1.0，对应彩虹效果
  float x = c * (1 - fabs(fmod(hue / 60.0, 2) - 1));
  float m = 0; // 亮度校正，这里可以保持为 0

  float r, g, b;
  if (hue < 60) {
    r = c;
    g = x;
    b = 0;
  } else if (hue < 120) {
    r = x;
    g = c;
    b = 0;
  } else if (hue < 180) {
    r = 0;
    g = c;
    b = x;
  } else if (hue < 240) {
    r = 0;
    g = x;
    b = c;
  } else if (hue < 300) {
    r = x;
    g = 0;
    b = c;
  } else {
    r = c;
    g = 0;
    b = x;
  }

  // 将 [0, 1] 的值映射到 [0, 255]，并返回 RGB 结构
  return RGB{(r + m) * 255, (g + m) * 255, (b + m) * 255};
}

void GAlgo::renderCircle(
    const POINT_EGDE_2D &center, float radio,
    std::function<void(const POINT_EGDE_2D &, const RGB &)> set_pixel_f,
    std::function<RGB(const POINT_EGDE_2D &)> get_color_f) {
  int x_min = center.x() - radio;
  int x_max = center.x() + radio;
  int y_min = center.y() - radio;
  int y_max = center.y() + radio;

  // 遍历包围盒中的每个像素点
  for (int x = x_min; x <= x_max; ++x) {
    for (int y = y_min; y <= y_max; ++y) {
      // 使用 get_function 检查像素点是否在圆的范围内
      if (std::pow(x - center.x(), 2) + std::pow(y - center.y(), 2) <=
          std::pow(radio, 2)) {
        POINT_EGDE_2D current_point{(float)x, (float)y, 1};
        set_pixel_f(current_point, get_color_f(current_point));
      }
    }
  }
}

std::tuple<float, float, float, float>
GAlgo::getRoundingBox(const Triangle &tri) {
  float min_x = std::numeric_limits<float>::max();
  float min_y = std::numeric_limits<float>::max();
  float max_x = std::numeric_limits<float>::min();
  float max_y = std::numeric_limits<float>::min();

  for (const Vector4f &edge : tri.v) {

    float x = edge.x();
    float y = edge.y();
    min_x = min_x < x ? min_x : x;
    min_y = min_y < y ? min_y : y;
    max_x = max_x > x ? max_x : x;
    max_y = max_y > y ? max_y : y;
  }
  return std::make_tuple(min_x, min_y, max_x, max_y);
}

Eigen::Vector3f GAlgo::computeLightPlaneInterSection(const Light &light,
                                                     const Plane &plane) {
  Eigen::Vector3f b = plane.origin.head<3>() - light.position.head<3>();
  Matrix3f equaltion;
  equaltion.col(0) = light.dir.head<3>();
  equaltion.col(1) = plane.basis1.head<3>();
  equaltion.col(2) = plane.basis2.head<3>();
  Matrix<float, 3, 4> extend;
  extend << equaltion, b;
  auto extend_qr = extend.colPivHouseholderQr();
  auto equaltion_qr = equaltion.colPivHouseholderQr();
  if (extend_qr.rank() != equaltion_qr.rank()) {
    return Vector3f{-1, -1, -1};
  }

  Vector3f res = equaltion_qr.solve(b);
  res[1] = -res[1];
  res[2] = -res[2];
  return res;
}

std::tuple<bool, Eigen::Matrix<float, 3, 2>>
GAlgo::computeLightAABBInterSection(const Light &light, const AABB &aabb) {
  int constexpr arr_len = 6;
  std::array<Plane, arr_len> planes;

  // 顶面
  planes[0] = {
      aabb.top_2 - aabb.top_1, // basis1
      aabb.top_4 - aabb.top_1, // basis2
      aabb.top_1               // origin
  };

  // 底面
  planes[1] = {aabb.bottom_2 - aabb.bottom_1, aabb.bottom_4 - aabb.bottom_1,
               aabb.bottom_1};

  // 前面
  planes[2] = {aabb.top_1 - aabb.bottom_1, aabb.bottom_2 - aabb.bottom_1,
               aabb.bottom_1};

  // 后面
  planes[3] = {aabb.top_4 - aabb.bottom_4, aabb.bottom_3 - aabb.bottom_4,
               aabb.bottom_4};

  // 左面
  planes[4] = {aabb.top_1 - aabb.bottom_1, aabb.bottom_4 - aabb.bottom_1,
               aabb.bottom_1};

  // 右面
  planes[5] = {aabb.top_2 - aabb.bottom_2, aabb.bottom_3 - aabb.bottom_2,
               aabb.bottom_2};

  using RES_T = Eigen::Matrix<float, 3, 2>;
  RES_T res;
  int ans_num = 0;
  for (int i = 0; i < arr_len && ans_num < 2; i++) {
    Vector3f slove = GAlgo::computeLightPlaneInterSection(light, planes[i]);
    if (slove[0] > 0 && slove[1] <= 1 && slove[2] <= 1)
      res.col(ans_num++) = slove;
  }
  if (ans_num < 2) {
    return {false, RES_T{}};
  }
  return {true, std::move(res)};
}
