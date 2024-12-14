#pragma once
#include "logger.hpp"
#include <opencv2/opencv.hpp>
static bool has_window = false;
void onMouse(int event, int x, int y, int flags, void *userdata) {
  if (event == cv::EVENT_LBUTTONDOWN) {
    G_LOGGER_TRACE("x:{%d}, y:{%d}", x, y);
  }
}