#include "linearCameraControl.hpp"
#include <chrono>
#include <ctime>
#include "transformation.hpp"
#define _USE_MATH_DEFINES
#include <cmath>

long long getCurrentTimeStamp() {
	auto now = std::chrono::system_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
		now.time_since_epoch()
	);
	return duration.count();

}
GComponent::LinearCameraControl::LinearCameraControl
(const GTypes::AABB& aabb,
	int screen_width,
	int screen_height,
	float ratio_z,
	float ratio_x,
	GComponent::Camera& camera): aabb(aabb),
	screen_width(screen_width),
	screen_height(screen_height),
	ratio_z(ratio_z), ratio_x(ratio_x),
	camera(camera)
{
}

void GComponent::LinearCameraControl::keyBoardX(int x)
{
	float step = computeMoveStep(this->ratio_x, this->last_time_x_press, this->screen_width, x);
	camera.move(transformation::generateTranslateMatrix(step, 0, 0));
}

void GComponent::LinearCameraControl::keyBoardZ(int y)
{
	float step = computeMoveStep(this->ratio_z, this->last_time_z_press, this->screen_width, y);
	camera.move(transformation::generateTranslateMatrix(0, step, 0));
}

void GComponent::LinearCameraControl::mouse(int x, int y)
{
	auto step_compute_f = [](int screen_len, int l) -> float {
		return l / screen_len * (-M_PI);
		};
	float x_step = x - last_time_mouse_coordiante.x();
	float y_step = y - last_time_mouse_coordiante.y();
	camera.rotateByY(step_compute_f(this->screen_height, y_step));
	camera.rotateByX(step_compute_f(this->screen_width, x_step));
	last_time_mouse_coordiante = Eigen::Matrix2i{ x,y };
}



float GComponent::LinearCameraControl::computeMoveStep(long long& ratio, long long& timestamp, int& screen_len, int sign)
{
	long long current_time_stamp = getCurrentTimeStamp();
	long long interval = current_time_stamp - timestamp;
	if (interval > press_threshlod) {
		interval = 25;
	}
	float move_step = interval / ratio * screen_len * sign;
	timestamp = getCurrentTimeStamp();
	return move_step;
}
