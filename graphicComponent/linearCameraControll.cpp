#include "linearCameraControl.hpp"
using namespace GComponent;

LinearCameraControl::LinearCameraControl(const GTypes::AABB& aabb,
	int screen_width,
	int screen_height,
	float ratio_z,
	float ratio_x, GComponent::Camera& camera) :aabb(aabb),
	screen_width(screen_width),
	screen_height(screen_height),
	ratio_x(ratio_x),
	ratio_z(ratio_z),
	camera(camera),
	last_time_x_press(0),
	last_time_z_press(0),
	press_threshlod(20),
	last_time_mouse_coordiante()
{
};
void LinearCameraControl::keyBoardX(int x, long long timestamp) {
	computeMoveStep(ratio_x, timestamp, last_time_x_press, screen_width, 0);
}
void LinearCameraControl::keyBoardZ(int y, long long timestamp) {
	computeMoveStep(ratio_z, timestamp, last_time_z_press, screen_height, 2);
}
void LinearCameraControl::mouse(int x, int y) {
	float x_len = x - last_time_mouse_coordiante.x();
	float y_len = y - last_time_mouse_coordiante.y();
	float x_angle = x_len / screen_width * ratio_rorate_x;
	float y_angle = y_len / screen_width * ratio_rorate_y;

	std::lock_guard t{ camera.mutex };
	camera.rotateByX(x_angle);
	camera.rotateByY(y_angle);
}
void LinearCameraControl::computeMoveStep(long long& ratio,
	long long current_timestamp,
	long long& timestamp,
	int& screen_len,
	int offset) {
	long long interval = current_timestamp - last_time_x_press;
	float step_len = screen_len * (interval > press_threshlod ? 20 : interval) / ratio;

	//update camera
	std::lock_guard t{ camera.mutex };
	Eigen::Matrix4f movement =  Eigen::Matrix4f::Identity();
	movement.col(3)[3] = 1;
	movement.col(3)[offset] = step_len;
	camera.move(std::move(movement));
	timestamp = current_timestamp;
}