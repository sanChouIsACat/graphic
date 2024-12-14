#include "camera.hpp"
#include "transformation.hpp"
void GComponent::Camera::move(GTypes::POINT_EGDE_3D par)
{
	camera_point.col(3) += par;
}

void GComponent::Camera::rotateByX(float angle)
{
	camera_point = transformation::generateArbitraryRotateMatrix(
		{ 1,0,0,0 }, { 0,0,0,1 }, angle) * camera_point;
}

void GComponent::Camera::rotateByY(float angle)
{
	camera_point = transformation::generateArbitraryRotateMatrix(
		{ 0,1,0,0 }, { 0,0,0,1 }, angle) * camera_point;
}

void GComponent::Camera::rotateByZ(float angle)
{
	camera_point = transformation::generateArbitraryRotateMatrix(
		{ 0,0,1,0 }, { 0,0,0,1 }, angle) * camera_point;
}

Eigen::MatrixX4f GComponent::Camera::getAndUpdateViewTransformMatrix()
{
	// get nature coordinates using old transform matrix
	CameraPoint new_carema = camera_point * old_view_transform.transpose();
	old_view_transform = new_carema;
	camera_point = { 0,0,0,1 };
	return new_carema;
}

Eigen::MatrixX4f GComponent::Camera::getViewTransformMatrix()
{
	return old_view_transform;
}
