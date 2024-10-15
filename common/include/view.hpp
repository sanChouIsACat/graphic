#pragma once
#include <eigen3/Eigen/Eigen>
namespace view {
	// generate an orthogonalProjection matrix
	Eigen::Matrix4f generateOrthogonalProjection(const Eigen::Vector4f& down_left, const Eigen::Vector4f& top_right);

	// generate a perspective projection matrix
	Eigen::Matrix4f generatePerspectiveProjection(const Eigen::Vector4f& down_left, const Eigen::Vector4f& top_right, float far);
}