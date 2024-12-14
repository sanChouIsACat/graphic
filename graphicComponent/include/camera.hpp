#pragma once
#include "eigen3/Eigen/Eigen"
#include "types.hpp"
#include "mutex"
namespace GComponent {
	class Camera {
	private:
		using CameraPoint = Eigen::Matrix4f;
	private:
		CameraPoint camera_point;
		Eigen::Matrix4f old_view_transform;
	public:
		Camera();
		std::recursive_mutex mutex;
		// all movement calculations are in carema coordinates systems
		void move(Eigen::Matrix4f par);
		void rotateByX(float angle);
		void rotateByY(float angle);
		void rotateByZ(float angle);
		// get current camera transform matrix and update carema coordinates systems to new one
		Eigen::MatrixX4f getAndUpdateViewTransformMatrix();
		// get origin(the same as last time getAndUpdateViewTransformMatrix returned) camera transform matrix
		Eigen::MatrixX4f getViewTransformMatrix();

	};
}