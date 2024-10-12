#pragma once
#include <eigen3/Eigen/Eigen>

namespace algebra {
	/*
	* generate an orthogonal basis which contains the given vector.
	*/
	Eigen::Matrix4f generateOrthogonal3DBasis(
		const Eigen::Vector4f & vector, const Eigen::Vector4f& point);
	template<typename T>
	Eigen::Vector3f homogeneousToNormal(const T& vector)
	{
		Eigen::Vector3f normal;
		float w = vector.w();
		w = w == 0 ? 1 : w;
		normal << vector.x() / w, vector.y() / w, vector.z() / w;
		return normal;
	}
}