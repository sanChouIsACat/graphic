#pragma once
#include <eigen3/Eigen/Eigen>

namespace algebra {
	/*
	* generate an orthogonal basis which contains the given vector.
	* the vector is the nth-axis of the new basis.
	*/
	Eigen::Matrix4f generateOrthogonalBasis(
		const Eigen::Vector4f & vector, const Eigen::Vector4f& point, int vector_offset);
	
	/*
	* generate an matrix P that transform the xy coordinate to the new coordinate system.
	* the given vector is the nth-axis of the new basis.
	*/
	Eigen::Matrix4f generateBasisTransformationMatrix(
		const Eigen::Vector4f& vector, const Eigen::Vector4f& point, int vector_offset);
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