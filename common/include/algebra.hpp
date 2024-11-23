#pragma once
#include <eigen3/Eigen/Eigen>
#include "g_type_traits.hpp"

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
	Eigen::Vector3f homogeneousToNormalCoordinate(const T& vector)
	{
		Eigen::Vector3f normal;
		float w = vector.w();
		w = w == 0 ? 1 : w;
		normal << vector.x() / w, vector.y() / w, vector.z() / w;
		return normal;
	}
	
	// let w be 1. The return value will be it self.
	Eigen::Vector4f homogeneousNormalize(Eigen::Vector4f& point);

	// normalize the vector to the given length.
	template<typename T>
	T normalizeToLength(const T& vector, float length)
	{
		return vector.normalized() * length;
	}

	// normalize the vector to the given length inplace.
	template<typename T>
	void normalizeToLengthInPlace(T& vector, float length)
	{
		vector.normalize();
		vector = vector * length;
	}

	template<typename T>
	bool insideCircle(const T& circle, const T& point, float radiu) {
		static_assert(g_type_traits::has_two_dimensions_v<T>, "using at least two dimensions vectors");
		return (std::pow(circle.x() - point.x(), 2) + std::pow(circle.y() - point.y(), 2)) < radiu;
	}

	template<typename T>
	bool isApproxEqual(const T& v1, const T& v2, float epsilon = 1e-5) {
		static_assert(g_type_traits::has_norm_function<T>, "using a vector");

		return (v1 - v2).norm() < epsilon;
	}
}