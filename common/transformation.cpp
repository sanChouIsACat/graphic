#pragma once

#include <eigen3/Eigen/Eigen>
#include "gasserts.hpp"
#include "algebra.hpp"
#include "transformation.hpp"

using namespace Eigen;
namespace transformation {
	Eigen::Matrix4f generateScaleMatrix(float scale_x, float scale_y, float scale_z)
	{
		Eigen::Matrix4f scale_matrix;
		scale_matrix << scale_x, 0, 0, 0,
			0, scale_y, 0, 0,
			0, 0, scale_z, 0,
			0, 0, 0, 1;
		return scale_matrix;
	}


	Eigen::Matrix4f generateRotateByZMatrix(float angle)
	{
		const float angle_cos = std::cos(angle);
		const float angle_sin = std::sin(angle);
		Matrix4f rotate_matrix;
		rotate_matrix << angle_cos, -angle_sin, 0, 0,
			angle_sin, angle_cos, 0, 0,
			0, 0, 1, 0,
			0, 0, 0, 1;

		return rotate_matrix;
	}
	Eigen::Matrix4f generateArbitraryRotateMatrix(const Eigen::Vector4f direction, const Eigen::Vector4f point, float angle)
	{
		G_ASSERTS_TRUE(direction.w() == 0, "the direction should not be a point.");
		G_ASSERTS_TRUE(point.w() != 0, "the point should not be a vector.");
		float x = direction.x();
		float y = direction.y();
		float z = direction.z();
		float a = point.x();
		float b = point.y();
		float c = point.z();
		G_ASSERTS_FALSE(x == 0 && y == 0 && z == 0, "the direction should not be zero.");

		/*
		* It is calculated by the formula R =  * D * inverse of P.
		* P is the matrix that is cols vectors are the new axis's coordinate represented in origin coordinate system.
		* the new axis is generated from direction and point var. 
		* D is the matrix that rotate angle around by Z axis.
		* P' is the inverse matrix of P. Which actually the transpose matrix of P.
		*
		* The whole idea is:
		* 1. find an orthogonal basis that treat direction variable as the z-axis. 
		* Note that new axises are not uniqued.
		* 2. transform the natural coordinate system to the new coordinate system.
		* 3. rotate by z-axis.
		* 4. transform back.
		* 
		*/
		
		const Matrix4f P = algebra::generateOrthogonalBasis(direction, point, 2);
		const Matrix4f D = generateRotateByZMatrix(angle);
		return P * D * P.inverse();
	}
	
	Eigen::Matrix4f generateTranslateMatrix(float translate_x, float translate_y, float translate_z)
	{
		Eigen::Matrix4f translate_matrix;
		translate_matrix << 0, 0, 0, translate_x,
			0, 0, 0, translate_y,
			0, 0, 0, translate_z,
			0, 0, 0, 1;

		return translate_matrix;
	}
}

