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
		translate_matrix << 1, 0, 0, translate_x,
			0, 1, 0, translate_y,
			0, 0, 1, translate_z,
			0, 0, 0, 1;

		return translate_matrix;
	}

	Eigen::Matrix4f generateWindowingMatrix(const Eigen::Vector4f down_left,
		const Eigen::Vector4f top_right,
		const Eigen::Vector4f new_down_left,
		const Eigen::Vector4f new_top_right)
	{
		Matrix4f translate_to_origin_m = generateTranslateMatrix(-down_left.x(), -down_left.y(), -down_left.z());
		G_LOGGER_TRACE("translate_to_origin_m :\n %s", EigenStructToString(translate_to_origin_m).c_str());
		float x_scale_ratio = (new_top_right.x() - new_down_left.x()) / (top_right.x() - down_left.x());
		float y_scale_ratio = (new_top_right.y() - new_down_left.y()) / (top_right.y() - down_left.y());
		float z_scale_ratio = (new_top_right.z() - new_down_left.z()) / (top_right.z() - down_left.z());
		G_LOGGER_TRACE("x:%f, y:%f, z:%f", x_scale_ratio, y_scale_ratio, z_scale_ratio);
		Matrix4f scale_m = generateScaleMatrix(x_scale_ratio,y_scale_ratio,z_scale_ratio);
		G_LOGGER_TRACE("scale_m :\n %s", EigenStructToString(scale_m).c_str());
		Matrix4f translate_back_m = generateTranslateMatrix(new_down_left.x(), new_down_left.y(), new_down_left.z());
		G_LOGGER_TRACE("translate_back_m :\n %s", EigenStructToString(translate_back_m).c_str());
		return translate_back_m * scale_m * translate_to_origin_m;
	}
}

