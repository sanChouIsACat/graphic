#pragma once
#include "eigen3/Eigen/Eigen"
namespace interpolation {
	/*
	* Both functin will return true if the point is inside the triangle.
	* Note the compartion is in 2D and triangle is projected.
	*/ 
	bool insideTriangle(float x, float y,const std::array<Eigen::Vector4f, 3>& edges);
}