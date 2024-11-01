#pragma once
#include "eigen3/Eigen/Eigen"
#include "logger.hpp"
namespace interpolation {
	/*
	* Both functin will return true if the point is inside the triangle.
	* Note the compartion is in 2D and triangle is projected.
	*/ 
	bool insideTriangle(float x, float y,const std::array<Eigen::Vector4f, 3>& edges);

	/*
	* use dy/dx to decide whether draw additional line in same column.
	*/
	void tanx_line_draw(const Eigen::Vector3f& begin,
		const Eigen::Vector3f& end,
		std::function<void(const Eigen::Vector3f&, const Eigen::Vector3f&)> set_pixel);

	/*
	* directly copied from games101 homework.
	*/
	void standard_line_draw(const Eigen::Vector3f& begin,
		const Eigen::Vector3f& end,
		std::function<void(const Eigen::Vector3f&, const Eigen::Vector3f&)> set_pixel);

	
	/*
	* interpoliate properties inside an triangle
	*/
	template<typename T>
	std::tuple<float, float, float> computeBarycentric2D(float x, float y, const T v)
	{
		bool constexpr compile_type_check =
			std::is_same_v<T, std::array<Eigen::Vector4f, 3>> ||
			std::is_same_v<T, Eigen::Vector4f*> ||
			std::is_same_v<T, const Eigen::Vector4f*>;
		static_assert(compile_type_check, "v must be an array of 3 Eigen::Vector3f or it's pointer");

		float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
		float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
		float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());

		/*
		* the above ugly formual is the analytical solution of the following linear question:
		* If we use triangle vertices as the basis of a 2D plane, how to compute coordiante of the same point.
		* This coordinate system has a unique name: Barycentric.
		* more detail at tiger book 86.
		*/

		// direct solution but not fast
	 //   Matrix3f basis;
	 //   for (int i = 0; i < 2; i++) {
		//	basis.col(i) = (v[i + 1] / v[i + 1].w() - v[i] / v[i].w()).head<2>().homogeneous();
	 //   }
		//basis.col(2) = Eigen::Vector3f{ v[0].x(),v[0].y(),v[0].w() };
	 //   Vector3f bary = basis.inverse() * Vector3f(x, y, 1);
	 //   for (int i = 0; i < 3; i++)
	 //   {
	 //       bary[i] /= bary.z();
	 //   }
	 //   c1 = bary[0];
	 //   c2 = bary[1];
	 //   c3 = 1 - c1 - c2;
		return { c1,c2,c3 };
	}
}