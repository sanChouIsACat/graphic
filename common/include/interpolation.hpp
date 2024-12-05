#pragma once
#include "eigen3/Eigen/Eigen"
#include "logger.hpp"
#include "g_type_traits.hpp"
#include "gasserts.hpp"

namespace interpolation {
	/*
	* Both functin will return true if the point is inside the triangle.
	* Note the compartion is in 2D and triangle is projected.
	*/ 
	template<typename T>
	bool insideTriangle(float x, float y, const T& edges) {
		static_assert(g_type_traits::has_subscript_operator_v<T>, "T should be support [] operator");
		
		int res = 0;
		for (int i = 0; i < 3; i++)
		{
			G_ASSERTS_TRUE(edges[i].w() == 1, "should be point but vector");

			float ab_x = edges[i].x() - edges[(i + 1) % 3].x();
			float ab_y = edges[i].y() - edges[(i + 1) % 3].y();
			float qa_x = x - edges[i].x();
			float qa_y = y - edges[i].y();
			// ab cross qa
			float cross_res = ab_x * qa_y - ab_y * qa_x;
			G_LOGGER_TRACE("ab(%.2f,%.2f) corss qa(%.2f,%.2f) equals %f", ab_x, ab_y, qa_x, qa_y, cross_res);
			res += cross_res < 0 ? -1 : 1;
		}

		return std::abs(res) == 3;
	};

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
	std::tuple<float, float, float> computeBarycentric2D(float x, float y, const T& v)
	{
		// using new algos
		return computeBarycentric(Eigen::Vector4f{ x, y, 0.0f, 1.0f}, v);

		//old algos
		//
		// 
		// 
		//bool constexpr compile_type_check =
		//	std::is_same_v<T, std::array<Eigen::Vector4f, 3>> ||
		//	std::is_same_v<T, std::array<Eigen::Vector3f, 3>> ||
		//	std::is_array_v<T> ||
		//	std::is_same_v<T, Eigen::Vector4f*> ||
		//	std::is_same_v<T, const Eigen::Vector4f*>;
		//static_assert(compile_type_check, "v must be an array of 3 Eigen::Vector3f or it's pointer");

		//float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
		//float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
		//float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
		//return { c1,c2,c3 };

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
	}
	namespace interpolation_internel {
		template<typename Scalar, int Rows, int Cols>
		Eigen::Matrix<Scalar, Rows, Cols> makeMatrix() {
			return Eigen::Matrix<Scalar, Rows, Cols>{};
		}
	}
	/*
	* originCoor:point
	* v: a b c of the triangle
	* support arbitry dimensions.Idea:
	* 1. present point with new origin a. mark new Coor d
	* 2. solove equations:
	* [b_a, c_a]x = d
	*/
	template<typename T,typename U>
	std::tuple<float, float, float> computeBarycentric(const T& originCoor, const U& v)
	{
		using coor_demension_trait = g_type_traits::eigen_matrix_dimension_trait<T>;
		using tri_demension_trait = g_type_traits::eigen_matrix_dimension_trait<g_type_traits::element_type_t<U>>;
		using ScalarType = coor_demension_trait::scalar;
		int constexpr row_num = coor_demension_trait::row_num;
		int constexpr col_num = coor_demension_trait::col_num;

		T b_a = v[1] - v[0];
		T c_a = v[2] - v[0];
		T new_basis_coor = originCoor - v[0];
		int constexpr dimonsion_size = row_num - 1;
		using EQUALTIONS = Eigen::Matrix<coor_demension_trait::scalar, dimonsion_size, 3>;
		EQUALTIONS equations =
			interpolation_internel::makeMatrix<coor_demension_trait::scalar, dimonsion_size, 3>();
		equations.col(0) = b_a.head(dimonsion_size);
		equations.col(1) = c_a.head(dimonsion_size);
		equations.col(2) = new_basis_coor.head(dimonsion_size);
		G_LOGGER_TRACE("the equaltions:\n %s\n", EigenStructToString(equations).c_str());
		Eigen::FullPivLU<EQUALTIONS> equations_aug(equations);
		int rank = equations_aug.rank();
		if (rank == 3) {
			return { -1, -1, -1 };
		}

		auto a = equations.block(0, 0, dimonsion_size, 2);
		G_LOGGER_TRACE("the a:\n %s\n the b:\n%s\n"
			, EigenStructToString(a).c_str()
			, EigenStructToString(new_basis_coor.head(dimonsion_size)).c_str());
		Eigen::Vector2f res = a.colPivHouseholderQr().solve(new_basis_coor.head(dimonsion_size));
		G_LOGGER_TRACE("the solve:\n %s\n"
			, EigenStructToString(res).c_str());
		return { 1 - res[0] - res[1], res[0], res[1]};
	}
}