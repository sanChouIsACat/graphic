#pragma once
#include "Triangle.hpp"
#include "g_type_traits.hpp"
#include "interpolation.hpp"
#include "logger.hpp"
#include "types.hpp"
#include <tuple>
namespace GAlgo {
// directly copied from chatgpt
RGB generateRainbowColor(float t);

void renderCircle(
    const POINT_EGDE_2D &center, float radio,
    std::function<void(const POINT_EGDE_2D &, const RGB &)> set_pixel_f,
    std::function<RGB(const POINT_EGDE_2D &)> get_color_f);

// return sequence: min_x, min_y, max_x, max_y
std::tuple<float, float, float, float>
getRoundingBox(const GTypes::Triangle &tri);

// namespace _internal {
//	template<typename T, std::size_t N>
//	struct check_arithmetic {
//		static constexpr void check(const T& arg) {
//			static constexpr std::string_view a = "Argument";
//			static constexpr std::string_view c1 = "does not support
//subscript operations."; 			static constexpr std::string_view c2 = "'s element
//does not support arithmetic operations."; 			constexpr std::string_view error_1 =
//g_type_traits::str_join_v<a, N, c1>; 			constexpr std::string_view error_2 =
//g_type_traits::str_join_v<a, N, c2>;
//			static_assert(g_type_traits::has_subscript_operator_v<T>,
//"test1"); 			static_assert(g_type_traits::has_subscript_operator_v<T>, "some
//argument should support subscript operations"); 			using ele_type =
//g_type_traits::element_type_t<T>;
//			static_assert(g_type_traits::has_arithmetic_operations_v<ele_type>,
//				"some argument's element should support arithmetic
//operations");
//		}
//	};

//	template<typename... PTs, std::size_t... Indices>
//	constexpr int check_all_arithmetic(std::index_sequence<Indices...>,
//const PTs&... properties) { 		return 1; 		(check_arithmetic<PTs,
//Indices>::check(properties), ...);
//
//	}
//}
template <typename T>
T interploateProperties(float a, float b, float c, T ap, T bp, T cp) {
  T res = a * ap + b * bp + c * cp;
  return res;
}

template <typename... PT, typename T>
auto BarycentricProperties(float x, float y, const T &points,
                           const PT &...properties) {
  /*
   * check_all_arithmetic should give more specfical error message with index i.
   * But static_assert can only accept literal.
   * Which means error message can only be generated at `edit time`(the latest
   * phase is expantion of macro)
   */
  // constexpr int  k =
  // _internal::check_all_arithmetic(std::index_sequence_for<PT...>(),
  // properties...);
  static_assert((g_type_traits::has_subscript_operator_v<PT> && ...),
                "some argument should support subscript operations");
  static_assert((g_type_traits::has_arithmetic_operations_v<
                     g_type_traits::element_type_t<PT>> &&
                 ...),
                "some argument should support arithmetic operations");

  auto [a, b, c] = interpolation::computeBarycentric2D(x, y, points);
  return std::make_tuple(
      a, b, c,
      ((interploateProperties(a, b, c, properties[0], properties[1],
                              properties[2])))...);
}
/*
 * Suppose point d in the give plane, we should have the equation:
 * c + t * d = a + k_1 * b_1 + k_2 * b_2
 * c is the poztion of light, d is the direction of light.
 * a is the plane's origin, b_1 is the first axis, b_2 is the second axis.
 * Return: a[0]:t, a[1]:k_1, a[2]:k_2.
 * If light is parallel with plane,return -1,-1,-1
 */
Eigen::Vector3f computeLightPlaneInterSection(const GTypes::Light &light,
                                              const GTypes::Plane &plane);

/*
 * Solve plane one by one.
 */
std::tuple<bool, Eigen::Matrix<float, 3, 2>>
computeLightAABBInterSection(const GTypes::Light &light,
                             const GTypes::AABB &aabb);
} // namespace GAlgo