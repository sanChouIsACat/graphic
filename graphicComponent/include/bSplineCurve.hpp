#pragma once
#include "explicitCurve.hpp"
#include <unordered_map>

namespace GAlgo {
	class BSplineCurve :public ExplicatCurveI
	{
	private:
		struct BaseFunctionKey {
			int i;
			int k;
			bool operator==(const BaseFunctionKey& other) const {
				return i == other.i && k == other.k;
			}
		};
		struct BaseFunctionKeyHash {
			std::size_t operator()(const BaseFunctionKey& key) const {
				// 使用 std::hash 组合两个整数的哈希值
				return std::hash<int>()(key.i) ^ (std::hash<int>()(key.k) << 1);
			}
		};

	public:
		BSplineCurve(const SET_PIXEL_F& set_pixel_f,int p = 0) :
			ExplicatCurveI(set_pixel_f),p(p) {};
		void setP(int p);
		void drawCurve(float sample_rate,
			const std::vector<POINT_EGDE_2D>& control_point,
			std::function<RGB(float t)> color_f) override;
	private:
		std::vector<float> knot_vector;
		std::unordered_map<BaseFunctionKey, float, BaseFunctionKeyHash> buffer;
		int p;

		void computeKnotVector(int n, int p);
		float coxDeBoor(float t, int i, int k);	
		// for debugging
		void BSplineCurve::printBuffer(float t);

		//cache it when append
		std::vector<POINT_EGDE_2D>* last_time_control_points;
		std::vector<POINT_EGDE_2D> new_control_points;
	};
}