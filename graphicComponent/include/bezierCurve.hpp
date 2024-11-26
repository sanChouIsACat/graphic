#pragma once
#include "explicitCurve.hpp"
namespace GAlgo {
	class BezierCurve:public ExplicatCurveI
	{
	public:
		BezierCurve(const SET_PIXEL_F& set_pixel_f) :
			ExplicatCurveI(set_pixel_f) {};
		void drawCurve(float sample_rate,
			const std::vector<POINT_EGDE_2D>& control_point,
			std::function<RGB(float t)> color_f) override;
	private:
		std::vector<long long> buffer;
		static long long combination(int n, int k);
	};
}