#pragma once
#include<functional>
#include<DrawI.hpp>
#include<types.hpp>
#include<vector>

namespace GAlgo {
	class ExplicatCurveI
	{
	protected:
		using SET_PIXEL_F = std::function<void(int, int, const RGB&)>;
	public:
		ExplicatCurveI(const SET_PIXEL_F& set_pixel_f) :set_pixel_f(set_pixel_f) {};
		virtual ~ExplicatCurveI() {};
		virtual void drawCurve(float sample_rate,
			const std::vector<POINT_EGDE_2D>& control_point,
			std::function<RGB(float t)> color_f) = 0;
	protected:
		const SET_PIXEL_F set_pixel_f;
	};
}