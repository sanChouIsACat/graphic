#include "bSplineCurve.hpp"
#include "logger.hpp"
int computeKnotVSize(int n, int p) {
	return 2 * (p) + n;
}
void GAlgo::BSplineCurve::printBuffer(float t) {
	for each (auto& pair in buffer)
	{
		G_LOGGER_TRACE("[%f]: [%d,%d] is [%f]."
		, t,pair.first.i,pair.first.k,pair.second);
	}
}
void GAlgo::BSplineCurve::drawCurve(float sample_rate,
	const std::vector<POINT_EGDE_2D>& control_point,
	std::function<RGB(float t)> color_f)
{
	if (control_point.empty()) {
		return;
	}

	last_time_control_points = &std::remove_const_t<std::vector<POINT_EGDE_2D>>(control_point);
	std::vector<POINT_EGDE_2D>& operate_v = std::remove_const_t<std::vector<POINT_EGDE_2D>>(control_point);
	// n is associated with control points' size
	// n - p <= 2 must be satisfied
	if ((int)control_point.size() - p <= 2) {
		// use old data
		if (&control_point != last_time_control_points) {
			new_control_points.resize(p + 3);
			std::copy(control_point.begin(), control_point.end(), new_control_points.begin());
			for (int i = 0; i < new_control_points.size() - control_point.size(); i++)
			{
				new_control_points[new_control_points.size() - 1 - i] = control_point[control_point.size() - 1];
			}
		}
		
		operate_v = new_control_points;
	}
	int n = operate_v.size();
	if (computeKnotVSize(n, p) != operate_v.size()) {
		computeKnotVector(n, p);
	}

	for (float i = 0; i <= 1; i += sample_rate) {
		float x = 0;
		float y = 0;
		buffer.clear();
		for (int j = 0; j < n; j++)
		{
			float basic_function_value = coxDeBoor(i, j, p);
			x += basic_function_value * operate_v[j].x();
			y += basic_function_value * operate_v[j].y();
		}
		//G_LOGGER_TRACE("[%f:set pixel [%d,%d]", i, (int)std::round(x), (int)std::round(y));
		set_pixel_f(std::round(x), std::round(y), color_f(i));
	}
}

void GAlgo::BSplineCurve::setP(int p)
{
	this->p = p;
}

void GAlgo::BSplineCurve::computeKnotVector(int n, int p)
{
	n = n - p;
	knot_vector.resize(2 * (p + 1) + n);
	for (int i = 0; i <= p + 1; i++) {
		knot_vector[i] = 0;
	}
	float interval;
	if (n % 5 == 0 || n % 2 == 0) {
		interval = 1.0 / (n + 1);
	}
	else {
		interval = 1.0 / n;
	}
	float currentV = interval;
	for (int i = p + 1; i < n + p + 1; i++) {
		knot_vector[i] = currentV;
		currentV += interval;
	}

	for (int i = n + p + 1; i < 2 * (p + 1) + n; i++) {
		knot_vector[i] = 1;
	}
}

float GAlgo::BSplineCurve::coxDeBoor(float t, int i, int k)
{
	BaseFunctionKey key{ i, k };
	// caller guarantees that when t changes, buffer is cleared.
	auto& computed_value = buffer.find(key);
	if (computed_value != buffer.end()) return (*computed_value).second;
	float ans;
	float first_value = -1;
	float second_value = -1;
	if (k == 0) {
		if (t < knot_vector[i + 1] && t >= knot_vector[i]) ans = 1;
		else ans = 0;
	}
	else {
		first_value = coxDeBoor(t, i, k - 1);
		second_value = coxDeBoor(t, i + 1, k - 1);

		float a = knot_vector[i];
		float b = knot_vector[i + 1];
		float c = knot_vector[i + k];
		float d = knot_vector[i + k + 1];
		float ans1 = 0;
		float ans2 = 0;
		if (c - a > 1e-05) {
			float distance = (t - a) / (c - a);
			ans1 = distance * first_value;
		}
		if (d - b > 1e-05) {
			float distance = (d - t) / (d - b);
			ans2 = distance * second_value;
		}
		ans = ans1 + ans2;
	}
	buffer[key] = ans;
	/*G_LOGGER_TRACE("current computing [%d,%d] is [%f], the first value[%d,%d] is [%f], the second value[%d,%d] is [%f]."
		, i, k, ans, i, k - 1, first_value, i + 1, k - 1, second_value);*/
	return ans;
}
