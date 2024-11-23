//#pragma once
//#include <vector>
//#include "types.hpp"
//#include <opencv2/opencv.hpp>
//#include <type_traits>
//#include <mutex>
//#include <unordered_set>
//#include <unordered_map>
//
//// all event parameters expect window should be packed or currying. 
//// window will be filled with framework
//// return true means exit the loop
//namespace g_curves {
//	namespace curves_internal {
//		using EventPackedFunction = std::function<bool(cv::Mat&)>;
//		template<typename... Args>
//		using EventFunction = std::function<bool(cv::Mat&, Args...)>;
//		struct EventFunctionPack {
//			
//			EventPackedFunction function;
//			std::string name;
//			EventFunctionPack(EventPackedFunction function, const std::string& name) :
//				function(function), name(name) {};
//			EventFunctionPack(EventFunctionPack&& t) noexcept{
//				function = std::move(t.function);
//				name = std::move(t.name);
//			}
//			EventFunctionPack(const EventFunctionPack& t) {
//				function = t.function;
//				name = t.name;
//			}
//		};
//
//		template<typename... Args>
//		EventFunctionPack packEvent(EventFunction<Args...> event_function, const std::string& event_name, Args... args) {
//			return EventFunctionPack{ std::bind(event_function, std::placeholders::_1, std::forward<Args>(args)...),
//				event_name };
//		}
//	}
//
//	// the first argument is contorl_points. draw function should response for computing and drawing
//	using DRAW_CURVES_ALGO = std::function<void(const std::vector<POINT_EGDE_2D>&)>;
//	// the class doesn't response for checking and manage buffer resource.
//	class Curves {
//	private:
//		static const std::unordered_map<unsigned int, curves_internal::EventFunctionPack> event_implementation_mapping;
//
//		// window properties
//		cv::Mat main_window;
//		cv::Mat replica_window;
//		const std::string main_window_name;
//		const std::string replica_window_name;
//		unsigned int width;
//		unsigned int height;
//
//		// data
//		unsigned int control_circle_radiu;
//		std::unordered_set<POINT_EGDE_2D,type_comparer::Vector3fHash,type_comparer::Vector3fEqual> control_points;
//
//		// event control
//		// when left button is down, record the coor and check when left button up
//		static constexpr unsigned int up = 0;
//		static constexpr unsigned int down = 1;
//		unsigned int mouse_state = up;
//		// indicate whether click the circle 
//		bool inside_circle = false;
//		// for delete
//		std::unordered_set<POINT_EGDE_2D>::iterator origin_point;
//		
//
//		// event_functions that will be called once a loop
//		std::queue<curves_internal::EventFunctionPack> event_loop;
//		DRAW_CURVES_ALGO draw_algo;
//
//	public:
//		Curves(unsigned int width,
//			unsigned int height,
//			unsigned int control_circle_radiu,
//			const std::string& main_window_name,
//			const std::string& replica_window_name) :
//			main_window(width, height, CV_8UC3, cv::Scalar(0)),
//			replica_window(main_window),
//			width(width),
//			height(height),
//			control_circle_radiu(control_circle_radiu),
//			main_window_name(main_window_name),
//			replica_window_name(replica_window_name)
//		{
//			auto set_window = [](const std::string& name, cv::Mat& window) {
//				cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
//				cv::namedWindow(name, cv::WINDOW_AUTOSIZE);
//				};
//			set_window(main_window_name, main_window);
//			set_window(replica_window_name, replica_window);
//		}
//
//		void draw();
//	private:
//		static void mouse_handler(int event, int x, int y, int flags, void* userdata);
//	};
//}
