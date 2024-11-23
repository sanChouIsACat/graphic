#pragma once
#include <opencv2/opencv.hpp>
#include <frameBuffer.hpp>
#include <boost/statechart/state_machine.hpp>
#include "eventLoop.hpp"
#include <string>
#include <any>

namespace GComponent {
	// when input event is keyboard, value of x and y is unidenfied.
	struct MouseKeyboardEvent : public boost::statechart::event<MouseKeyboardEvent> {
		int id;
		int x;
		int y;
		// should have using template to make sure type safe.
		// But statechart's template is too complex to add template here...
		void* gameWindow;
		MouseKeyboardEvent(int event, int x, int y, void* gameWindow = nullptr) :
			id(event),
			x(x),
			y(y),
			gameWindow(gameWindow) {};
	};
	template<typename StateMachine>
	class GameWindow: public FrameBuffer {
	public:
		using DRAR_FUNCTION = std::function<void()>;
		using DRAR_FUNCTIONS = std::vector<DRAR_FUNCTION>;
	private:
		cv::Mat window;
		StateMachine& state_machine;
		DRAR_FUNCTIONS draw_functions;
		const std::string window_name;
		EventLoop gui_loop{ 1000, 15 };
	private:
		static void onMouse(int event, int x, int y, int flags, void* userdata) {
			GameWindow* _this = static_cast<GameWindow*>(userdata);
			// onMouse function is executed in gui thread when cv::waitKey is called.
			// so process_event is serial called.
			_this->state_machine.process_event(MouseKeyboardEvent{ event, x, y, _this });
		}
		void show() {
			std::fill(frame_buf.begin(), frame_buf.end(), RGB{ 0,0,0 });
			for (int i = 0; i < draw_functions.size(); i++)
			{
				draw_functions[i]();
			}
			cv::imshow(window_name.c_str(), window);
			int key = cv::waitKey(1);
			if (key != -1) {
				state_machine.process_event(MouseKeyboardEvent{ key, -1, -1, this });
			}
			std::function<void()> show_event = std::bind(&GameWindow::show, this);
			gui_loop.add_event(show_event, "show");
		}
	public:
		GameWindow(int w,
			int h,
			const std::string window_name,StateMachine& m) :FrameBuffer(w, h),
			window_name(window_name),
			state_machine(m),
			window(w, h, CV_32FC3, frame_buf.data()) {
			cv::cvtColor(window, window, cv::COLOR_RGB2BGR);
		}

		void exitNow() {
			gui_loop.stop();
		}

		void inject_gui_event(const MouseKeyboardEvent& event) {
			// add to gui event loop to avoid data race
			EventFunction<> process_event = [this,&event]() {
				state_machine.process_event(MouseKeyboardEvent{ event.id, event.x, event.y ,this });
				};
			gui_loop.add_event(process_event, "process injected event");
		}

		template<typename... Args>
		bool inject_event(EventFunction<Args...> event_function,
			const std::string& event_name,
			Args... args){
			gui_loop.add_event(event_function, event_name, std::forward<Args>(args)...);
		}

		template<typename T>
		std::enable_if_t<std::is_same_v<std::decay_t<T>, DRAR_FUNCTIONS>>
		set_draw_functions(T&& draw_functions) {
			this->draw_functions = std::forward<T>(draw_functions);
		}

		void run() {
			show();
			state_machine.initiate();
			cv::setMouseCallback(window_name.c_str(), &GameWindow::onMouse, this);
			gui_loop.start();
			//gui_loop.add_event(&GameWindow::show, "draw picture");
		}
	};
}