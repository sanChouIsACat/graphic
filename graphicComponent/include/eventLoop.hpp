#pragma once
#include <functional>
#include "boost/circular_buffer.hpp"
#include <string>
#include <mutex>
#include <thread>
#include "logger.hpp"

namespace GComponent {
	template<typename... Args>
	using EventFunction = std::function<void(Args...)>;
	namespace eventLoopInternal {
		struct EventFunctionPack {
			using EventPackedFunction = std::function<void()>;
			EventPackedFunction function;
			std::string name;
			EventFunctionPack(EventPackedFunction function, const std::string& name) :
				function(function), name(name) {};
			EventFunctionPack(EventFunctionPack&& t) noexcept {
				function = std::move(t.function);
				name = std::move(t.name);
			}
			EventFunctionPack(EventFunctionPack& t) {
				function = t.function;
				name = t.name;
			}

			EventFunctionPack& operator=(EventFunctionPack&& t) noexcept {
				function = std::move(t.function);
				name = std::move(t.name);
				return *this;
			}

			EventFunctionPack& operator=(EventFunctionPack& t) {
				function = t.function;
				name = t.name;
				return *this;
			}
		};

		template<typename... Args>
		EventFunctionPack packEvent(EventFunction<Args...> event_function, const std::string& event_name, Args... args) {
			return EventFunctionPack{ std::bind(event_function, std::placeholders::_1, std::forward<Args>(args)...),
				event_name };
		}
	}
	class EventLoop {
	private:
		std::recursive_mutex mutex;
		boost::circular_buffer<eventLoopInternal::EventFunctionPack> ring_buffer;
		bool stop_condition = false;
		/*
		* |**** event **** |**** event **** |**** event **** |**** event **** |
		* | loop_interval  | loop_interval  | loop_interval  | loop_interval  |
		*
		* that is if the event execution time is less than loop_interval, loop will wait a few time.
		* But if the event exection time is more than loop_interval, loop will do nothing
		*/
		unsigned int loop_interval;
		long long last_loop_timestamp = -1;

	private:
		long long getCurrentTimeStamp() {
			auto now = std::chrono::system_clock::now(); // 获取当前时间点
			auto duration = now.time_since_epoch(); // 获取从 epoch 开始的时间
			return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
			
		}

	public:
		EventLoop(const unsigned int buffer_count, const unsigned int loop_interval) :
			ring_buffer(buffer_count),loop_interval(loop_interval) {};

		template<typename... Args>
		bool add_event(EventFunction<Args...> event_function, const std::string& event_name, Args... args) {
			mutex.lock();
			if (ring_buffer.full()) {
				mutex.unlock();
				return false;
			}
			//G_LOGGER_INFO("push event:%s", event_name.c_str());
			ring_buffer.push_back(
				eventLoopInternal::EventFunctionPack{
				std::bind(event_function,
					std::forward<Args>(args)...
					),
				event_name }
			);
			mutex.unlock();
			return true;
		}
		void start() {
			last_loop_timestamp = getCurrentTimeStamp();
			while (!stop_condition) {
				long long current_time = getCurrentTimeStamp();
				long long time_diff = current_time - last_loop_timestamp;
				int p = 0;
				while(time_diff < loop_interval) {
					if (!p) {
						current_time = getCurrentTimeStamp();
						time_diff = current_time - last_loop_timestamp;
					}
					p = (p + 1) % 1000;
				}
				last_loop_timestamp = current_time;
				mutex.lock();
				if (ring_buffer.empty()) {
					mutex.unlock();
					continue;
				}
				eventLoopInternal::EventFunctionPack& buffer = ring_buffer.front();
				//G_LOGGER_INFO("comsume event:%s", buffer.name.c_str());
				buffer.function();
				ring_buffer.pop_front();
				mutex.unlock();
			}
		}
		
		void stop() {
			stop_condition = true;
		}

		bool isStop() {
			return stop_condition;
		}
	};
}
