#include <gtest/gtest.h>
#include "eventLoop.hpp"
#include <thread>
using namespace GComponent;
std::thread startLoop(EventLoop& loop) {
	return std::thread {&EventLoop::start,&loop };
}
TEST(eventLoopTest, pushEventToFullLoop) {
	EventLoop event_loop{ 1,20 };
	std::function<void()> do_nothing = []() {};
	ASSERT_TRUE(event_loop.add_event(do_nothing, "something"));
	ASSERT_FALSE(event_loop.add_event(do_nothing, "something"));
}

TEST(eventLoopTest, stopAndSequenceTest) {
	EventLoop event_loop{ 20,10 };
	bool stop_event_data = false;
	bool nothing_event_data = false;
	std::function<void()> stop_event = [&event_loop, &stop_event_data, &nothing_event_data]() {
		if (nothing_event_data) {
			stop_event_data = true;
		}
		event_loop.stop();
	};
	std::function<void()> nothing_event = [&event_loop, &nothing_event_data]() {
		nothing_event_data = true;
	};

	auto thread = startLoop(event_loop);

	ASSERT_TRUE(event_loop.add_event(nothing_event, "something"));
	ASSERT_TRUE(event_loop.add_event(stop_event, "stop"));

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	ASSERT_TRUE(stop_event_data);
	ASSERT_TRUE(nothing_event_data);
	ASSERT_TRUE(event_loop.isStop());

	thread.join();
}

TEST(eventLoopTest, feqTest) {
	EventLoop event_loop{ 20,10 };
	int event_data = 0;
	std::function<void()> nothing_event = [&nothing_event, &event_loop, &event_data]() {
		event_data += 1;
		event_loop.add_event(nothing_event, "whatever");
	};

	auto thread = startLoop(event_loop);

	ASSERT_TRUE(event_loop.add_event(nothing_event, "something"));

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	event_loop.stop();
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	GTEST_LOG_(INFO) << "event data:" << event_data;
	ASSERT_TRUE(event_data <=105 && event_data >= 95);
	ASSERT_TRUE(event_loop.isStop());

	thread.join();
}