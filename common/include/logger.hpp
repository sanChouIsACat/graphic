#pragma once
#include <stdio.h>
#include <ctime>
#include <sstream>
#include <iostream>
#include <type_traits>

// use to check type T whether supports std::ostream << T
template <typename T, typename = void>
struct has_ostream_operator : std::false_type {};

/*
* std::declval<T>() is a prvalue of type T, used to simulate the creation of an object of type T without the need to construct it.
* It is an hole in the type system.
* decltype keyword can get the type of an expression.
* so decltype(std::declval<std::ostringstream&>() << std::declval<T>(), void()) is splited the folowing statement:
* 1. std::declval<std::ostringstream&>() << std::declval<T>() equals expression ` a << b`, 
*    where var a has type std::ostringstream, var b has type T
* 2. `std::declval<std::ostringstream&>() << std::declval<T>(), void()` ends with type void.
*     Without void(), when T has multiple overrided << operator, the compiler won't know which to choose, then throw an error.
* 3. When T does not support std::ostream << T, substitution will fail, has_ostream_operator<T> is subclass of false_type.
*    When T support std::ostream << T, substitution will fail, has_ostream_operator<T> is subclass of true_tpye. This is called
*    SFINAE.
* 
*/ 
template <typename T>
struct has_ostream_operator<T, decltype(std::declval<std::ostringstream&>() << std::declval<T>(), void())> : std::true_type {};

template<typename T>
std::string EigenStructToString(const T& obj)
{
	std::ostringstream oss;
	static_assert(has_ostream_operator<T>::value, "T does not support std::ostream << T");
	oss << obj;
	return oss.str();
}

#define G_LOGGER(color_left, level, format, ...) do { \
    std::time_t now = std::time(nullptr); \
    std::tm* localTime = std::localtime(&now); \
    char timeStr[9]; \
    std::strftime(timeStr, sizeof(timeStr), "%H:%M:%S", localTime); \
    std::printf(color_left "[%s %s] [%s:%d]\033[0m: " format "\n", level, timeStr, __FILE__, __LINE__, __VA_ARGS__); \
} while(0)

#define G_LOGGER_ERROR(format, ...) G_LOGGER("\033[31m", "ERROR", format, __VA_ARGS__)
#define G_LOGGER_WARN(format, ...) G_LOGGER("\033[33m", "WARN", format, __VA_ARGS__)
#define G_LOGGER_INFO(format, ...) G_LOGGER("\033[32m", "INFO", format, __VA_ARGS__)
#define G_LOGGER_DEBUG(format, ...) G_LOGGER("\033[34m", "DEBUG", format, __VA_ARGS__)
#define G_LOGGER_TRACE(format, ...) G_LOGGER("\033[35m", "TRACE", format, __VA_ARGS__)

#ifndef DEBUG_GRAPHICS
#undef G_LOGGER_DEBUG
#define G_LOGGER_DEBUG(format, ...)
#endif // !DEBUG_GRAPHICS

#ifndef TRACE_GRAPHICS
#undef G_LOGGER_TRACE
#define G_LOGGER_TRACE(format, ...)
#endif // !TRACE_GRAPHICS
