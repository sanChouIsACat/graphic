#pragma once
#include <stdio.h>
#include <ctime>

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
