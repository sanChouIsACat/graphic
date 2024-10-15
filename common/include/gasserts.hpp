#pragma once
#include "logger.hpp"
#define G_ASSERTS_TRUE(condition, message, ...) do { \
    if (!(condition)) { \
        G_LOGGER_ERROR("Assert condition:["#condition "]. From human:["  message "]", __VA_ARGS__); \
        throw std::runtime_error("Assert failed"); \
    }; \
} while(0)
#define G_ASSERTS_FALSE(condition, message, ...) G_ASSERTS_TRUE(!(condition), message, __VA_ARGS__)


#ifndef DEBUG_GRAPHICS
    #undef ASSERTS_TRUE
    #undef ASSERTS_FALSE
#define G_ASSERTS_TRUE(condition, message, ...)
#define G_ASSERTS_FALSE(condition, message, ...)
#endif // DEBUG_GRAPHICS