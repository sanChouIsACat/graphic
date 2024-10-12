#pragma once
#include "logger.hpp"
#define G_ASSERTS_TRUE(condition, message, ...) do { \
    if (!(condition)) G_LOGGER_ERROR("Assert condition:["#condition "]. From human:["  message "]", __VA_ARGS__); \
} while(0)


#ifndef DEBUG_GRAPHICS
    #undef ASSERTS_TRUE
#endif // DEBUG_GRAPHICS