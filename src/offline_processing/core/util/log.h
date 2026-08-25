#pragma once
// Simple leveled logger (stderr). Compatible with MATLAB fprintf-style output.

#include <cstdio>
#include <string>

namespace sonar {

enum class LogLevel { Debug = 0, Info = 1, Warn = 2, Error = 3 };

void set_log_level(LogLevel level);
LogLevel log_level();

void log_msg(LogLevel level, const char* fmt, ...)
#if defined(__GNUC__)
    __attribute__((format(printf, 2, 3)))
#endif
    ;

}  // namespace sonar

#define SONAR_LOG_DEBUG(...) ::sonar::log_msg(::sonar::LogLevel::Debug, __VA_ARGS__)
#define SONAR_LOG_INFO(...) ::sonar::log_msg(::sonar::LogLevel::Info, __VA_ARGS__)
#define SONAR_LOG_WARN(...) ::sonar::log_msg(::sonar::LogLevel::Warn, __VA_ARGS__)
#define SONAR_LOG_ERROR(...) ::sonar::log_msg(::sonar::LogLevel::Error, __VA_ARGS__)

