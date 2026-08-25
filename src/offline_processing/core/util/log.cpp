#include "util/log.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>

namespace sonar {

static LogLevel g_level = LogLevel::Info;

void set_log_level(LogLevel level) { g_level = level; }
LogLevel log_level() { return g_level; }

void log_msg(LogLevel level, const char* fmt, ...) {
    if (level < g_level) return;
    const char* tag = "DBG";
    switch (level) {
        case LogLevel::Debug: tag = "DBG"; break;
        case LogLevel::Info: tag = "INF"; break;
        case LogLevel::Warn: tag = "WRN"; break;
        case LogLevel::Error: tag = "ERR"; break;
    }
    // Timestamp (local time)
    std::time_t now = std::time(nullptr);
    std::tm tmv{};
#if defined(_WIN32)
    localtime_s(&tmv, &now);
#else
    localtime_r(&now, &tmv);
#endif
    char tbuf[32];
    std::strftime(tbuf, sizeof(tbuf), "%H:%M:%S", &tmv);

    char buf[4096];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    std::fprintf(stderr, "[%s %s] %s\n", tbuf, tag, buf);
    std::fflush(stderr);
}

}  // namespace sonar

