#pragma once
// Small helpers: local timestamp strings.
#include <ctime>
#include <string>

namespace sonar::util {

// "yyyymmdd_HHMMSS" (MATLAB datestr(now,'yyyymmdd_HHMMSS') equivalent).
inline std::string timestamp_now() {
    std::time_t now = std::time(nullptr);
    std::tm tmv{};
#if defined(_WIN32)
    localtime_s(&tmv, &now);
#else
    localtime_r(&now, &tmv);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tmv);
    return buf;
}

// Filename with timestamp: {stem}_{yyyymmdd_HHMMSS}_{suffix}
inline std::string timestamp_filename(const std::string& stem, const std::string& stamp,
                                      const std::string& suffix) {
    return stem + "_" + stamp + "_" + suffix;
}

}  // namespace sonar::util

