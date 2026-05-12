#include "echoverse_time_types.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <iomanip>
#include <limits>
#include <regex>
#include <stdexcept>

#if defined(_WIN32)
#include <winsock2.h>
#else
#include <sys/time.h>
#endif

using namespace sonar_types_v2;

std::string Time::DEFAULT_FORMAT = "%Y%m%d-%H:%M:%S";

Time::Time(int64_t _microseconds) : microseconds(_microseconds) {}
Time::Time() : microseconds(0) {}

Time Time::now() {
#if defined(_WIN32)
    const auto epoch_us = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch());
    return Time(static_cast<int64_t>(epoch_us.count()));
#else
    timeval t;
    gettimeofday(&t, 0);
    return Time(static_cast<int64_t>(t.tv_sec) * UsecPerSec + t.tv_usec);
#endif
}

Time Time::monotonic() {
    static auto monotonicClock = std::chrono::steady_clock();
    auto tp = monotonicClock.now().time_since_epoch();
    auto us = std::chrono::duration_cast<std::chrono::microseconds>(tp);
    return Time(us.count());
}

bool Time::operator<(const Time& ts) const { return microseconds < ts.microseconds; }
bool Time::operator>(const Time& ts) const { return microseconds > ts.microseconds; }
bool Time::operator==(const Time& ts) const { return microseconds == ts.microseconds; }
bool Time::operator!=(const Time& ts) const { return !(*this == ts); }
bool Time::operator>=(const Time& ts) const { return !(*this < ts); }
bool Time::operator<=(const Time& ts) const { return !(*this > ts); }
Time Time::operator-(const Time& ts) const { return Time(microseconds - ts.microseconds); }
Time Time::operator+(const Time& ts) const { return Time(microseconds + ts.microseconds); }
Time Time::operator/(int divider) const { return Time(microseconds / divider); }
Time Time::operator*(double factor) const { return Time(static_cast<int64_t>(microseconds * factor)); }
bool Time::isNull() const { return microseconds == 0; }

timeval Time::toTimeval() const {
#if defined(_WIN32)
    timeval tv;
    tv.tv_sec = static_cast<long>(microseconds / UsecPerSec);
    tv.tv_usec = static_cast<long>(microseconds % UsecPerSec);
    return tv;
#else
    timeval tv = {static_cast<time_t>(microseconds / UsecPerSec), static_cast<suseconds_t>(microseconds % UsecPerSec)};
    return tv;
#endif
}

std::vector<int> Time::toTimeValues() const {
    int64_t usec = microseconds;
    int64_t days = usec / 86400000000ll;
    usec -= days * 86400000000ll;
    int64_t hours = usec / 3600000000ll;
    usec -= hours * 3600000000ll;
    int64_t minutes = usec / 60000000ll;
    usec -= minutes * 60000000ll;
    int64_t seconds = usec / 1000000ll;
    usec -= seconds * 1000000ll;
    int64_t millis = usec / 1000ll;
    usec -= millis * 1000ll;
    return {static_cast<int>(usec), static_cast<int>(millis), static_cast<int>(seconds),
            static_cast<int>(minutes), static_cast<int>(hours), static_cast<int>(days)};
}

std::string Time::toString(Time::Resolution resolution, const std::string& mainFormat) const {
    struct timeval tv = toTimeval();
    int uSecs = static_cast<int>(tv.tv_usec);
    time_t when = tv.tv_sec;
    struct tm tm_value;
#if defined(_WIN32)
    localtime_s(&tm_value, &when);
#else
    localtime_r(&when, &tm_value);
#endif

    char time[50];
    strftime(time, 50, mainFormat.c_str(), &tm_value);
    char tzInfo[6];
    strftime(tzInfo, 6, "%z", &tm_value);

    char buffer[100];
    switch (resolution) {
        case Seconds:
            std::snprintf(buffer, sizeof(buffer), "%s%s", time, tzInfo);
            break;
        case Milliseconds:
            std::snprintf(buffer, sizeof(buffer), "%s:%03d%s", time, static_cast<int>(uSecs / 1000.0), tzInfo);
            break;
        case Microseconds:
            std::snprintf(buffer, sizeof(buffer), "%s:%06d%s", time, uSecs, tzInfo);
            break;
        default:
            throw std::invalid_argument("Time::toString(): invalid resolution");
    }
    return std::string(buffer);
}

double Time::toSeconds() const { return static_cast<double>(microseconds) / UsecPerSec; }
int64_t Time::toMilliseconds() const { return microseconds / 1000; }
int64_t Time::toMicroseconds() const { return microseconds; }
Time Time::fromMicroseconds(int64_t value) { return Time(value); }
Time Time::fromMilliseconds(int64_t value) { return Time(value * 1000); }
Time Time::fromSeconds(int64_t value) { return Time(value * UsecPerSec); }
Time Time::fromSeconds(int value) { return Time(static_cast<int64_t>(value) * UsecPerSec); }
Time Time::fromSeconds(int64_t value, int usec) { return Time(value * UsecPerSec + static_cast<int64_t>(usec)); }
Time Time::fromSeconds(double value) {
    int64_t seconds = static_cast<int64_t>(value);
    return Time(seconds * UsecPerSec + static_cast<int64_t>(std::round((value - seconds) * UsecPerSec)));
}
Time Time::max() { return Time(std::numeric_limits<int64_t>::max()); }

int64_t Time::getTimezoneOffset(time_t when) {
    struct tm tm_value;
#if defined(_WIN32)
    gmtime_s(&tm_value, &when);
#else
    gmtime_r(&when, &tm_value);
#endif
    tm_value.tm_isdst = -1;
    time_t localWhen = mktime(&tm_value);
    return localWhen - when;
}

int64_t Time::tzInfoToSeconds(const std::string& tzInfo) {
    int64_t tzOffset = 0;
    int hours;
    int minutes;
#if defined(_WIN32)
    int r = ::sscanf_s(tzInfo.c_str(), "%3d%2d", &hours, &minutes);
#else
    int r = std::sscanf(tzInfo.c_str(), "%3d%2d", &hours, &minutes);
#endif
    if (r != 2 || tzInfo.size() != 5) {
        throw std::invalid_argument("Time::tzInfoToSeconds parse failed for " + tzInfo);
    }
    tzOffset = hours * 3600;
    if (tzOffset < 0) {
        tzOffset -= minutes * 60;
    } else {
        tzOffset += minutes * 60;
    }
    return -tzOffset;
}

std::ostream& sonar_types_v2::operator<<(std::ostream& io, const Time& time) {
    const int64_t microsecs = time.toMicroseconds();
    io << (microsecs / 1000000) << std::setfill('0') << "." << std::setw(3) << (std::llabs(microsecs) / 1000) % 1000
       << "." << std::setw(3) << (std::llabs(microsecs) % 1000) << std::setfill(' ');
    return io;
}
