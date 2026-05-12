#pragma once

#include <cstdint>
#include <ostream>
#include <string>
#include <sys/types.h>
#include <vector>

struct timeval;

namespace sonar_types_v2 {
struct Time {
private:
    explicit Time(int64_t _microseconds);

public:
    int64_t microseconds;

    static const int UsecPerSec = 1000000LL;
    enum Resolution { Seconds = 1, Milliseconds = 1000, Microseconds = 1000000 };

    Time();
    static std::string DEFAULT_FORMAT;

    static Time now();
    static Time monotonic();

    bool operator<(Time const& ts) const;
    bool operator>(Time const& ts) const;
    bool operator==(Time const& ts) const;
    bool operator!=(Time const& ts) const;
    bool operator>=(Time const& ts) const;
    bool operator<=(Time const& ts) const;
    Time operator-(Time const& ts) const;
    Time operator+(Time const& ts) const;
    Time operator/(int divider) const;
    Time operator*(double factor) const;

    bool isNull() const;
    timeval toTimeval() const;
    std::vector<int> toTimeValues() const;
    std::string toString(Resolution resolution = Microseconds,
                         const std::string& mainFormat = Time::DEFAULT_FORMAT) const;

    static int64_t getTimezoneOffset(time_t when);
    static int64_t tzInfoToSeconds(const std::string& tzInfo);

    double toSeconds() const;
    int64_t toMilliseconds() const;
    int64_t toMicroseconds() const;

    static Time fromMicroseconds(int64_t value);
    static Time fromMilliseconds(int64_t value);
    static Time fromSeconds(int64_t value);
    static Time fromSeconds(int value);
    static Time fromSeconds(int64_t value, int microseconds);
    static Time fromSeconds(double value);
    static Time max();
};

std::ostream& operator<<(std::ostream& io, Time const& time);
} // namespace sonar_types_v2
