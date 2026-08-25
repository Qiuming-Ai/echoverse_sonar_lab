#pragma once
// High-resolution stopwatch / stage timer (ms), mirroring MATLAB tic/toc.
#include <chrono>

namespace sonar {

class Timer {
public:
    Timer() { reset(); }
    void reset() { start_ = std::chrono::steady_clock::now(); }
    // Elapsed milliseconds since reset()/construction.
    double ms() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(now - start_).count();
    }
    double sec() const { return ms() / 1e3; }

private:
    std::chrono::steady_clock::time_point start_;
};

}  // namespace sonar

