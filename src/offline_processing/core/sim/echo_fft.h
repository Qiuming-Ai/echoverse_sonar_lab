#pragma once

#include <string>
#include <vector>

#include "types.h"

namespace sonar::sim {

bool cpu_fft_echo_available();

// Builds the per-receiver sparse impulse responses, evaluates their linear
// convolutions with FFTW plan-many, and resamples each completed channel
// immediately so the full 10x-rate receive matrix is never materialized.
bool echo_fft_convolve(const MatD& points, const MatD& amplitudes, const MatD& tx,
                       const MatD& rx, double sound_speed, double fs_work,
                       const std::vector<std::vector<cplx>>& excitation,
                       const std::vector<double>& delays, const std::string& rounding,
                       const std::string& attenuation, int oversample_factor,
                       double t0, double tmax,
                       MatFC& output);

}  // namespace sonar::sim

