#pragma once

#include <vector>

#include "types.h"

namespace sonar::dsp {

// True when the binary was linked with the bundled CPU-only FFTW backend.
bool fft_convolution_available();

// FFTW plan-many convolution of every input column with one filter.  The
// result follows MATLAB conv(column, filter, 'same'), including the even-tap
// offset. channel_window may be empty or contain exactly input.cols() values.
// Returns false when FFTW is unavailable or ESL_FFT_MATCH_FILTER=0 requests
// the direct reference path.
bool fft_convolve_columns_same(const MatC& input, const std::vector<cplx>& filter,
                               const std::vector<double>& channel_window, MatC& output);

}  // namespace sonar::dsp

