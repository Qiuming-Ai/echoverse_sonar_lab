#pragma once
// Polyphase-free rational resampler mirroring MATLAB resample(x, P, Q).
//
// v1 implementation notes (see docs/matlab_diff_notes.md for validation):
//  - anti-alias filter: ideal lowpass (sinc) windowed with Kaiser(beta=12),
//    length L = 2*10*max(P,Q)+1, cutoff fc = 1/(2*max(P,Q)) — matches the
//    firls+kaiser filter used by MATLAB resample.
//  - p == q short-circuits to identity (candidate for validation).
//  - output length = ceil(Lx*P/Q); transients trimmed by the filter delay.
#include <complex>
#include <vector>

namespace sonar::dsp {

// Design the resample anti-aliasing filter for ratio P/Q.
std::vector<double> resample_filter(int P, int Q);

// Resample one real vector (MATLAB resample(x,P,Q)).
std::vector<double> resample(const std::vector<double>& x, int P, int Q);

// Resample one complex vector.
std::vector<std::complex<double>> resample(const std::vector<std::complex<double>>& x, int P,
                                           int Q);

// Resample each column (dim=1), like MATLAB resample applied to a matrix.
// in: [rows x cols] column-major.
void resample_columns(const std::vector<double>& in, int rows, int cols, int P, int Q,
                      std::vector<double>& out, int& out_rows);
void resample_columns(const std::vector<std::complex<double>>& in, int rows, int cols, int P,
                      int Q, std::vector<std::complex<double>>& out, int& out_rows);

}  // namespace sonar::dsp

