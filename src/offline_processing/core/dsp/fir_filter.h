#pragma once
// FIR filter design (Kaiser window) + FIR filtering, mirroring
// SignalFilter.m (designfilt 'lowpassfir', 'kaiserwin') as a v1 base.
//
// DIFF NOTE: MATLAB designfilt/kaiserord order estimation may differ from the
// standard Kaiser formulas implemented here. Numeric validation against the
// MATLAB-exported coefficients is planned (see docs/matlab_diff_notes.md).
#include <complex>
#include <string>
#include <vector>

namespace sonar::dsp {

// Modified Bessel function of the first kind, order 0 (I0).
double bessel_i0(double x);

// Kaiser window of length M (MATLAB kaiser(M, beta)).
std::vector<double> kaiser_window(int M, double beta);

// Kaiser lowpass FIR design.
//  Fpass : passband edge (Hz)
//  Fstop : stopband edge (Hz)
//  Apass : passband ripple (dB)
//  Astop : stopband attenuation (dB)
//  Fs    : sample rate (Hz)
// Returns FIR coefficients h (length = order+1).
std::vector<double> design_kaiser_lowpass(double Fpass, double Fstop, double Apass, double Astop,
                                          double Fs);

// One-sided lowpass design used by SignalFilter('lowpass','single').
// passband is the EFFECTIVE (already scaled) passband edge.
std::vector<double> design_lowpass_single(double passband_hz, double stopband_atten_db,
                                          double passband_ripple_db, double fs);

// Direct-form FIR filtering with same output length (MATLAB filter(b,1,x)
// with OutputSameLength=true): y[n] = sum_k h[k]*x[n-k], x[n<0] = 0.
std::vector<double> fir_filter_same(const std::vector<double>& x, const std::vector<double>& h);
std::vector<std::complex<double>> fir_filter_same(const std::vector<std::complex<double>>& x,
                                                  const std::vector<double>& h);

// Filter each column of a matrix (MATLAB filter over dim=1).
// Returns matrix of same shape.
template <typename T>
void fir_filter_columns(const std::vector<std::vector<T>>& in, const std::vector<double>& h,
                        std::vector<std::vector<T>>& out);

}  // namespace sonar::dsp

