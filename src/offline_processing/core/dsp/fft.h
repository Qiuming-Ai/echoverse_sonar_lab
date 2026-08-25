#pragma once
// FFT / Hilbert transform.
//  - radix-2 iterative FFT for power-of-two sizes
//  - Bluestein's algorithm for arbitrary sizes (matches MATLAB fft semantics)
//  - hilbert(x): analytic signal (port of MATLAB hilbert)
#include <complex>
#include <vector>

namespace sonar::dsp {

using cplx = std::complex<double>;

// in-place iterative radix-2 FFT. n must be power of two. inverse = false -> forward.
void fft_radix2(std::vector<cplx>& data, bool inverse);

// FFT of arbitrary length n (Bluestein when n is not a power of two).
// Input may be shorter/longer than n; only first min(size,n) samples used,
// output has length n (zero padded), matching MATLAB fft(x, n).
std::vector<cplx> fft(const std::vector<cplx>& x, int n = -1);
std::vector<cplx> ifft(const std::vector<cplx>& X, int n = -1);

// Real-input FFT convenience: returns length-n spectrum (MATLAB fft(x,n)).
std::vector<cplx> fft_real(const std::vector<double>& x, int n = -1);

// Hilbert analytic signal (MATLAB hilbert(x)).
// For real input x returns xa = x + j*H{x}. For complex input the result is
// the analytic signal as in MATLAB (only positive frequencies, DC kept,
// Nyquist kept for even length).
std::vector<cplx> hilbert(const std::vector<double>& x);

}  // namespace sonar::dsp

