#pragma once
// Echo simulation (port of EchoInit.m + sim_rx_from_scatterers_perTX.m).
// v1: linear CPU execution, double precision, sparse-IR accumulation.
#include <string>

#include "types.h"

namespace sonar::sim {

struct EchoSimOptions {
    std::string round = "round";        // round | floor | ceil
    std::string precision = "single";   // single | double  (MATLAB default single)
    std::string atten = "none";         // none | twoway_R | sqrt_twoway_R
    std::vector<double> delay;          // per-scatterer extra delay (s); empty => 0
    // The real 3d.esl3d benchmark is faster with the direct tiled kernel.
    // FFTW remains opt-in for denser scenes via this flag or ESL_CPU_FFT_ECHO=1.
    bool use_fft = false;
    int oversample_factor = 5;         // balanced working rate: fs * 5
    bool use_legacy_direct = false;    // retained only for numerical/perf validation
    bool fuse_downsample = true;       // polyphase-fused direct kernel
};

// Reports which CPU path the most recent call used.  The environment variable
// ESL_CPU_FFT_ECHO=0 can force the direct reference without changing config.
bool last_echo_used_cpu_fft();

// Port of sim_rx_from_scatterers_perTX.m.
//   P  : [K x 3] scatterer positions (m)
//   A  : [K x 1] amplitudes
//   TX : [Nt x 3] transmit element positions
//   RX : [Nr x 3] receive element positions
//   c  : sound speed (m/s)
//   fs : sample rate (Hz)  -- the value passed to MATLAB (interp_factor*fs)
//   excitation : [Nt x M] per-element excitation (real for LFM)
// Outputs:
//   y    : [Mout x Nr] complex received signals, single precision (fs rate)
//   t0   : earliest absolute arrival delay (s)
void sim_rx_from_scatterers(const MatD& P, const MatD& A, const MatD& TX, const MatD& RX,
                            double c, double fs, const MatC& excitation,
                            const EchoSimOptions& opts, MatFC& y, double& t0);

// Compatibility surface inherited from the standalone port. EchoVerse links
// echo_cuda_disabled.cpp, so these functions are no-ops and synthesis remains
// in process on the CPU.
void set_cuda_echo_backend(bool enabled, const std::string& exe_path);
bool cuda_echo_enabled();
// Retained no-op entry points keep the ported simulator ABI self-contained.
void enable_cuda_echo_from_env(int argc, char** argv);
void enable_cuda_echo(int argc, char** argv);
// Always returns false in the embedded build.
bool echo_cuda_sim(const MatD& P, const MatD& A, const MatD& TX, const MatD& RX, double c,
                   double fs, const MatC& excitation, const EchoSimOptions& opts, MatFC& y,
                   double& t0);

// Full echo pipeline (port of EchoInit.m). Fills out.y_deci ([N x Nrx]) and
// metadata. snr_db is the configured SNR; set to NaN to skip awgn.
void echo_pipeline(const SonarConfig& s, const MatD& P, const MatD& A, EchoFrame& out);

// Echo pipeline with all intermediate stages exposed (for golden comparison /
// stage-by-stage diagnosis). echo_pipeline() is a thin wrapper of this.
struct EchoStages {
    MatFC v_sim;   // after sim_rx (interp_factor*fs rate)
    MatFC v_rs;    // after resample(1,interp) + doppler + awgn (fs rate)
    MatFC v_mix;   // after down-conversion
    MatFC y_fir;   // after FIR
    MatFC y_deci;  // after decimation
    double t0 = 0.0;
};
void echo_pipeline_stages(const SonarConfig& s, const MatD& P, const MatD& A, EchoStages& st);

}  // namespace sonar::sim
