#pragma once
// TVG (port of TVG/tvg_factory.m). Pure math, no MATLAB dependency.
#include <string>
#include <vector>

namespace sonar::image {

struct TvgParams {
    double t_start = 0.0;   // s
    double c = 1500.0;      // sound speed
    double K = 20.0;        // log slope
    double alpha_db_m = 0.0;
    double R0 = 1.0;
    double G0_db = 0.0;
    double clamp_Rmin = 0.1;
};

// Options accepted by tvg_factory (mirrors tvg_factory.m name-value args).
struct TvgOptions {
    double c = 1500.0;
    std::string mode = "volume";  // point | volume | intermediate | custom
    double K = 40.0;              // used only when mode == "custom"
    double alpha_db_m = 0.0;      // if <= 0 and freq_khz > 0 -> Thorp
    double freq_khz = 0.0;        // Thorp absorption frequency (kHz)
    double R0 = 1.0;
    double G0_db = 0.0;
    double clamp_Rmin = 0.1;
};

// Port of tvg_factory(t_start, ...): maps mode -> K, resolves alpha via
// Thorp when not given, returns a TvgParams ready for evaluation.
TvgParams tvg_factory(double t_start, const TvgOptions& opt);

// Thorp absorption (dB/m) at frequency in kHz.
double thorp_alpha_db_per_m(double f_khz);

// GdB(tau) = K*log10(R/R0) + 2*alpha*R + G0,  R = max(Rmin, c*(t_start+tau)/2)
double tvg_db(const TvgParams& p, double tau);

// Amplitude-domain gain factor Gamp(tau) = 10^(GdB/20).
double tvg_gain_amp(const TvgParams& p, double tau);

// Power-domain gain factor Gpow(tau) = 10^(GdB/10).
double tvg_gain_pow(const TvgParams& p, double tau);

// Vectorized Gamp over a tau axis (MATLAB: tvg.Gamp(tau)').
std::vector<double> tvg_gain_amp_vec(const TvgParams& p, const std::vector<double>& tau);

}  // namespace sonar::image

