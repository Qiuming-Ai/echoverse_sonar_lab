#include "image/tvg.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sonar::image {

TvgParams tvg_factory(double t_start, const TvgOptions& opt) {
    TvgParams p;
    p.t_start = t_start;
    p.c = opt.c;
    p.R0 = opt.R0;
    p.G0_db = opt.G0_db;
    p.clamp_Rmin = opt.clamp_Rmin;

    // mode -> K (point: 40, volume: 20, intermediate: 30, custom: K)
    const std::string m = opt.mode;
    if (m == "point") {
        p.K = 40.0;
    } else if (m == "volume") {
        p.K = 20.0;
    } else if (m == "intermediate") {
        p.K = 30.0;
    } else if (m == "custom") {
        p.K = opt.K;
    } else {
        throw std::runtime_error("tvg_factory: mode must be point|volume|intermediate|custom");
    }

    // absorption: given alpha, else Thorp from freq_khz
    if (opt.alpha_db_m > 0.0) {
        p.alpha_db_m = opt.alpha_db_m;
    } else {
        if (opt.freq_khz <= 0.0)
            throw std::runtime_error(
                "tvg_factory: provide alpha_dB_m or freq_kHz (for Thorp)");
        p.alpha_db_m = thorp_alpha_db_per_m(opt.freq_khz);
    }
    return p;
}

double thorp_alpha_db_per_m(double f_khz) {
    const double f2 = f_khz * f_khz;
    const double a_db_km = 0.11 * f2 / (1.0 + f2) + 44.0 * f2 / (4100.0 + f2) +
                           2.75e-4 * f2 + 0.003;
    return a_db_km * 1e-3;
}

double tvg_db(const TvgParams& p, double tau) {
    const double t = p.t_start + tau;
    const double R = std::max(p.clamp_Rmin, p.c * t / 2.0);
    return p.K * std::log10(R / p.R0) + 2.0 * p.alpha_db_m * R + p.G0_db;
}

double tvg_gain_amp(const TvgParams& p, double tau) {
    return std::pow(10.0, tvg_db(p, tau) / 20.0);
}

double tvg_gain_pow(const TvgParams& p, double tau) {
    return std::pow(10.0, tvg_db(p, tau) / 10.0);
}

std::vector<double> tvg_gain_amp_vec(const TvgParams& p, const std::vector<double>& tau) {
    std::vector<double> out(tau.size());
    for (size_t i = 0; i < tau.size(); ++i) out[i] = tvg_gain_amp(p, tau[i]);
    return out;
}

}  // namespace sonar::image

