#include "image/file2image_pipeline.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include "dsp/conv.h"
#include "dsp/fft_convolution.h"
#include "dsp/window.h"
#include "image/das_beamformer.h"
#include "image/tvg.h"
#include "util/log.h"

namespace sonar::image {

namespace {

double infer_angle_step(const std::vector<double>& angles) {
    std::vector<double> diffs;
    for (size_t i = 1; i < angles.size(); ++i) {
        const double d = angles[i] - angles[i - 1];
        if (d > 1e-9) diffs.push_back(d);
    }
    if (diffs.empty()) return 0.1;
    std::sort(diffs.begin(), diffs.end());
    const double median = diffs[diffs.size() / 2];
    return std::round(median * 1e6) / 1e6;
}

std::vector<double> make_angle_axis(double start, double finish, double step) {
    if (!(step > 0.0) || finish < start)
        throw std::runtime_error("Invalid sector angle bounds");
    const int count = std::max(1, static_cast<int>(std::floor((finish - start) / step + 0.5)) + 1);
    std::vector<double> out;
    out.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        const double a = start + static_cast<double>(i) * step;
        if (a <= finish + step * 1e-6) out.push_back(a);
    }
    if (out.empty() || finish - out.back() > step * 0.5)
        out.push_back(finish);
    else
        out.back() = finish;
    return out;
}

double value_for_sector(const std::vector<double>& values, int sector, double fallback) {
    if (values.empty()) return fallback;
    return values[static_cast<size_t>(std::min(sector, static_cast<int>(values.size()) - 1))];
}

}  // namespace

std::vector<std::vector<double>> resolve_sector_angles_deg(const SonarAttributes& attrs,
                                                           int sectorCount) {
    if (sectorCount <= 0) throw std::runtime_error("sectorCount must be positive");
    if (attrs.scan_angle.empty()) throw std::runtime_error("scan_angle is empty");
    if (sectorCount == 1) return {attrs.scan_angle};

    std::vector<std::vector<double>> sectors(static_cast<size_t>(sectorCount));
    const double step = infer_angle_step(attrs.scan_angle);
    const double tol = std::max(1e-9, step * 1e-5);

    if (attrs.sector_div.size() >= static_cast<size_t>(sectorCount + 1)) {
        const bool hasCompleteScan =
            attrs.scan_angle.front() <= attrs.sector_div.front() + tol &&
            attrs.scan_angle.back() >= attrs.sector_div[static_cast<size_t>(sectorCount)] - tol;

        if (hasCompleteScan) {
            for (int s = 0; s < sectorCount; ++s) {
                const double lower = attrs.sector_div[static_cast<size_t>(s)];
                const double upper = attrs.sector_div[static_cast<size_t>(s + 1)];
                for (double a : attrs.scan_angle) {
                    const bool afterLower = (s == 0) ? (a >= lower - tol) : (a > lower + tol);
                    if (afterLower && a <= upper + tol)
                        sectors[static_cast<size_t>(s)].push_back(a);
                }
                if (sectors[static_cast<size_t>(s)].empty())
                    throw std::runtime_error("scan_angle does not cover every sector_div interval");
            }
            return sectors;
        }

        // Older MATLAB DataMakerInit files evaluate `scan_angle = angles_div{:}`
        // to the first cell only. Reconstruct the remaining regular grids from
        // sector_div and the step present in that first sector.
        const double origin = attrs.sector_div[0];
        auto snapToGrid = [&](double value) {
            return origin + std::round((value - origin) / step) * step;
        };
        for (int s = 0; s < sectorCount; ++s) {
            const double start = (s == 0)
                                     ? origin
                                     : snapToGrid(attrs.sector_div[static_cast<size_t>(s)]) + step;
            const double finish = snapToGrid(attrs.sector_div[static_cast<size_t>(s + 1)]);
            sectors[static_cast<size_t>(s)] = make_angle_axis(start, finish, step);
        }
        return sectors;
    }

    // Last-resort compatibility for files without sector_div: partition the
    // complete scan axis into contiguous, nearly equal groups.
    if (attrs.scan_angle.size() < static_cast<size_t>(sectorCount))
        throw std::runtime_error("Cannot split scan_angle: sector_div is missing");
    const size_t base = attrs.scan_angle.size() / static_cast<size_t>(sectorCount);
    const size_t rem = attrs.scan_angle.size() % static_cast<size_t>(sectorCount);
    size_t offset = 0;
    for (int s = 0; s < sectorCount; ++s) {
        const size_t count = base + (static_cast<size_t>(s) < rem ? 1 : 0);
        sectors[static_cast<size_t>(s)].assign(attrs.scan_angle.begin() + static_cast<ptrdiff_t>(offset),
                                               attrs.scan_angle.begin() + static_cast<ptrdiff_t>(offset + count));
        offset += count;
    }
    return sectors;
}

File2ImageResult process_ping_for_image(const SonarAttributes& attrs,
                                        const MatC& ping) {
    if (ping.empty()) throw std::runtime_error("Cannot image an empty ping");
    if (attrs.match_filter_data.empty())
        throw std::runtime_error("match_filter_data is empty");
    if (attrs.decimate_factor <= 0 || !(attrs.sampling_frequency > 0.0))
        throw std::runtime_error("Invalid sampling_frequency/decimate_factor");
    if (attrs.receive_array_position.rows() != ping.cols() ||
        attrs.receive_array_position.cols() < 3)
        throw std::runtime_error("receive_array_position does not match ping channels");

    const int N = ping.rows();
    const int M = ping.cols();
    const int sectorCount = attrs.match_filter_data.cols();
    if (attrs.sector_num > 1 && attrs.sector_num != sectorCount)
        throw std::runtime_error("sector_num does not match match_filter_data columns");

    const auto sectorAnglesDeg = resolve_sector_angles_deg(attrs, sectorCount);
    const std::vector<double> channelWindow = dsp::hamming(M);
    const double fs = attrs.sampling_frequency / static_cast<double>(attrs.decimate_factor);
    const bool multiSector = sectorCount > 1;

    std::vector<MatC> beamParts(static_cast<size_t>(sectorCount));
    std::vector<double> anglesCombined;
    MatD commonRanges;
    int totalAngles = 0;

    for (int s = 0; s < sectorCount; ++s) {
        const std::vector<cplx> mf = attrs.match_filter_data.column(s);
        MatC sigMf;
        const bool usedFft = dsp::fft_convolve_columns_same(ping, mf, channelWindow, sigMf);
        if (!usedFft) {
            sigMf.resize(N, M);
            for (int m = 0; m < M; ++m) {
                std::vector<cplx> col(static_cast<size_t>(N));
                for (int r = 0; r < N; ++r) col[static_cast<size_t>(r)] = ping(r, m);
                const std::vector<cplx> filtered = dsp::conv(col, mf, "same");
                for (int r = 0; r < N; ++r)
                    sigMf(r, m) = filtered[static_cast<size_t>(r)] *
                                  channelWindow[static_cast<size_t>(m)];
            }
        }

        const double f0 = value_for_sector(attrs.center_frequency, s, 0.0);
        TvgParams tvg;
        tvg.t_start = 0.0;
        tvg.c = attrs.sound_velocity;
        tvg.K = 20.0;
        tvg.G0_db = 0.0;
        tvg.alpha_db_m = thorp_alpha_db_per_m(f0 / 1e3);
        std::vector<double> tau(static_cast<size_t>(N));
        for (int r = 0; r < N; ++r) tau[static_cast<size_t>(r)] = static_cast<double>(r) / fs;
        const std::vector<double> gain = tvg_gain_amp_vec(tvg, tau);
        for (int m = 0; m < M; ++m)
            for (int r = 0; r < N; ++r) sigMf(r, m) *= gain[static_cast<size_t>(r)];

        // ImagePlot.m normalizes each CDM/FDM sector independently before
        // beamforming. Preserve the existing LFM file2image behavior.
        if (multiSector) {
            double peak = 0.0;
            for (size_t k = 0; k < sigMf.size(); ++k)
                peak = std::max(peak, std::abs(sigMf.data()[k]));
            if (peak > 0.0)
                for (size_t k = 0; k < sigMf.size(); ++k) sigMf.data()[k] /= peak;
        }

        DasCfg cfg;
        cfg.rx_xyz = attrs.receive_array_position;
        cfg.c = attrs.sound_velocity;
        // The active MATLAB multi-sector ImagePlot path uses one common
        // range origin; compensate_range_shift remains disabled there.
        const double delay = multiSector ? 0.0 : value_for_sector(attrs.sample_delay, 0, 0.0);
        cfg.t0 = delay - attrs.pulse_duration / 2.0;
        cfg.fs = fs;
        cfg.f0 = f0;
        cfg.fd_sign = 1;
        cfg.cf_enable = false;
        cfg.angles.reserve(sectorAnglesDeg[static_cast<size_t>(s)].size());
        for (double deg : sectorAnglesDeg[static_cast<size_t>(s)]) {
            const double rad = deg * M_PI / 180.0;
            cfg.angles.push_back(rad);
            anglesCombined.push_back(rad);
        }

        MatD ranges;
        das_plane_wave_id(cfg, sigMf, beamParts[static_cast<size_t>(s)], ranges);
        if (s == 0) commonRanges = std::move(ranges);
        else if (beamParts[static_cast<size_t>(s)].rows() != commonRanges.rows())
            throw std::runtime_error("Sector beam range dimensions do not match");
        totalAngles += beamParts[static_cast<size_t>(s)].cols();
        SONAR_LOG_INFO("image sector %d/%d: MF column %d (%s), angles=%d, f0=%.0f", s + 1,
                       sectorCount, s + 1, usedFft ? "FFTW batch" : "direct",
                       beamParts[static_cast<size_t>(s)].cols(), f0);
    }

    File2ImageResult out;
    out.beam.resize(commonRanges.rows(), totalAngles);
    int colOffset = 0;
    for (const MatC& part : beamParts) {
        for (int c = 0; c < part.cols(); ++c)
            for (int r = 0; r < part.rows(); ++r)
                out.beam(r, colOffset + c) = part(r, c);
        colOffset += part.cols();
    }
    out.ranges = std::move(commonRanges);
    out.angles = std::move(anglesCombined);
    return out;
}

}  // namespace sonar::image

