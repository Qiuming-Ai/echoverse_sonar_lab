#include "BeamImageSynthesizer.hpp"

// Boost includes
#include <boost/random.hpp>

// C++ includes
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

using namespace sonar_core;
using namespace cv;

void BeamImageSynthesizer::refreshBeamColumnLut(const cv::Mat& cv_image) {
    if (sonar_beam_count == 0) {
        beam_column_spans.clear();
        return;
    }
    if (cached_last_sonar.bin_count == sonar_bin_count &&
        cached_last_sonar.beam_count == sonar_beam_count &&
        !beam_column_spans.empty()) {
        return;
    }

    beam_column_spans.clear();
    const double beamAngularStep = horizontal_beam_width.getRad() / sonar_beam_count;
    const double halfHorizontalFov = horizontal_beam_width.getRad() * 0.5;
    const double halfImageWidth = static_cast<double>(cv_image.cols) * 0.5;
    const double projectiveScale = halfImageWidth / std::tan(halfHorizontalFov);

    for (size_t beamIndex = 0; beamIndex < sonar_beam_count; ++beamIndex) {
        const int leftColumn =
            static_cast<int>(std::round(halfImageWidth + std::tan(-halfHorizontalFov + beamIndex * beamAngularStep) *
                                                             projectiveScale));
        const int rightColumn = static_cast<int>(std::round(
            halfImageWidth + std::tan(-halfHorizontalFov + (beamIndex + 1) * beamAngularStep) * projectiveScale));
        beam_column_spans.push_back(leftColumn);
        beam_column_spans.push_back(rightColumn);
    }
}

void BeamImageSynthesizer::accumulateBinEnergyFromRoi(cv::Mat& cv_image, std::vector<float>& bins) {
    std::vector<int> depthHistogram(sonar_bin_count, 0);
    float* channelData = cv_image.ptr<float>();
    for (int pixelIndex = 0; pixelIndex < cv_image.cols * cv_image.rows; ++pixelIndex) {
        const int binIndex = static_cast<int>(channelData[pixelIndex * 3 + 1] *
                                              static_cast<float>(sonar_bin_count - 1));
        depthHistogram[binIndex]++;
    }

    bins.assign(sonar_bin_count, 0.0f);
    for (int pixelIndex = 0; pixelIndex < cv_image.cols * cv_image.rows; ++pixelIndex) {
        const int binIndex = static_cast<int>(channelData[pixelIndex * 3 + 1] *
                                              static_cast<float>(sonar_bin_count - 1));
        const float weightedIntensity = (1.0f / depthHistogram[binIndex]) * logisticResponse(channelData[pixelIndex * 3]);
        bins[binIndex] += weightedIntensity;
    }
}

void BeamImageSynthesizer::applyMultiplicativeSpeckle(std::vector<float>& bins) {
    const float gaussianMean = 0.95f;
    const float gaussianStdDev = 0.30f;
    const auto timestampNow = std::chrono::high_resolution_clock::now().time_since_epoch();
    const auto seedNs = std::chrono::duration_cast<std::chrono::nanoseconds>(timestampNow);
    const std::uint32_t randomSeed = static_cast<std::uint32_t>(seedNs.count());

    boost::random::mt19937 rngEngine(randomSeed);
    boost::random::normal_distribution<float> gaussianNoise(gaussianMean, gaussianStdDev);

    const float lowerBound = 0.03f;
    for (size_t i = 0; i < bins.size(); ++i) {
        if (bins[i] < lowerBound) {
            bins[i] = lowerBound;
        }
        bins[i] *= std::fabs(gaussianNoise(rngEngine));
    }
}

float BeamImageSynthesizer::logisticResponse(float x) {
    const float slope = 18.0f;
    const float midpoint = 0.666666667f;
    const float centeredValue = (x - midpoint) * slope;
    return (0.5f * std::tanh(0.5f * centeredValue) + 0.5f);
}

float BeamImageSynthesizer::computeBinSamplingInterval(float range) {
    const float twoWayTravelTime = range * 2.0f / sound_speed_mps;
    return twoWayTravelTime / sonar_bin_count;
}

void BeamImageSynthesizer::generateBeamIntensityBins(const cv::Mat& cv_image,
                                                     std::vector<float>& bins,
                                                     bool enable_noise) {
    bins.resize(sonar_beam_count * sonar_bin_count);
    refreshBeamColumnLut(cv_image);

    std::vector<float> singleBeamBins;
    for (size_t beamIndex = 0; beamIndex < sonar_beam_count; ++beamIndex) {
        cv::Mat beamRoi = cv_image.colRange(beam_column_spans[beamIndex * 2], beam_column_spans[beamIndex * 2 + 1]).clone();
        accumulateBinEnergyFromRoi(beamRoi, singleBeamBins);
        if (enable_noise) {
            applyMultiplicativeSpeckle(singleBeamBins);
        }
        std::memcpy(&bins[sonar_bin_count * beamIndex], &singleBeamBins[0], sonar_bin_count * sizeof(float));
    }
}

sonar_types_v2::samples::Sonar BeamImageSynthesizer::buildSonarSample(const std::vector<float>& bins, float range) {
    sonar_types_v2::samples::Sonar out;
    out.time = sonar_types_v2::Time::now();
    out.bin_duration = sonar_types_v2::Time::fromSeconds(computeBinSamplingInterval(range) / 2.0);
    out.beam_width = horizontal_beam_width;
    out.beam_height = vertical_beam_height;
    out.speed_of_sound = sound_speed_mps;
    out.bin_count = sonar_bin_count;
    out.beam_count = sonar_beam_count;
    out.bins = bins;

    cached_last_sonar = out;
    return out;
}

void BeamImageSynthesizer::applyOutputGainClamp(std::vector<float>& bins, float gain) {
    const float gain_factor = 2.f * gain;
    std::transform(bins.begin(), bins.end(), bins.begin(),
                   [gain_factor](float x) { return gain_factor * x; });
    std::replace_if(bins.begin(), bins.end(), [](float x) { return x > 1.0f; }, 1.0f);
}
