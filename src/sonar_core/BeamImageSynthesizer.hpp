#ifndef SONAR_CORE_BEAM_IMAGE_SYNTHESIZER_HPP_
#define SONAR_CORE_BEAM_IMAGE_SYNTHESIZER_HPP_

// C++ includes
#include <cstdint>
#include <vector>

// Rock includes
#include <sonar_types_v2/echoverse_sonar_types.hpp>
#include <sonar_types_v2/echoverse_pose_types.hpp>
#include <sonar_types_v2/echoverse_math_types.hpp>

// Opencv includes
#include <opencv2/core/core.hpp>

namespace sonar_core {

class BeamImageSynthesizer {

public:
    /** Number of range bins per beam */
    uint32_t sonar_bin_count;

    /** Number of azimuth beams in a ping */
    uint32_t sonar_beam_count;

    /** Horizontal sector opening (orthogonal to sensor Z axis) */
    sonar_types_v2::Angle horizontal_beam_width;

    /** Vertical sector opening (along sensor Z axis) */
    sonar_types_v2::Angle vertical_beam_height;

    /** Acoustic propagation speed in water (m/s) */
    float sound_speed_mps;

    /** Packed min/max ROI columns for each beam: [min0,max0,min1,max1,...] */
    std::vector<int> beam_column_spans;

    /** Cache of the last synthesized message (used for LUT invalidation) */
    sonar_types_v2::samples::Sonar cached_last_sonar;

    BeamImageSynthesizer()
	    : sonar_bin_count(500)
	    , sonar_beam_count(0)
	    , horizontal_beam_width(sonar_types_v2::Angle::fromRad(0.0))
	    , vertical_beam_height(sonar_types_v2::Angle::fromRad(0.0))
	    , sound_speed_mps(sonar_types_v2::samples::Sonar::getSpeedOfSoundInWater())
	    , beam_column_spans()
	    , cached_last_sonar()
    {}

    BeamImageSynthesizer(uint32_t bin_count, uint32_t beam_count, sonar_types_v2::Angle beam_width, sonar_types_v2::Angle beam_height)
	    : sonar_bin_count(bin_count)
	    , sonar_beam_count(beam_count)
	    , horizontal_beam_width(beam_width)
	    , vertical_beam_height(beam_height)
	    , sound_speed_mps(sonar_types_v2::samples::Sonar::getSpeedOfSoundInWater())
	    , beam_column_spans()
	    , cached_last_sonar()
    {}

    /**
    *  Split the shader image in beam parts. The shader is not radially spaced equally
    *  over the FOV-X degree sector, so it is needed to identify which column is contained
    *  on each beam.
    *  @param cv_image: the shader image (normal, depth and angle informations) in float
    *  @param bins: the output simulated sonar data (all beams) in float
    *  @param enable_noise: enable/disable speckle noise in acoustic image
    */
    void generateBeamIntensityBins(const cv::Mat& cv_image, std::vector<float>& bins, bool enable_noise);

    /**
    *  Encapsulate the simulated sonar data in the Rock's sonar datatype.
    *  @param bins: the simulated sonar data in float
    *  @param range: the maximum coveraged area in meters
    *  @return the simulated sonar in the Rock's structure
    */
    sonar_types_v2::samples::Sonar buildSonarSample(const std::vector<float>& bins, float range);

    /**
    *  Apply an additional gain in the simulated sonar data.
    *  @param bins: the simulated sonar data in float
    *  @param gain: the additional gain percent (0.0 - 1.0)
    */
    void applyOutputGainClamp(std::vector<float>& bins, float gain);

private:
    void refreshBeamColumnLut(const cv::Mat& cv_image);

    /**
    *  Convert the shader image (normal and depth) in bins intensity (one beam).
    *  @param cv_image: the shader image (normal and depth informations) in float
    *  @param bins: the output simulated sonar data (one beam) in float
    */
    void accumulateBinEnergyFromRoi(cv::Mat& cv_image, std::vector<float>& bins);

    /**
    *  Speckle is a granular 'noise' that inherently exists in and degrades the quality
    *  of underwater imaging sonars. The multiplicative component follows a non-uniform
    *  Gaussian distribution; and the additive component is formed by a Gaussian noise
    *  with 0 mean and Q variance. This function applies the speckle noise to
    *  simulated sonar image.
    *  @param bins: the simulated sonar data in float
    */
    void applyMultiplicativeSpeckle(std::vector<float>& bins);

    /**
    *  Accept the input value x then returns it's sigmoid value in float.
    *  @param x: the input value in float
    *  @return the sigmoid value in float
    */
    float logisticResponse(float x);

    /**
    *  Calculate the sample time period that is applied to the received sonar echo signal.
    *  @param range: the range (meters) in float
    *  @return the sampling interval
    */
    float computeBinSamplingInterval(float range);
};
} // end namespace sonar_core

#endif
