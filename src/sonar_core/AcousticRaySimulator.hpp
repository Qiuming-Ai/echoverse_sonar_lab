#ifndef __IMAGING_SONAR_SIMULATION_HPP__
#define __IMAGING_SONAR_SIMULATION_HPP__

#include <Eigen/Core>

#include "BeamImageSynthesizer.hpp"
#include <sonar_imaging/SceneDepthComposer.hpp>
#include <sonar_imaging/OffscreenCaptureRig.hpp>
#include <sonar_types_v2/echoverse_sonar_types.hpp>
#include <sonar_types_v2/echoverse_math_types.hpp>
#include <sonar_types_v2/echoverse_frame_types.hpp>

namespace sonar_core
{

class AcousticRaySimulator
{
public:
    AcousticRaySimulator(float range_m,
                         float gain_factor,
                         uint32_t bin_count,
                         sonar_types_v2::Angle beam_width,
                         sonar_types_v2::Angle beam_height,
                         unsigned int resolution_px,
                         bool use_height_mode,
                         osg::ref_ptr<osg::Group> root,
                         uint32_t beam_count = 0);
    ~AcousticRaySimulator();

    /**
     * Generate a sonar sample response of a sonar at a given pose
     * @param pose: pose of the auv]
    */
    sonar_types_v2::samples::Sonar renderPing(const Eigen::Affine3d& sonar_pose);

    /**
    *  Process shader image in bins intensity.
    *  @param osg_image: the shader image (normal, depth and angle informations) in osg::Image format
    *  @param bins: the output simulated sonar data (all beams) in float
    */
    void ingestShaderFrame(osg::ref_ptr<osg::Image>& osg_image, std::vector<float>& bins);
    void configureCapturePipeline(unsigned int resolution_px, bool use_height_mode);

    sonar_types_v2::samples::frame::Frame getLastFrame();

    void setAttenuationCoefficient(const double frequency,
                                   const double temperature,
                                   const double depth,
                                   const double salinity,
                                   const double acidity,
                                   bool enable);

    void setSonarBinCount(uint32_t bin_count);
    uint32_t getSonarBinCount();

    void setSonarBeamCount(uint32_t beam_count);
    uint32_t getSonarBeamCount();

    void setSonarBeamWidth(sonar_types_v2::Angle beam_width);
    sonar_types_v2::Angle getSonarBeamWidth();

    void setSonarBeamHeight(sonar_types_v2::Angle beam_height);
    sonar_types_v2::Angle getSonarBeamHeight();

    void enableSpeckleNoise(bool enable);
    void enableReverb(bool enable);

    void setRange(float range);
    float getRange();

    void setGain(float gain_value);
    float getGain();

    sonar_core::BeamImageSynthesizer synthesizer;
    sonar_imaging::SceneDepthComposer depth_composer;
    sonar_imaging::OffscreenCaptureRig capture_tool;

    /**
     *  Update sonar pose according to auv pose.
     *  @param pose: pose of the auv
    */
    void updateCapturePose(const Eigen::Affine3d pose);

private:
    float sonar_gain_;
    float sonar_range_m_;
    cv::Mat last_shader_frame_cv_;
    bool speckle_noise_enabled_;
    unsigned int render_resolution_px_;
    bool render_height_mode_;
};

}

#endif
