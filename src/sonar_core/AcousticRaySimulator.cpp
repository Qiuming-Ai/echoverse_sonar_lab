#include "AcousticRaySimulator.hpp"

#include <sonar_core/SignalProcessingUtils.hpp>
#include <cmath>

using namespace sonar_core;

AcousticRaySimulator::AcousticRaySimulator(float range_m,
                                           float gain_factor,
                                           uint32_t bin_count,
                                           sonar_types_v2::Angle beam_width,
                                           sonar_types_v2::Angle beam_height,
                                           unsigned int resolution_px,
                                           bool use_height_mode,
                                           osg::ref_ptr<osg::Group> root,
                                           uint32_t beam_count)
    : synthesizer(bin_count, beam_count, beam_width, beam_height),
      sonar_gain_(gain_factor),
      sonar_range_m_(range_m),
      speckle_noise_enabled_(false),
      render_resolution_px_(resolution_px),
      render_height_mode_(use_height_mode) {
    depth_composer = sonar_imaging::SceneDepthComposer(range_m);
    if (root.valid()) {
        depth_composer.attachSceneNode(root);
    }
    configureCapturePipeline(resolution_px, use_height_mode);
}

AcousticRaySimulator::~AcousticRaySimulator() = default;

void AcousticRaySimulator::ingestShaderFrame(osg::ref_ptr<osg::Image>& osg_image,
                                    std::vector<float>& bins) {
    cv::Mat cv_image;
    sonar_core::convertOsgImageToCvMat(osg_image, cv_image);
    synthesizer.generateBeamIntensityBins(cv_image, bins, speckle_noise_enabled_);
    last_shader_frame_cv_ = cv_image;
    synthesizer.applyOutputGainClamp(bins, sonar_gain_);
}

sonar_types_v2::samples::frame::Frame AcousticRaySimulator::getLastFrame() {
    sonar_types_v2::samples::frame::Frame frame;
    if (last_shader_frame_cv_.empty()) {
        return frame;
    }

    cv::Mat frame_cv;
    last_shader_frame_cv_.convertTo(frame_cv, CV_8UC3, 255.0);
    cv::flip(frame_cv, frame_cv, 0);
    frame.size = sonar_types_v2::samples::frame::frame_size_t(static_cast<uint16_t>(frame_cv.cols),
                                                               static_cast<uint16_t>(frame_cv.rows));
    frame.data_depth = static_cast<uint32_t>(frame_cv.elemSize1() * 8);
    frame.pixel_size = static_cast<uint32_t>(frame_cv.elemSize());
    frame.row_size = static_cast<uint32_t>(frame_cv.step[0]);
    frame.frame_mode = (frame_cv.channels() == 1) ? sonar_types_v2::samples::frame::MODE_GRAYSCALE
                                                   : sonar_types_v2::samples::frame::MODE_BGR;
    frame.frame_status = sonar_types_v2::samples::frame::STATUS_VALID;
    const size_t bytes = static_cast<size_t>(frame_cv.rows) * frame_cv.step[0];
    frame.image.resize(bytes);
    if (bytes > 0) {
        if (frame_cv.isContinuous()) {
            std::copy(frame_cv.data, frame_cv.data + bytes, frame.image.begin());
        } else {
            const size_t row_bytes = static_cast<size_t>(frame_cv.cols) * frame_cv.elemSize();
            for (int r = 0; r < frame_cv.rows; ++r) {
                const uint8_t* row_ptr = frame_cv.ptr<uint8_t>(r);
                std::copy(row_ptr, row_ptr + row_bytes, frame.image.begin() + static_cast<size_t>(r) * frame_cv.step[0]);
            }
        }
    }
    return frame;
}

sonar_types_v2::samples::Sonar AcousticRaySimulator::renderPing(const Eigen::Affine3d& sonar_pose) {
    updateCapturePose(sonar_pose);
    osg::ref_ptr<osg::Image> osg_image = capture_tool.captureNodeImage(depth_composer.depthComposerNode());
    std::vector<float> bins;
    ingestShaderFrame(osg_image, bins);
    return synthesizer.buildSonarSample(bins, sonar_range_m_);
}

void AcousticRaySimulator::updateCapturePose(const Eigen::Affine3d pose) {
    static const osg::Matrixd rock_coordinate_matrix =
          osg::Matrixd::rotate(M_PI_2, osg::Vec3(0, 0, 1))
        * osg::Matrixd::rotate(-M_PI_2, osg::Vec3(1, 0, 0));

    osg::Matrixd matrix(pose.data());
    matrix.invert(matrix);
    const osg::Matrixd transformed = matrix * rock_coordinate_matrix;

    osg::Vec3 eye;
    osg::Vec3 center;
    osg::Vec3 up;
    transformed.getLookAt(eye, center, up);
    capture_tool.setViewPose(eye, center, up);
}

void AcousticRaySimulator::configureCapturePipeline(unsigned int shaderResolution, bool useHeightMode) {
    capture_tool = sonar_imaging::OffscreenCaptureRig(
        synthesizer.vertical_beam_height.getRad(),
        synthesizer.horizontal_beam_width.getRad(),
        shaderResolution,
        useHeightMode);
    capture_tool.setBackgroundColor(osg::Vec4d(0.0, 0.0, 0.0, 1.0));
}

void AcousticRaySimulator::setAttenuationCoefficient(const double frequency,
                                                const double temperature,
                                                const double depth,
                                                const double salinity,
                                                const double acidity,
                                                bool enable) {
    double attenuation_coeff = 0.0;
    if (enable) {
        attenuation_coeff = computeAbsorptionCoefficient(frequency, temperature, depth, salinity, acidity);
    }
    depth_composer.setAttenuationCoefficient(attenuation_coeff);
}

void AcousticRaySimulator::setSonarBinCount(uint32_t bin_count) {
    synthesizer.sonar_bin_count = bin_count;
}

uint32_t AcousticRaySimulator::getSonarBinCount() {
    return synthesizer.sonar_bin_count;
}

void AcousticRaySimulator::setSonarBeamCount(uint32_t beam_count) {
    synthesizer.sonar_beam_count = beam_count;
}

uint32_t AcousticRaySimulator::getSonarBeamCount() {
    return synthesizer.sonar_beam_count;
}

void AcousticRaySimulator::setSonarBeamWidth(sonar_types_v2::Angle beam_width) {
    synthesizer.horizontal_beam_width = beam_width;
    configureCapturePipeline(render_resolution_px_, render_height_mode_);
}

sonar_types_v2::Angle AcousticRaySimulator::getSonarBeamWidth() {
    return synthesizer.horizontal_beam_width;
}

void AcousticRaySimulator::setSonarBeamHeight(sonar_types_v2::Angle beam_height) {
    synthesizer.vertical_beam_height = beam_height;
    configureCapturePipeline(render_resolution_px_, render_height_mode_);
}

sonar_types_v2::Angle AcousticRaySimulator::getSonarBeamHeight() {
    return synthesizer.vertical_beam_height;
}

void AcousticRaySimulator::setGain(float gain_value) {
    sonar_gain_ = gain_value;
}

float AcousticRaySimulator::getGain() {
    return sonar_gain_;
}

void AcousticRaySimulator::setRange(float range_value) {
    depth_composer.setMaxRange(range_value);
    sonar_range_m_ = range_value;
}

float AcousticRaySimulator::getRange() {
    return sonar_range_m_;
}

void AcousticRaySimulator::enableSpeckleNoise(bool enable) {
    speckle_noise_enabled_ = enable;
}

void AcousticRaySimulator::enableReverb(bool enable) {
    depth_composer.setDrawReverb(enable);
}
