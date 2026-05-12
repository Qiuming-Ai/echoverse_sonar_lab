#pragma once

#include <sonar_types_v2/echoverse_sonar_types.hpp>
#include <sonar_types_v2/echoverse_math_types.hpp>
#include <cstdint>
#include <opencv2/core.hpp>

namespace standalone_mvp {

/** Delegates to Sonar::setRegularBeamBearings. */
inline void applyRegularBeamBearings(sonar_types_v2::samples::Sonar& sonar, sonar_types_v2::Angle start, sonar_types_v2::Angle interval) {
    sonar.setRegularBeamBearings(start, interval);
}

/** Delegates to Sonar::validate(). */
inline void validateSonarSample(sonar_types_v2::samples::Sonar& sonar) {
    sonar.validate();
}

/**
 * Off-screen render using Rock SonarPlot (same paint path as gui/rock_widget_collection).
 * For interactive UI, multibeam_gui embeds the full Rock SonarWidget (gain/range/palette/grid + SonarCanvas::setData).
 * OpenCV BGR output for optional cv::imshow when STANDALONE_USE_OPENCV_SONAR=1.
 * overlay_range_m drives SonarPlot distance grid labels (match UI "Range" m).
 */
cv::Mat renderSonarLikeSonarWidget(const sonar_types_v2::samples::Sonar& sonar, int plot_width, int plot_height,
                                   int overlay_range_m = 18);

/** Same as imaging_sonar_simulation MultibeamSonarTask: sonar.setRegularBeamBearings after simulate. */
void finalizeMultibeamSonarSample(sonar_types_v2::samples::Sonar& sonar, sonar_types_v2::Angle beam_width, std::uint32_t beam_count);

} // namespace standalone_mvp
