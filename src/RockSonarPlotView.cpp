#include "RockSonarPlotView.hpp"

#include "SonarCanvas.hpp"

#include <QApplication>
#include <QEventLoop>
#include <QImage>
#include <QPixmap>
#include <QtGlobal>

#include <opencv2/imgproc.hpp>

#include <memory>

namespace standalone_mvp {

void finalizeMultibeamSonarSample(sonar_types_v2::samples::Sonar& sonar, sonar_types_v2::Angle beam_width,
                                  [[maybe_unused]] std::uint32_t beam_count) {
    const std::uint32_t n = sonar.beam_count;
    if (n == 0) {
        return;
    }
    const sonar_types_v2::Angle interval = sonar_types_v2::Angle::fromRad(beam_width.getRad() / static_cast<double>(n));
    const sonar_types_v2::Angle start = sonar_types_v2::Angle::fromRad(-beam_width.getRad() / 2.0);
    sonar.setRegularBeamBearings(start, interval);
}

namespace {

struct QtAppOnce {
    int argc = 1;
    char name_storage[24] = "standalone_mvp";
    char* argv_storage[2] = {name_storage, nullptr};
    std::unique_ptr<QApplication> app;

    QtAppOnce() {
        if (!QApplication::instance()) {
            app = std::make_unique<QApplication>(argc, argv_storage);
        }
    }
};

/** Rock SonarPlot defaults to JET; Rock UI selects Hot — match that. */
class SonarCanvasHot final : public SonarCanvas {
public:
    explicit SonarCanvasHot(QWidget* parent = nullptr) : SonarCanvas(parent) {
        applyColormap(PALETTE_HOT);
    }

    void setOverlayRangeMeters(int meters) {
        range = meters;
    }
};

} // namespace

cv::Mat renderSonarLikeSonarWidget(const sonar_types_v2::samples::Sonar& sonar, int plot_width, int plot_height,
                                     int overlay_range_m) {
    if (!sonar.beam_count || !sonar.bin_count ||
        sonar.bins.size() < static_cast<std::size_t>(sonar.beam_count) * sonar.bin_count) {
        return {};
    }

    static QtAppOnce qt_app;

    SonarCanvasHot plot(nullptr);
    plot.setOverlayRangeMeters(std::max(1, overlay_range_m));
    plot.resize(plot_width, plot_height);
    plot.setData(sonar);
    plot.update();
    QApplication::processEvents(QEventLoop::AllEvents, 100);

    const QPixmap pm = plot.grab();
    if (pm.isNull()) {
        return {};
    }

    QImage qi = pm.toImage();
    if (qi.format() != QImage::Format_RGB888) {
        qi = qi.convertToFormat(QImage::Format_RGB888);
    }

    cv::Mat rgb(qi.height(), qi.width(), CV_8UC3, const_cast<unsigned char*>(qi.constBits()),
                static_cast<std::size_t>(qi.bytesPerLine()));
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    return bgr.clone();
}

} // namespace standalone_mvp
