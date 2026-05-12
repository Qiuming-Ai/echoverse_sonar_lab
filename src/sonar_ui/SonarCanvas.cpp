#include "SonarCanvas.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

using namespace sonar_palette;

namespace {
constexpr int kPaletteSize = 256;
constexpr int kBottomMargin = 30;
constexpr int kOverlayPaddingX = 30;
const QColor kSonarBackgroundColor(80, 80, 80);
}

SonarCanvas::SonarCanvas(QWidget *parent)
    : QFrame(parent),
      scaleX(1.0),
      scaleY(1.0),
      range(5),
      numSteps(0),
      changedSize(true),
      changedSectorScan(false),
      changedMotorStep(false),
      autoDetectMotorStep(true),
      isMultibeamSonar(true),
      continuous(true),
      enabledGrid(true) {
    motorStep.rad = 0.0;
    lastDiffStep.rad = 0.0;
    leftLimit.rad = 0.0;
    rightLimit.rad = 0.0;

    applyColormap(PALETTE_JET);
    updateOrigin();

    QPalette pal(palette());
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    pal.setColor(QPalette::Window, kSonarBackgroundColor);
#else
    pal.setColor(QPalette::Background, kSonarBackgroundColor);
#endif
    setAutoFillBackground(true);
    setPalette(pal);
}

SonarCanvas::~SonarCanvas() = default;

void SonarCanvas::setData(const sonar_types_v2::samples::Sonar& sonar) {
    if (!sonar.beam_count || !sonar.bin_count || sonar.bearings.empty() || sonar.bins.empty()) {
        return;
    }

    // Compatibility target: current GUI and plotting call paths expect fan rendering.
    isMultibeamSonar = true;

    const bool geometryChanged =
        changedSize ||
        sonar.bin_count != lastSonar.bin_count ||
        sonar.beam_count != lastSonar.beam_count ||
        lastSonar.bearings.empty() ||
        sonar.bearings.size() != lastSonar.bearings.size() ||
        !(sonar.bearings.front() == lastSonar.bearings.front());

    lastSonar = sonar;
    sonarData = sonar.bins;

    if (geometryChanged) {
        if (isMultibeamSonar) {
            generateMultibeamTransferTable(lastSonar);
        } else {
            generateScanningTransferTable(lastSonar);
        }
        changedSize = false;
    }

    if (!isMultibeamSonar) {
        if (changedSectorScan || changedMotorStep) {
            sonarData.assign(static_cast<size_t>(numSteps) * static_cast<size_t>(lastSonar.bin_count), 0.0f);
            generateScanningTransferTable(lastSonar);
            changedSectorScan = false;
            changedMotorStep = false;
        }

        if (autoDetectMotorStep && lastSonar.bearings.size() > 0 && isMotorStepChanged(lastSonar.bearings[0])) {
            sonarData.assign(static_cast<size_t>(numSteps) * static_cast<size_t>(lastSonar.bin_count), 0.0f);
            generateScanningTransferTable(lastSonar);
        }

        addScanningData(lastSonar);
    }

    update();
}

bool SonarCanvas::isMotorStepChanged(const sonar_types_v2::Angle& bearing) {
    if (lastSonar.bearings.empty()) {
        return false;
    }

    sonar_types_v2::Angle diffStep = bearing - lastSonar.bearings[0];
    diffStep.rad = std::fabs(diffStep.rad);
    if (!continuous &&
        (std::fabs((leftLimit - bearing).rad) < motorStep.rad ||
         std::fabs((rightLimit - bearing).rad) < motorStep.rad)) {
        lastDiffStep = diffStep;
        return false;
    }

    if (diffStep.rad <= 0.0) {
        lastDiffStep = diffStep;
        return false;
    }

    if (!motorStep.isApprox(diffStep) && lastDiffStep.isApprox(diffStep)) {
        motorStep = diffStep;
        numSteps = static_cast<int>((M_PI * 2.0) / motorStep.rad);
        lastDiffStep = diffStep;
        return true;
    }

    lastDiffStep = diffStep;
    return false;
}

void SonarCanvas::setMotorStep(const sonar_types_v2::Angle& step) {
    motorStep = step;
    if (motorStep.rad > 0.0) {
        numSteps = static_cast<int>((M_PI * 2.0) / motorStep.rad);
    } else {
        numSteps = 0;
    }
    changedMotorStep = true;
    autoDetectMotorStep = false;
}

void SonarCanvas::addScanningData(const sonar_types_v2::samples::Sonar& sonar) {
    if (numSteps <= 0 || sonar.bearings.empty() || sonarData.empty()) {
        return;
    }
    const int idBeam = static_cast<int>(std::round((numSteps - 1) * (sonar.bearings[0].rad + M_PI) / (2.0 * M_PI)));
    if (idBeam < 0 || idBeam >= numSteps) {
        return;
    }
    const size_t dst = static_cast<size_t>(idBeam) * static_cast<size_t>(sonar.bin_count);
    if (dst + sonar.bin_count > sonarData.size() || sonar.bin_count > sonar.bins.size()) {
        return;
    }
    std::copy_n(sonar.bins.begin(), sonar.bin_count, sonarData.begin() + static_cast<std::ptrdiff_t>(dst));
}

int SonarCanvas::sonarIndexAtPixel(int x, int y) const {
    if (x < 0 || y < 0 || x >= width() || y >= height()) {
        return -1;
    }
    const size_t idx = static_cast<size_t>(y * width() + x);
    if (idx >= transfer.size()) {
        return -1;
    }
    return transfer[idx];
}

bool SonarCanvas::hasDrawableSonar() const {
    return !transfer.empty() && !sonarData.empty();
}

void SonarCanvas::paintEvent(QPaintEvent *) {
    if (isMultibeamSonar) {
        sonarData = lastSonar.bins;
    }

    QImage img(width(), height(), QImage::Format_RGB888);
    img.fill(kSonarBackgroundColor);

    if (hasDrawableSonar()) {
        for (int y = 0; y < height(); ++y) {
            for (int x = 0; x < width(); ++x) {
                const int sonarIdx = sonarIndexAtPixel(x, y);
                if (sonarIdx < 0 || static_cast<size_t>(sonarIdx) >= sonarData.size()) {
                    continue;
                }
                int paletteIdx = static_cast<int>(std::round(sonarData[static_cast<size_t>(sonarIdx)] * 255.0f));
                paletteIdx = std::clamp(paletteIdx, 0, kPaletteSize - 1);
                const QColor color = colorMap[static_cast<size_t>(paletteIdx)];
                img.setPixel(x, y, qRgb(color.red(), color.green(), color.blue()));
            }
        }
    }

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.drawImage(0, 0, img);
    drawOverlay();
}

void SonarCanvas::updateOrigin() {
    origin.setX(width() / 2);
    origin.setY(isMultibeamSonar ? (height() - kBottomMargin) : (height() / 2));
}

void SonarCanvas::resizeEvent(QResizeEvent *event) {
    scaleX = (width() > 400) ? static_cast<double>(width()) / (BASE_WIDTH - 134) : 0.2;
    scaleY = (height() > 200) ? static_cast<double>(height() - 100) / (BASE_HEIGHT - 100) : 0.2;
    updateOrigin();
    changedSize = true;
    QWidget::resizeEvent(event);
}

void SonarCanvas::drawOverlay() {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    for (int i = 0; i < kPaletteSize; ++i) {
        painter.setPen(QPen(colorMap[static_cast<size_t>(i)]));
        painter.drawRect(width() - kOverlayPaddingX, height() - 10 - i * 2, 20, 2);
    }

    if (!enabledGrid || !lastSonar.bin_count || lastSonar.bearings.empty()) {
        return;
    }

    painter.setPen(QPen(Qt::white));
    if (isMultibeamSonar) {
        const double span_rad = (lastSonar.beam_count > 1)
            ? (lastSonar.beam_width.rad / lastSonar.beam_count) * (lastSonar.beam_count - 1)
            : lastSonar.beam_width.rad;
        const sonar_types_v2::Angle sectorSize = sonar_types_v2::Angle::fromRad(span_rad);

        for (int i = 1; i <= 5; ++i) {
            painter.drawArc(origin.x() - i * scaleX * 100, origin.y() - i * scaleY * 100,
                i * 200 * scaleX, i * 200 * scaleY,
                static_cast<int>((90 - sectorSize.getDeg() / 2) * 16),
                static_cast<int>(sectorSize.getDeg() * 16));

            const QString rangeText = QString::number(i * range * 1.0 / 5);
            const int tx = origin.x() + static_cast<int>(i * 100 * scaleX * std::sin(sectorSize.rad / 2));
            const int ty = height() - static_cast<int>(i * 100 * scaleY * std::cos(sectorSize.rad / 2));
            painter.drawText(tx, ty - 5, rangeText);

            const size_t bearingIdx = static_cast<size_t>(((lastSonar.beam_count - 1) * 1.0 / 4) * (i - 1));
            if (bearingIdx < lastSonar.bearings.size()) {
                const sonar_types_v2::Angle ang = lastSonar.bearings[bearingIdx];
                const QPoint point(
                    origin.x() + static_cast<int>(BINS_REF_SIZE * std::sin(ang.rad) * scaleX),
                    origin.y() - static_cast<int>(BINS_REF_SIZE * std::cos(ang.rad) * scaleY));
                painter.drawLine(origin, point);
                painter.drawText(point.x() - 10, point.y() - 10, QString::number(ang.getDeg(), 'f', 1));
            }
        }
    } else {
        const double offsetX = BINS_REF_SIZE * 0.75 * scaleX;
        const double offsetY = BINS_REF_SIZE * 0.55 * scaleY;
        painter.drawLine(QPoint(origin.x(), static_cast<int>(origin.y() - offsetY)),
                         QPoint(origin.x(), static_cast<int>(origin.y() + offsetY)));
        painter.drawLine(QPoint(static_cast<int>(origin.x() - offsetX), origin.y()),
                         QPoint(static_cast<int>(origin.x() + offsetX), origin.y()));

        for (int i = 1; i <= 5; ++i) {
            const int ex = static_cast<int>(i * offsetX / 5);
            const int ey = static_cast<int>(i * offsetY / 5);
            painter.drawEllipse(origin, ex, ey);
            painter.drawText(origin.x() + ex + 2, origin.y() - 5, QString::number(i * range * 1.0 / 5));
        }

        const sonar_types_v2::Angle bearing = lastSonar.bearings[0];
        const QPoint point(
            static_cast<int>(origin.x() - offsetX * std::sin(bearing.rad)),
            static_cast<int>(origin.y() - offsetY * std::cos(bearing.rad)));
        painter.setPen(QPen(Qt::green));
        painter.drawLine(origin, point);
        painter.drawText(point.x() - 10, point.y() - 10, QString::number(bearing.getDeg(), 'f', 1));

        if (!continuous) {
            const QPoint pointLeft(
                static_cast<int>(origin.x() - offsetX * std::sin(leftLimit.rad)),
                static_cast<int>(origin.y() - offsetY * std::cos(leftLimit.rad)));
            const QPoint pointRight(
                static_cast<int>(origin.x() - offsetX * std::sin(rightLimit.rad)),
                static_cast<int>(origin.y() - offsetY * std::cos(rightLimit.rad)));
            painter.drawLine(origin, pointLeft);
            painter.drawLine(origin, pointRight);
        }
    }
}

void SonarCanvas::rangeChanged(int value) {
    range = value;
}

void SonarCanvas::sonarPaletteChanged(int index) {
    applyColormap(static_cast<PaletteType>(index));
    update();
}

void SonarCanvas::gridChanged(bool value) {
    enabledGrid = value;
    update();
}

void SonarCanvas::setSectorScan(bool isContinuous, sonar_types_v2::Angle left, sonar_types_v2::Angle right) {
    if (continuous != isContinuous ||
        ((!continuous || !isContinuous) && (leftLimit.rad != left.rad || rightLimit.rad != right.rad))) {
        changedSectorScan = true;
    }
    continuous = isContinuous;
    leftLimit = left;
    rightLimit = right;
}

void SonarCanvas::applyColormap(PaletteType type) {
    heatMapGradient.colormapSelector(type);
    colorMap.clear();
    colorMap.reserve(kPaletteSize);

    try {
        float red = 0.0f;
        float green = 0.0f;
        float blue = 0.0f;
        for (int i = 0; i < kPaletteSize; ++i) {
            heatMapGradient.getColorAtValue((1.0f / (kPaletteSize - 1)) * i, red, green, blue);
            colorMap.emplace_back(
                static_cast<int>(std::clamp(red, 0.0f, 1.0f) * 255.0f),
                static_cast<int>(std::clamp(green, 0.0f, 1.0f) * 255.0f),
                static_cast<int>(std::clamp(blue, 0.0f, 1.0f) * 255.0f));
        }
    } catch (const std::out_of_range& error) {
        std::cout << error.what() << std::endl;
        colorMap.assign(kPaletteSize, kSonarBackgroundColor);
    }
}

void SonarCanvas::generateMultibeamTransferTable(const sonar_types_v2::samples::Sonar& sonar) {
    transfer.clear();
    updateOrigin();

    if (sonar.beam_count == 0 || sonar.bin_count == 0 || sonar.bearings.empty()) {
        return;
    }

    const double interval = sonar.beam_width.rad / static_cast<double>(sonar.beam_count);
    if (!(interval > 0.0) || !std::isfinite(interval)) {
        return;
    }

    const double sector_min = sonar.bearings[0].rad;
    const double sector_max = sector_min + sonar.beam_width.rad;
    const int bin_count_i = static_cast<int>(sonar.bin_count);
    transfer.reserve(static_cast<size_t>(width() * height()));

    for (int j = 0; j < height(); ++j) {
        for (int i = 0; i < width(); ++i) {
            QPointF point(i - origin.x(), j - origin.y());
            point.rx() /= scaleX * BINS_REF_SIZE / sonar.bin_count;
            point.ry() /= scaleY * BINS_REF_SIZE / sonar.bin_count;

            const double radius = std::sqrt(point.x() * point.x() + point.y() * point.y());
            const double angle = std::atan2(point.x(), -point.y());

            if (!std::isfinite(radius) || !std::isfinite(angle) ||
                angle < sector_min || angle > sector_max ||
                radius > static_cast<double>(sonar.bin_count) ||
                radius <= 0.0 || j > origin.y()) {
                transfer.push_back(-1);
                continue;
            }

            int beam_idx = static_cast<int>(std::floor((angle - sector_min) / interval));
            beam_idx = std::clamp(beam_idx, 0, static_cast<int>(sonar.beam_count) - 1);
            const int r_idx = std::clamp(static_cast<int>(std::floor(radius)), 0, bin_count_i - 1);
            transfer.push_back(beam_idx * bin_count_i + r_idx);
        }
    }
}

void SonarCanvas::generateScanningTransferTable(const sonar_types_v2::samples::Sonar& sonar) {
    transfer.clear();
    if (motorStep.rad <= 0.0 || numSteps <= 0 || sonar.bin_count == 0) {
        return;
    }

    updateOrigin();
    transfer.reserve(static_cast<size_t>(width() * height()));

    for (int j = 0; j < height(); ++j) {
        for (int i = 0; i < width(); ++i) {
            QPointF point(i - origin.x(), j - origin.y());
            point.rx() /= scaleX * BINS_REF_SIZE * 0.75 / sonar.bin_count;
            point.ry() /= scaleY * BINS_REF_SIZE * 0.55 / sonar.bin_count;
            const double radius = std::sqrt(point.x() * point.x() + point.y() * point.y());
            if (!std::isfinite(radius) || radius <= 0.0 || radius > sonar.bin_count) {
                transfer.push_back(-1);
                continue;
            }

            const double angle = std::atan2(-point.x(), -point.y());
            const int beamIdx = static_cast<int>(std::round((numSteps - 1) * (angle + M_PI) / (2.0 * M_PI)));
            const int radiusIdx = static_cast<int>(std::clamp(std::floor(radius), 0.0, static_cast<double>(sonar.bin_count - 1)));
            transfer.push_back(beamIdx * static_cast<int>(sonar.bin_count) + radiusIdx);
        }
    }
}
