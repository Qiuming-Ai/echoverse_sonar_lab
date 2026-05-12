#include "SideScanWaterfallCanvas.hpp"

#include <QPaintEvent>
#include <QPainter>
#include <QPalette>
#include <QResizeEvent>
#include <QtGlobal>

#include <algorithm>
#include <vector>
#include <cmath>
#include <cstring>

#include <sonar_palette/PaletteTypes.hpp>

using sonar_palette::PaletteType;
namespace {
const QColor kSonarBackgroundColor(80, 80, 80);
}

SideScanWaterfallCanvas::SideScanWaterfallCanvas(QWidget* parent) : QFrame(parent)
{
    applyColormap(sonar_palette::PALETTE_JET);
    setMinimumSize(400, 300);
    setFrameShape(QFrame::NoFrame);
    QPalette Pal = palette();
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    Pal.setColor(QPalette::Window, kSonarBackgroundColor);
#else
    Pal.setColor(QPalette::Background, kSonarBackgroundColor);
#endif
    setAutoFillBackground(true);
    setPalette(Pal);
}

void SideScanWaterfallCanvas::applyColormap(PaletteType type)
{
    heatMapGradient_.colormapSelector(type);
    colorMap_.clear();
    colorMap_.reserve(256);
    for (int i = 0; i < 256; ++i) {
        float r = 0.0f;
        float g = 0.0f;
        float b = 0.0f;
        heatMapGradient_.getColorAtValue((1.0f / 255.0f) * static_cast<float>(i), r, g, b);
        colorMap_.push_back(
            QColor(static_cast<int>(r * 255.0f), static_cast<int>(g * 255.0f), static_cast<int>(b * 255.0f)));
    }
}

void SideScanWaterfallCanvas::rangeChanged(int value_m)
{
    rangeM_ = std::max(1, value_m);
    update();
}

void SideScanWaterfallCanvas::sonarPaletteChanged(int index)
{
    applyColormap(static_cast<PaletteType>(index));
    update();
}

void SideScanWaterfallCanvas::gridChanged(bool value)
{
    enabledGrid_ = value;
    update();
}

static bool extractBeam0Bins(const sonar_types_v2::samples::Sonar& s, std::vector<float>& out)
{
    if (!s.bin_count || !s.beam_count) {
        return false;
    }
    const size_t need = static_cast<size_t>(s.beam_count) * static_cast<size_t>(s.bin_count);
    if (s.bins.size() < need) {
        return false;
    }
    out.resize(s.bin_count);
    for (unsigned i = 0; i < s.bin_count; ++i) {
        out[i] = s.bins[i];
    }
    return true;
}

void SideScanWaterfallCanvas::setPortStarboardPing(const sonar_types_v2::samples::Sonar& port, const sonar_types_v2::samples::Sonar& starboard)
{
    std::vector<float> pb;
    std::vector<float> sb;
    if (!extractBeam0Bins(port, pb) || !extractBeam0Bins(starboard, sb)) {
        return;
    }
    const unsigned n = static_cast<unsigned>(std::min(pb.size(), sb.size()));
    if (n == 0) {
        return;
    }
    if (binCount_ != n || scroll_.isNull()) {
        binCount_ = n;
        const int w = static_cast<int>(2 * n);
        scroll_ = QImage(w, k_history_lines, QImage::Format_RGB888);
        scroll_.fill(kSonarBackgroundColor);
    }

    const int h = scroll_.height();
    const int bpl = scroll_.bytesPerLine();
    uchar* bits = scroll_.bits();
    std::memmove(bits, bits + static_cast<size_t>(bpl), static_cast<size_t>(bpl * (h - 1)));

    uchar* row = bits + static_cast<size_t>(bpl * (h - 1));
    for (unsigned x = 0; x < n; ++x) {
        const float v = pb[n - 1 - x];
        int cm = static_cast<int>(std::round(v * 255.0f));
        cm = std::clamp(cm, 0, 255);
        const QColor& c = colorMap_[static_cast<size_t>(cm)];
        row[static_cast<int>(x) * 3 + 0] = static_cast<uchar>(c.red());
        row[static_cast<int>(x) * 3 + 1] = static_cast<uchar>(c.green());
        row[static_cast<int>(x) * 3 + 2] = static_cast<uchar>(c.blue());
    }
    for (unsigned x = 0; x < n; ++x) {
        const float v = sb[x];
        int cm = static_cast<int>(std::round(v * 255.0f));
        cm = std::clamp(cm, 0, 255);
        const QColor& c = colorMap_[static_cast<size_t>(cm)];
        row[static_cast<int>(n + x) * 3 + 0] = static_cast<uchar>(c.red());
        row[static_cast<int>(n + x) * 3 + 1] = static_cast<uchar>(c.green());
        row[static_cast<int>(n + x) * 3 + 2] = static_cast<uchar>(c.blue());
    }
    update();
}

void SideScanWaterfallCanvas::resizeEvent(QResizeEvent* event)
{
    QFrame::resizeEvent(event);
    update();
}

void SideScanWaterfallCanvas::paintEvent(QPaintEvent*)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);

    constexpr int margin_bottom = 40;
    constexpr int margin_left = 44;
    const QRect plotRect(margin_left, 0, std::max(1, width() - margin_left), std::max(1, height() - margin_bottom));

    painter.fillRect(rect(), kSonarBackgroundColor);

    if (!scroll_.isNull() && plotRect.width() > 0 && plotRect.height() > 0) {
        painter.drawImage(plotRect, scroll_);
    }

    if (enabledGrid_ && plotRect.width() > 2 && plotRect.height() > 2) {
        painter.setPen(QPen(QColor(255, 255, 255, 70), 1));
        const int w = plotRect.width();
        const int hh = plotRect.height();
        for (int i = 1; i < 5; ++i) {
            const int y = plotRect.top() + hh * i / 5;
            painter.drawLine(plotRect.left(), y, plotRect.left() + w, y);
        }
        painter.setPen(QPen(QColor(255, 255, 255, 130), 1));
        const int cx = plotRect.left() + w / 2;
        painter.drawLine(cx, plotRect.top(), cx, plotRect.top() + hh);
        painter.drawLine(plotRect.left() + w / 4, plotRect.top(), plotRect.left() + w / 4, plotRect.top() + hh);
        painter.drawLine(plotRect.left() + 3 * w / 4, plotRect.top(), plotRect.left() + 3 * w / 4, plotRect.top() + hh);
    }

    painter.setPen(Qt::white);
    QFont f = painter.font();
    f.setPointSize(9);
    painter.setFont(f);
    const int by = height() - 22;
    painter.drawText(plotRect.left(), by, QString::number(-rangeM_) + QString::fromLatin1(" m"));
    painter.drawText(std::max(0, plotRect.left() + plotRect.width() / 2 - 10), by, QString::fromLatin1("0"));
    painter.drawText(plotRect.left() + plotRect.width() - 40, by, QString::fromLatin1("+") + QString::number(rangeM_) + QString::fromLatin1(" m"));

    painter.save();
    painter.translate(12, plotRect.top() + plotRect.height() / 2 + 40);
    painter.rotate(-90);
    painter.drawText(0, 0, QString::fromUtf8("时间"));
    painter.restore();
}
