#include "ui/TabDropOverlay.hpp"

#include <QPainter>

namespace {
constexpr int kEdgeFractionPercent = 28;
constexpr int kCornerFractionPercent = 24;
constexpr int kCenterFractionPercent = 36;
const QColor kZoneFill(62, 111, 163, 110);
const QColor kZoneBorder(140, 190, 240, 220);
const QColor kHoverFill(98, 168, 240, 170);
const QColor kHoverBorder(200, 230, 255, 255);
} // namespace

TabDropOverlay::TabDropOverlay(QWidget* parent)
    : QWidget(parent) {
    setAttribute(Qt::WA_TransparentForMouseEvents, true);
    setAttribute(Qt::WA_NoSystemBackground, true);
}

TabDropOverlay::DropZone TabDropOverlay::zoneAt(const QPoint& local_pos) const {
    if (display_mode_ == DisplayMode::SingleTarget) {
        return rect().contains(local_pos) ? DropZone::Center : DropZone::None;
    }
    if (topLeftRect().contains(local_pos)) return DropZone::TopLeft;
    if (topRightRect().contains(local_pos)) return DropZone::TopRight;
    if (bottomLeftRect().contains(local_pos)) return DropZone::BottomLeft;
    if (bottomRightRect().contains(local_pos)) return DropZone::BottomRight;
    if (leftRect().contains(local_pos)) return DropZone::Left;
    if (rightRect().contains(local_pos)) return DropZone::Right;
    if (topRect().contains(local_pos)) return DropZone::Top;
    if (bottomRect().contains(local_pos)) return DropZone::Bottom;
    if (centerRect().contains(local_pos)) return DropZone::Center;
    return DropZone::None;
}

void TabDropOverlay::setHoveredZone(DropZone zone) {
    if (hovered_zone_ == zone) {
        return;
    }
    hovered_zone_ = zone;
    update();
}

TabDropOverlay::DropZone TabDropOverlay::hoveredZone() const {
    return hovered_zone_;
}

void TabDropOverlay::setDisplayMode(DisplayMode mode) {
    if (display_mode_ == mode) {
        return;
    }
    display_mode_ = mode;
    update();
}

TabDropOverlay::DisplayMode TabDropOverlay::displayMode() const {
    return display_mode_;
}

QRect TabDropOverlay::centerRect() const {
    const int w = width();
    const int h = height();
    const int cw = (w * kCenterFractionPercent) / 100;
    const int ch = (h * kCenterFractionPercent) / 100;
    return QRect((w - cw) / 2, (h - ch) / 2, cw, ch);
}

QRect TabDropOverlay::leftRect() const {
    return QRect(0, 0, (width() * kEdgeFractionPercent) / 100, height());
}

QRect TabDropOverlay::rightRect() const {
    const int w = (width() * kEdgeFractionPercent) / 100;
    return QRect(width() - w, 0, w, height());
}

QRect TabDropOverlay::topRect() const {
    return QRect(0, 0, width(), (height() * kEdgeFractionPercent) / 100);
}

QRect TabDropOverlay::bottomRect() const {
    const int h = (height() * kEdgeFractionPercent) / 100;
    return QRect(0, height() - h, width(), h);
}

QRect TabDropOverlay::topLeftRect() const {
    const int w = (width() * kCornerFractionPercent) / 100;
    const int h = (height() * kCornerFractionPercent) / 100;
    return QRect(0, 0, w, h);
}

QRect TabDropOverlay::topRightRect() const {
    const int w = (width() * kCornerFractionPercent) / 100;
    const int h = (height() * kCornerFractionPercent) / 100;
    return QRect(width() - w, 0, w, h);
}

QRect TabDropOverlay::bottomLeftRect() const {
    const int w = (width() * kCornerFractionPercent) / 100;
    const int h = (height() * kCornerFractionPercent) / 100;
    return QRect(0, height() - h, w, h);
}

QRect TabDropOverlay::bottomRightRect() const {
    const int w = (width() * kCornerFractionPercent) / 100;
    const int h = (height() * kCornerFractionPercent) / 100;
    return QRect(width() - w, height() - h, w, h);
}

void TabDropOverlay::paintEvent(QPaintEvent* event) {
    QWidget::paintEvent(event);
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const auto drawZone = [&](DropZone zone, const QRect& rect) {
        const bool is_hover = (zone == hovered_zone_);
        painter.setPen(QPen(is_hover ? kHoverBorder : kZoneBorder, is_hover ? 2 : 1));
        painter.setBrush(is_hover ? kHoverFill : kZoneFill);
        painter.drawRoundedRect(rect.adjusted(3, 3, -3, -3), 8, 8);
    };

    if (display_mode_ == DisplayMode::SingleTarget) {
        drawZone(DropZone::Center, rect());
        return;
    }

    drawZone(DropZone::Left, leftRect());
    drawZone(DropZone::Right, rightRect());
    drawZone(DropZone::Top, topRect());
    drawZone(DropZone::Bottom, bottomRect());
    drawZone(DropZone::TopLeft, topLeftRect());
    drawZone(DropZone::TopRight, topRightRect());
    drawZone(DropZone::BottomLeft, bottomLeftRect());
    drawZone(DropZone::BottomRight, bottomRightRect());
    drawZone(DropZone::Center, centerRect());
}

