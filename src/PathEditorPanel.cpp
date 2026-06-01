#include "PathEditorPanel.hpp"

#include <QCryptographicHash>
#include <QDataStream>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QAbstractItemView>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QLabel>
#include <QMenu>
#include <QMouseEvent>
#include <QPainter>
#include <QSignalBlocker>
#include <QVBoxLayout>
#include <QWheelEvent>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>

namespace standalone_mvp {
namespace {

constexpr quint32 kMapCacheVersion = 5u;

double parseDoubleOr(const QTableWidgetItem* item, double fallback) {
    if (!item) {
        return fallback;
    }
    bool ok = false;
    const double v = item->text().toDouble(&ok);
    return ok ? v : fallback;
}

} // namespace

class PathMapCanvas final : public QWidget {
public:
    explicit PathMapCanvas(QWidget* parent = nullptr)
        : QWidget(parent) {
        setMinimumSize(220, 220);
        setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        setMouseTracking(true);
    }

    void setMap(const sonar_imaging::TopDownDepthMapResult& map) {
        map_ = map;
        zoom_ = 1.0;
        pan_world_ = QPointF(0.0, 0.0);
        dragging_index_ = -1;
        is_panning_ = false;
        update();
    }

    void setWaypoints(const std::vector<PathWaypointConfig>& points) {
        waypoints_ = points;
        update();
    }

    void setWaypointRenderingEnabled(bool enabled) {
        draw_waypoints_ = enabled;
        update();
    }

    void setInteractionMode(bool add_mode, bool edit_mode) {
        add_mode_ = add_mode;
        edit_mode_ = edit_mode;
        dragging_index_ = -1;
    }

    void setLivePose(double x, double y, double yaw_rad) {
        live_x_ = x;
        live_y_ = y;
        live_yaw_rad_ = yaw_rad;
        has_live_pose_ = true;
        update();
    }

    bool hasMap() const { return map_.valid(); }

    std::function<void(double x, double y, double z)> onAddPoint;
    std::function<void(int index, double x, double y)> onMovePointXY;
    std::function<void(int index)> onDeletePoint;

protected:
    void paintEvent(QPaintEvent*) override {
        QPainter p(this);
        p.fillRect(rect(), QColor(12, 18, 28));

        if (!map_.valid()) {
            p.setPen(QColor(180, 200, 220));
            p.drawText(rect(), Qt::AlignCenter, QStringLiteral("Map is loading..."));
            return;
        }
        const QRectF target = imageRectInWidget();
        const QRectF source = imageSourceRectInMapPixels();
        p.save();
        p.setClipRect(target);
        p.drawImage(target, map_.image, source);
        drawContours(p, target, source);

        if (draw_waypoints_) {
            p.setRenderHint(QPainter::Antialiasing, true);
            QPen line_pen(QColor(255, 210, 80));
            line_pen.setWidth(2);
            p.setPen(line_pen);
            for (std::size_t i = 1; i < waypoints_.size(); ++i) {
                const QPointF a = worldToWidget(QPointF(waypoints_[i - 1].x, waypoints_[i - 1].y));
                const QPointF b = worldToWidget(QPointF(waypoints_[i].x, waypoints_[i].y));
                p.drawLine(a, b);
            }

            p.setPen(Qt::NoPen);
            for (std::size_t i = 0; i < waypoints_.size(); ++i) {
                const QPointF pos = worldToWidget(QPointF(waypoints_[i].x, waypoints_[i].y));
                p.setBrush(i == 0 ? QColor(80, 220, 120) : QColor(255, 120, 80));
                p.drawEllipse(pos, 5.0, 5.0);
            }
        }
        drawLivePose(p);
        p.restore();

        drawAxes(p, target);
        drawColorbar(p, target);
    }

    void mousePressEvent(QMouseEvent* event) override {
        if (!map_.valid()) {
            QWidget::mousePressEvent(event);
            return;
        }
        if (event->button() == Qt::RightButton) {
            if (edit_mode_) {
                const int idx = nearestWaypointIndex(event->position(), 9.0);
                if (idx >= 0 && onDeletePoint) {
                    onDeletePoint(idx);
                    return;
                }
            }
            is_panning_ = true;
            pan_last_pos_ = event->position();
            return;
        }
        if (event->button() != Qt::LeftButton) {
            QWidget::mousePressEvent(event);
            return;
        }
        if (edit_mode_) {
            const int idx = nearestWaypointIndex(event->position(), 10.0);
            if (idx >= 0) {
                dragging_index_ = idx;
                drag_last_pos_ = event->position();
                return;
            }
        }
        if (!add_mode_) {
            return;
        }
        const QPointF pixel = widgetToImagePixel(event->position());
        if (pixel.x() < 0.0 || pixel.y() < 0.0) {
            return;
        }
        if (!map_.hasHitAtPixel(pixel)) {
            return;
        }
        const QPointF world = map_.pixelToWorld(pixel);
        const double z = map_.sampleZAtPixel(pixel) + 5.0;
        if (onAddPoint) {
            onAddPoint(world.x(), world.y(), z);
        }
    }

    void mouseMoveEvent(QMouseEvent* event) override {
        if (is_panning_ && map_.valid()) {
            const QRectF target = imageRectInWidget();
            const ViewBounds vb = currentViewBounds();
            const QPointF delta = event->position() - pan_last_pos_;
            if (target.width() > 1.0 && target.height() > 1.0) {
                const double span_x = std::max(1e-9, vb.max_x - vb.min_x);
                const double span_y = std::max(1e-9, vb.max_y - vb.min_y);
                pan_world_.rx() -= delta.x() / target.width() * span_x;
                pan_world_.ry() += delta.y() / target.height() * span_y;
                update();
            }
            pan_last_pos_ = event->position();
            return;
        }
        if (!map_.valid() || dragging_index_ < 0 || !edit_mode_) {
            QWidget::mouseMoveEvent(event);
            return;
        }
        const QPointF pixel = widgetToImagePixel(event->position());
        if (pixel.x() < 0.0 || pixel.y() < 0.0 || !map_.hasHitAtPixel(pixel)) {
            return;
        }
        const QPointF world = map_.pixelToWorld(pixel);
        if (onMovePointXY) {
            onMovePointXY(dragging_index_, world.x(), world.y());
        }
        drag_last_pos_ = event->position();
    }

    void mouseReleaseEvent(QMouseEvent* event) override {
        if (event->button() == Qt::RightButton) {
            is_panning_ = false;
        }
        if (event->button() == Qt::LeftButton) {
            dragging_index_ = -1;
        }
        QWidget::mouseReleaseEvent(event);
    }

    void wheelEvent(QWheelEvent* event) override {
        if (!map_.valid()) {
            QWidget::wheelEvent(event);
            return;
        }
        const QPointF cursor = event->position();
        const auto before = widgetToWorld(cursor);
        const double steps = static_cast<double>(event->angleDelta().y()) / 120.0;
        if (std::abs(steps) < 1e-6) {
            event->accept();
            return;
        }
        zoom_ = std::clamp(zoom_ * std::pow(1.15, steps), 1.0, 30.0);
        const auto after = widgetToWorld(cursor);
        if (before.valid && after.valid) {
            pan_world_.rx() += before.world.x() - after.world.x();
            pan_world_.ry() += before.world.y() - after.world.y();
        }
        update();
        event->accept();
    }

private:
    struct ViewBounds {
        double min_x = 0.0;
        double max_x = 0.0;
        double min_y = 0.0;
        double max_y = 0.0;
    };

    struct WorldAtCursor {
        bool valid = false;
        QPointF world;
    };

    ViewBounds currentViewBounds() const {
        ViewBounds vb;
        if (!map_.valid()) {
            return vb;
        }
        const double base_min_x = map_.min_x;
        const double base_max_x = map_.max_x;
        const double base_min_y = map_.min_y;
        const double base_max_y = map_.max_y;
        const double base_w = std::max(1e-6, base_max_x - base_min_x);
        const double base_h = std::max(1e-6, base_max_y - base_min_y);
        const double half_w = base_w * 0.5 / std::max(1.0, zoom_);
        const double half_h = base_h * 0.5 / std::max(1.0, zoom_);

        double cx = 0.5 * (base_min_x + base_max_x) + pan_world_.x();
        double cy = 0.5 * (base_min_y + base_max_y) + pan_world_.y();
        cx = std::clamp(cx, base_min_x + half_w, base_max_x - half_w);
        cy = std::clamp(cy, base_min_y + half_h, base_max_y - half_h);

        vb.min_x = cx - half_w;
        vb.max_x = cx + half_w;
        vb.min_y = cy - half_h;
        vb.max_y = cy + half_h;
        return vb;
    }

    QRectF mapViewportRect() const {
        constexpr double left = 54.0;
        constexpr double right = 86.0;
        constexpr double top = 18.0;
        constexpr double bottom = 54.0;
        return rect().adjusted(static_cast<int>(left), static_cast<int>(top), -static_cast<int>(right), -static_cast<int>(bottom));
    }

    QRectF imageRectInWidget() const {
        if (!map_.valid()) {
            return QRectF();
        }
        const QRectF viewport = mapViewportRect();
        const double rw = std::max(1.0, viewport.width());
        const double rh = std::max(1.0, viewport.height());
        const double iw = static_cast<double>(map_.width);
        const double ih = static_cast<double>(map_.height);
        const double scale = std::min(rw / iw, rh / ih);
        const double dw = iw * scale;
        const double dh = ih * scale;
        return QRectF(viewport.left() + (rw - dw) * 0.5, viewport.top() + (rh - dh) * 0.5, dw, dh);
    }

    QPointF widgetToImagePixel(const QPointF& widget_pos) const {
        const QRectF target = imageRectInWidget();
        if (!target.contains(widget_pos)) {
            return QPointF(-1.0, -1.0);
        }
        const QRectF source = imageSourceRectInMapPixels();
        const double u = (widget_pos.x() - target.left()) / std::max(1.0, target.width());
        const double v = (widget_pos.y() - target.top()) / std::max(1.0, target.height());
        return QPointF(
            std::clamp(source.left() + u * source.width(), 0.0, static_cast<double>(map_.width - 1)),
            std::clamp(source.top() + v * source.height(), 0.0, static_cast<double>(map_.height - 1)));
    }

    QPointF worldToWidget(const QPointF& world) const {
        const QRectF target = imageRectInWidget();
        const ViewBounds vb = currentViewBounds();
        const double span_x = std::max(1e-9, vb.max_x - vb.min_x);
        const double span_y = std::max(1e-9, vb.max_y - vb.min_y);
        const double u = (world.x() - vb.min_x) / span_x;
        const double v = (vb.max_y - world.y()) / span_y;
        const double x = target.left() + u * target.width();
        const double y = target.top() + v * target.height();
        return QPointF(x, y);
    }

    QPointF imageToWidgetPixel(double px, double py, const QRectF& target, const QRectF& source) const {
        const double x = target.left() + (px - source.left()) / std::max(1.0, source.width()) * target.width();
        const double y = target.top() + (py - source.top()) / std::max(1.0, source.height()) * target.height();
        return QPointF(x, y);
    }

    QRectF imageSourceRectInMapPixels() const {
        if (!map_.valid()) {
            return QRectF();
        }
        const ViewBounds vb = currentViewBounds();
        const QPointF top_left = map_.worldToPixel(QPointF(vb.min_x, vb.max_y));
        const QPointF bottom_right = map_.worldToPixel(QPointF(vb.max_x, vb.min_y));
        const double left = std::clamp(std::min(top_left.x(), bottom_right.x()), 0.0, static_cast<double>(map_.width - 1));
        const double right = std::clamp(std::max(top_left.x(), bottom_right.x()), 0.0, static_cast<double>(map_.width - 1));
        const double top = std::clamp(std::min(top_left.y(), bottom_right.y()), 0.0, static_cast<double>(map_.height - 1));
        const double bottom = std::clamp(std::max(top_left.y(), bottom_right.y()), 0.0, static_cast<double>(map_.height - 1));
        return QRectF(left, top, std::max(1.0, right - left), std::max(1.0, bottom - top));
    }

    WorldAtCursor widgetToWorld(const QPointF& widget_pos) const {
        WorldAtCursor out;
        if (!map_.valid()) {
            return out;
        }
        const QRectF target = imageRectInWidget();
        if (!target.contains(widget_pos)) {
            return out;
        }
        const ViewBounds vb = currentViewBounds();
        const double u = (widget_pos.x() - target.left()) / std::max(1.0, target.width());
        const double v = (widget_pos.y() - target.top()) / std::max(1.0, target.height());
        out.valid = true;
        out.world = QPointF(vb.min_x + u * (vb.max_x - vb.min_x), vb.max_y - v * (vb.max_y - vb.min_y));
        return out;
    }

    void drawAxes(QPainter& p, const QRectF& target) const {
        p.setRenderHint(QPainter::Antialiasing, false);
        p.setPen(QPen(QColor(190, 210, 235), 1));
        p.drawRect(target);
        p.drawLine(QPointF(target.left(), target.bottom()), QPointF(target.right(), target.bottom()));
        p.drawLine(QPointF(target.left(), target.top()), QPointF(target.left(), target.bottom()));
        const ViewBounds vb = currentViewBounds();

        constexpr int tick_count = 5;
        for (int i = 0; i <= tick_count; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(tick_count);
            const double x = target.left() + t * target.width();
            const double y = target.bottom() - t * target.height();

            // X ticks
            p.drawLine(QPointF(x, target.bottom()), QPointF(x, target.bottom() + 5.0));
            const double wx = vb.min_x + t * (vb.max_x - vb.min_x);
            p.drawText(QRectF(x - 30.0, target.bottom() + 7.0, 60.0, 16.0), Qt::AlignHCenter | Qt::AlignTop,
                       QString::number(wx, 'f', 1));

            // Y ticks
            p.drawLine(QPointF(target.left() - 5.0, y), QPointF(target.left(), y));
            const double wy = vb.min_y + (1.0 - t) * (vb.max_y - vb.min_y);
            p.drawText(QRectF(target.left() - 48.0, y - 8.0, 40.0, 16.0), Qt::AlignRight | Qt::AlignVCenter,
                       QString::number(wy, 'f', 1));
        }

        p.drawText(QRectF(target.left(), target.bottom() + 24.0, target.width(), 20.0), Qt::AlignHCenter | Qt::AlignTop,
                   QStringLiteral("X (m)"));
        p.save();
        p.translate(target.left() - 42.0, target.center().y());
        p.rotate(-90.0);
        p.drawText(QRectF(-target.height() * 0.5, -12.0, target.height(), 20.0), Qt::AlignCenter, QStringLiteral("Y (m)"));
        p.restore();
    }

    void drawColorbar(QPainter& p, const QRectF& target) const {
        const QRectF bar(target.right() + 22.0, target.top(), 18.0, target.height());
        QLinearGradient g(bar.topLeft(), bar.bottomLeft());
        g.setColorAt(0.0, QColor::fromHsv(200, 200, 200));
        g.setColorAt(1.0, QColor::fromHsv(0, 200, 120));
        p.fillRect(bar, g);
        p.setPen(QPen(QColor(190, 210, 235), 1));
        p.drawRect(bar);

        constexpr int ticks = 5;
        for (int i = 0; i <= ticks; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(ticks);
            const double y = bar.top() + t * bar.height();
            p.drawLine(QPointF(bar.right(), y), QPointF(bar.right() + 5.0, y));
            const double depth = map_.z_max - t * (map_.z_max - map_.z_min);
            p.drawText(QRectF(bar.right() + 8.0, y - 8.0, 50.0, 16.0), Qt::AlignLeft | Qt::AlignVCenter,
                       QString::number(depth, 'f', 1));
        }
        p.drawText(QRectF(bar.left() - 10.0, bar.top() - 18.0, 70.0, 16.0), Qt::AlignLeft | Qt::AlignVCenter, QStringLiteral("Z (m)"));
    }

    void drawContours(QPainter& p, const QRectF& target, const QRectF& source) const {
        if (map_.z_values.empty() || map_.width < 2 || map_.height < 2) {
            return;
        }
        p.setRenderHint(QPainter::Antialiasing, true);
        const double dn_min = map_.z_min;
        const double dn_max = std::max(dn_min + 1e-6, map_.z_max);
        const std::array<double, 6> levels{
            dn_min + (dn_max - dn_min) * 0.15,
            dn_min + (dn_max - dn_min) * 0.30,
            dn_min + (dn_max - dn_min) * 0.45,
            dn_min + (dn_max - dn_min) * 0.60,
            dn_min + (dn_max - dn_min) * 0.75,
            dn_min + (dn_max - dn_min) * 0.90};
        p.setPen(QPen(QColor(245, 245, 245, 170), 1));

        auto sample = [&](int x, int y) -> double {
            const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(map_.width) + static_cast<std::size_t>(x);
            return static_cast<double>(map_.z_values[idx]);
        };
        auto is_valid = [&](int x, int y) -> bool {
            const std::size_t idx = static_cast<std::size_t>(y) * static_cast<std::size_t>(map_.width) + static_cast<std::size_t>(x);
            return idx < map_.hit_mask.size() && map_.hit_mask[idx] != 0;
        };
        auto interp = [](double v1, double v2, double level) {
            const double dv = (v2 - v1);
            if (std::abs(dv) < 1e-9) return 0.5;
            return std::clamp((level - v1) / dv, 0.0, 1.0);
        };

        for (double level : levels) {
            for (int y = 0; y < map_.height - 1; y += 3) {
                for (int x = 0; x < map_.width - 1; x += 3) {
                    if (!is_valid(x, y) || !is_valid(x + 1, y) || !is_valid(x + 1, y + 1) || !is_valid(x, y + 1)) {
                        continue;
                    }
                    const double v00 = sample(x, y);
                    const double v10 = sample(x + 1, y);
                    const double v11 = sample(x + 1, y + 1);
                    const double v01 = sample(x, y + 1);

                    std::array<QPointF, 4> pts{};
                    int cnt = 0;
                    auto add_if_cross = [&](double a, double b, double ax, double ay, double bx, double by) {
                        const bool cross = (a < level && b >= level) || (a >= level && b < level);
                        if (!cross) return;
                        const double t = interp(a, b, level);
                        pts[cnt++] = imageToWidgetPixel(ax + (bx - ax) * t, ay + (by - ay) * t, target, source);
                    };

                    add_if_cross(v00, v10, x, y, x + 1, y);
                    add_if_cross(v10, v11, x + 1, y, x + 1, y + 1);
                    add_if_cross(v11, v01, x + 1, y + 1, x, y + 1);
                    add_if_cross(v01, v00, x, y + 1, x, y);

                    if (cnt == 2) {
                        p.drawLine(pts[0], pts[1]);
                    } else if (cnt == 4) {
                        p.drawLine(pts[0], pts[1]);
                        p.drawLine(pts[2], pts[3]);
                    }
                }
            }
        }
    }

    void drawLivePose(QPainter& p) const {
        if (!has_live_pose_ || !map_.valid()) {
            return;
        }
        const QPointF pos = worldToWidget(QPointF(live_x_, live_y_));
        const double arrow_len = 22.0;
        const QPointF tip(
            pos.x() + std::cos(live_yaw_rad_) * arrow_len,
            pos.y() - std::sin(live_yaw_rad_) * arrow_len);
        const double side = 7.0;
        const QPointF left(
            tip.x() - std::cos(live_yaw_rad_ - 0.7) * side,
            tip.y() + std::sin(live_yaw_rad_ - 0.7) * side);
        const QPointF right(
            tip.x() - std::cos(live_yaw_rad_ + 0.7) * side,
            tip.y() + std::sin(live_yaw_rad_ + 0.7) * side);

        p.setRenderHint(QPainter::Antialiasing, true);
        p.setPen(QPen(QColor(30, 250, 120), 2));
        p.setBrush(QColor(30, 250, 120));
        p.drawEllipse(pos, 4.0, 4.0);
        p.drawLine(pos, tip);
        QPolygonF tri;
        tri << tip << left << right;
        p.drawPolygon(tri);
    }

    sonar_imaging::TopDownDepthMapResult map_;
    std::vector<PathWaypointConfig> waypoints_;
    bool draw_waypoints_ = true;
    bool add_mode_ = true;
    bool edit_mode_ = false;
    int dragging_index_ = -1;
    QPointF drag_last_pos_;
    bool is_panning_ = false;
    QPointF pan_last_pos_;
    double zoom_ = 1.0;
    QPointF pan_world_;
    bool has_live_pose_ = false;
    double live_x_ = 0.0;
    double live_y_ = 0.0;
    double live_yaw_rad_ = 0.0;

    int nearestWaypointIndex(const QPointF& widget_pos, double radius_px) const {
        if (!draw_waypoints_ || waypoints_.empty()) {
            return -1;
        }
        const double r2 = radius_px * radius_px;
        int best = -1;
        double best_d2 = r2;
        for (int i = 0; i < static_cast<int>(waypoints_.size()); ++i) {
            const QPointF pos = worldToWidget(QPointF(waypoints_[static_cast<std::size_t>(i)].x,
                                                      waypoints_[static_cast<std::size_t>(i)].y));
            const QPointF d = pos - widget_pos;
            const double d2 = d.x() * d.x() + d.y() * d.y();
            if (d2 <= best_d2) {
                best = i;
                best_d2 = d2;
            }
        }
        return best;
    }
};

PathEditorPanel::PathEditorPanel(QWidget* parent)
    : QWidget(parent) {
    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(8, 8, 8, 8);
    root->setSpacing(8);

    map_canvas_ = new PathMapCanvas(this);
    root->addWidget(map_canvas_, 8);

    editor_panel_ = new QWidget(this);
    auto* side_layout = new QVBoxLayout(editor_panel_);
    side_layout->setContentsMargins(0, 0, 0, 0);
    side_layout->setSpacing(6);
    root->addWidget(editor_panel_, 5);
    editor_panel_->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

    // auto* title = new QLabel(QStringLiteral("Waypoints (X/Y/Z, speed applies to segment i->i+1)"), editor_panel_);
    // side_layout->addWidget(title);

    table_ = new QTableWidget(editor_panel_);
    table_->setColumnCount(4);
    table_->setHorizontalHeaderLabels({QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z"), QStringLiteral("Speed")});
    table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->setDragEnabled(true);
    table_->setAcceptDrops(true);
    table_->viewport()->setAcceptDrops(true);
    table_->setDropIndicatorShown(true);
    table_->setDragDropMode(QAbstractItemView::InternalMove);
    table_->setDragDropOverwriteMode(false);
    table_->setContextMenuPolicy(Qt::CustomContextMenu);
    side_layout->addWidget(table_, 1);

    auto* mode_row = new QHBoxLayout();
    add_mode_check_ = new QCheckBox(QStringLiteral("Add"), editor_panel_);
    edit_mode_check_ = new QCheckBox(QStringLiteral("Edit"), editor_panel_);
    loop_check_ = new QCheckBox(QStringLiteral("Loop path"), editor_panel_);
    clear_btn_ = new QPushButton(QStringLiteral("Clear"), editor_panel_);
    add_mode_check_->setChecked(true);
    mode_row->addWidget(add_mode_check_);
    mode_row->addWidget(edit_mode_check_);
    mode_row->addWidget(loop_check_);
    mode_row->addWidget(clear_btn_);
    mode_row->addStretch();
    side_layout->addLayout(mode_row);

    map_canvas_->onAddPoint = [this](double x, double y, double z) {
        PathWaypointConfig wp;
        wp.x = x;
        wp.y = y;
        wp.depth_m = z;
        wp.speed_mps = cfg_.waypoints.empty() ? 1.0 : cfg_.waypoints.back().speed_mps;
        cfg_.waypoints.push_back(wp);
        refreshTable();
        updateCanvasWaypoints();
        emit pathEdited(cfg_);
    };
    map_canvas_->onMovePointXY = [this](int index, double x, double y) {
        if (index < 0 || index >= static_cast<int>(cfg_.waypoints.size())) {
            return;
        }
        auto& wp = cfg_.waypoints[static_cast<std::size_t>(index)];
        wp.x = x;
        wp.y = y;
        refreshTable();
        updateCanvasWaypoints();
        emit pathEdited(cfg_);
    };
    map_canvas_->onDeletePoint = [this](int index) {
        if (index < 0 || index >= static_cast<int>(cfg_.waypoints.size())) {
            return;
        }
        cfg_.waypoints.erase(cfg_.waypoints.begin() + index);
        refreshTable();
        updateCanvasWaypoints();
        emit pathEdited(cfg_);
    };

    QObject::connect(clear_btn_, &QPushButton::clicked, this, [this]() {
        cfg_.waypoints.clear();
        refreshTable();
        updateCanvasWaypoints();
        emit pathEdited(cfg_);
    });
    QObject::connect(loop_check_, &QCheckBox::toggled, this, [this](bool on) {
        cfg_.loop = on;
        emit pathEdited(cfg_);
    });
    QObject::connect(add_mode_check_, &QCheckBox::toggled, this, [this](bool on) {
        if (on) {
            QSignalBlocker blocker(edit_mode_check_);
            edit_mode_check_->setChecked(false);
        }
        setInteractionModeUi(add_mode_check_->isChecked(), edit_mode_check_->isChecked());
    });
    QObject::connect(edit_mode_check_, &QCheckBox::toggled, this, [this](bool on) {
        if (on) {
            QSignalBlocker blocker(add_mode_check_);
            add_mode_check_->setChecked(false);
        }
        setInteractionModeUi(add_mode_check_->isChecked(), edit_mode_check_->isChecked());
    });
    QObject::connect(table_, &QTableWidget::itemChanged, this, [this](QTableWidgetItem*) {
        syncConfigFromTable();
        updateCanvasWaypoints();
        emit pathEdited(cfg_);
    });
    QObject::connect(table_->model(), &QAbstractItemModel::rowsMoved, this, [this]() {
        syncConfigFromTable();
        updateCanvasWaypoints();
        emit pathEdited(cfg_);
    });
    QObject::connect(table_, &QWidget::customContextMenuRequested, this, [this](const QPoint& pos) {
        const QModelIndex idx = table_->indexAt(pos);
        if (!idx.isValid()) {
            return;
        }
        QMenu menu(table_);
        QAction* del = menu.addAction(QStringLiteral("Delete waypoint"));
        QAction* chosen = menu.exec(table_->viewport()->mapToGlobal(pos));
        if (chosen != del) {
            return;
        }
        cfg_.waypoints.erase(cfg_.waypoints.begin() + idx.row());
        refreshTable();
        updateCanvasWaypoints();
        emit pathEdited(cfg_);
    });

    setInteractionModeUi(true, false);
}

void PathEditorPanel::setSceneRoot(osg::ref_ptr<osg::Node> scene_root) {
    scene_root_ = scene_root;
    ensureMapReady();
}

void PathEditorPanel::setProjectContext(const QString& project_root, const QString& world_spec) {
    project_root_ = project_root.trimmed();
    world_spec_ = world_spec.trimmed();
    ensureMapReady();
}

void PathEditorPanel::setPathConfig(const PathModeConfig& cfg) {
    cfg_ = cfg;
    {
        QSignalBlocker blocker(loop_check_);
        loop_check_->setChecked(cfg_.loop);
    }
    refreshTable();
    updateCanvasWaypoints();
}

void PathEditorPanel::setCompactLiveMapMode(bool on) {
    if (editor_panel_) {
        editor_panel_->setVisible(!on);
    }
    map_canvas_->setWaypointRenderingEnabled(!on);
    if (on) {
        map_canvas_->setInteractionMode(false, false);
    } else {
        map_canvas_->setInteractionMode(add_mode_check_ && add_mode_check_->isChecked(),
                                        edit_mode_check_ && edit_mode_check_->isChecked());
    }
}

void PathEditorPanel::setLivePose(double x, double y, double yaw_rad) {
    map_canvas_->setLivePose(x, y, yaw_rad);
}

void PathEditorPanel::notifySceneChanged() {
    const QString cache_path = cacheFilePath();
    if (!cache_path.isEmpty()) {
        QFile::remove(cache_path);
    }
    ensureMapReady();
}

PathModeConfig PathEditorPanel::pathConfig() const {
    return cfg_;
}

void PathEditorPanel::ensureMapReady() {
    if (!scene_root_.valid()) {
        return;
    }
    sonar_imaging::TopDownDepthMapResult map;
    const QString cache_path = cacheFilePath();
    const bool cache_ok = !cache_path.isEmpty() && loadMapCache(cache_path, &map);
    if (cache_ok) {
        map_canvas_->setMap(map);
        return;
    }

    map = map_generator_.generate(scene_root_, 900, 900);
    map_canvas_->setMap(map);
    if (!cache_path.isEmpty() && map.valid()) {
        saveMapCache(cache_path, map);
    }
}

QString PathEditorPanel::cacheFilePath() const {
    if (project_root_.isEmpty()) {
        return {};
    }
    const QString cache_dir = QDir(project_root_).filePath(QStringLiteral(".cache/path_mode"));
    QDir().mkpath(cache_dir);
    return QDir(cache_dir).filePath(cacheKey() + QStringLiteral(".bin"));
}

QString PathEditorPanel::cacheKey() const {
    QString key = world_spec_.trimmed();
    if (key.isEmpty()) {
        key = QStringLiteral("default_world");
    }
    const QByteArray hash = QCryptographicHash::hash(key.toUtf8(), QCryptographicHash::Sha1).toHex();
    return QStringLiteral("topdown_") + QString::fromLatin1(hash.left(16));
}

qint64 PathEditorPanel::worldMTimeMs() const {
    const QFileInfo fi(world_spec_);
    if (!fi.exists() || !fi.isFile()) {
        return -1;
    }
    return fi.lastModified().toMSecsSinceEpoch();
}

bool PathEditorPanel::loadMapCache(const QString& file_path, sonar_imaging::TopDownDepthMapResult* out) const {
    if (!out) {
        return false;
    }
    QFile f(file_path);
    if (!f.exists() || !f.open(QIODevice::ReadOnly)) {
        return false;
    }
    QDataStream in(&f);
    in.setVersion(QDataStream::Qt_6_0);
    quint32 version = 0;
    qint64 world_mtime = -1;
    in >> version;
    if (version != kMapCacheVersion) {
        return false;
    }
    in >> world_mtime;
    if (world_mtime != worldMTimeMs()) {
        return false;
    }

    sonar_imaging::TopDownDepthMapResult map;
    in >> map.width >> map.height;
    in >> map.min_x >> map.max_x >> map.min_y >> map.max_y;
    in >> map.far_plane_m >> map.eye_z >> map.z_min >> map.z_max;
    in >> map.image;
    QByteArray depth_bytes;
    in >> depth_bytes;
    QByteArray mask_bytes;
    in >> mask_bytes;
    QByteArray z_bytes;
    in >> z_bytes;
    if (in.status() != QDataStream::Ok || map.width <= 0 || map.height <= 0 || map.image.isNull()) {
        return false;
    }
    map.depth_norm.resize(depth_bytes.size() / static_cast<int>(sizeof(float)));
    if (!depth_bytes.isEmpty() && !map.depth_norm.empty()) {
        std::memcpy(map.depth_norm.data(), depth_bytes.constData(), map.depth_norm.size() * sizeof(float));
    }
    map.hit_mask.resize(mask_bytes.size());
    for (int i = 0; i < mask_bytes.size(); ++i) {
        map.hit_mask[static_cast<std::size_t>(i)] = static_cast<unsigned char>(mask_bytes.at(i));
    }
    map.z_values.resize(z_bytes.size() / static_cast<int>(sizeof(float)));
    if (!z_bytes.isEmpty() && !map.z_values.empty()) {
        std::memcpy(map.z_values.data(), z_bytes.constData(), map.z_values.size() * sizeof(float));
    }
    if (map.hit_mask.size() != map.depth_norm.size() || map.z_values.size() != map.depth_norm.size()) {
        return false;
    }
    *out = std::move(map);
    return true;
}

bool PathEditorPanel::saveMapCache(const QString& file_path, const sonar_imaging::TopDownDepthMapResult& map) const {
    QFile f(file_path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return false;
    }
    QDataStream out(&f);
    out.setVersion(QDataStream::Qt_6_0);
    out << static_cast<quint32>(kMapCacheVersion);
    out << worldMTimeMs();
    out << map.width << map.height;
    out << map.min_x << map.max_x << map.min_y << map.max_y;
    out << map.far_plane_m << map.eye_z << map.z_min << map.z_max;
    out << map.image;
    const QByteArray depth_bytes(reinterpret_cast<const char*>(map.depth_norm.data()),
                                 static_cast<int>(map.depth_norm.size() * sizeof(float)));
    out << depth_bytes;
    const QByteArray mask_bytes(reinterpret_cast<const char*>(map.hit_mask.data()),
                                static_cast<int>(map.hit_mask.size()));
    out << mask_bytes;
    const QByteArray z_bytes(reinterpret_cast<const char*>(map.z_values.data()),
                             static_cast<int>(map.z_values.size() * sizeof(float)));
    out << z_bytes;
    return out.status() == QDataStream::Ok;
}

void PathEditorPanel::refreshTable() {
    QSignalBlocker blocker(table_);
    table_->setRowCount(0);
    for (int i = 0; i < static_cast<int>(cfg_.waypoints.size()); ++i) {
        const auto& wp = cfg_.waypoints[static_cast<std::size_t>(i)];
        table_->insertRow(i);
        table_->setItem(i, 0, new QTableWidgetItem(QString::number(wp.x, 'f', 3)));
        table_->setItem(i, 1, new QTableWidgetItem(QString::number(wp.y, 'f', 3)));
        table_->setItem(i, 2, new QTableWidgetItem(QString::number(wp.depth_m, 'f', 3)));
        table_->setItem(i, 3, new QTableWidgetItem(QString::number(wp.speed_mps, 'f', 3)));
    }
}

void PathEditorPanel::syncConfigFromTable() {
    std::vector<PathWaypointConfig> points;
    points.reserve(static_cast<std::size_t>(table_->rowCount()));
    for (int i = 0; i < table_->rowCount(); ++i) {
        PathWaypointConfig wp;
        wp.x = parseDoubleOr(table_->item(i, 0), 0.0);
        wp.y = parseDoubleOr(table_->item(i, 1), 0.0);
        wp.depth_m = parseDoubleOr(table_->item(i, 2), -5.0);
        wp.speed_mps = std::max(0.001, parseDoubleOr(table_->item(i, 3), 1.0));
        points.push_back(wp);
    }
    cfg_.waypoints = std::move(points);
}

void PathEditorPanel::updateCanvasWaypoints() {
    map_canvas_->setWaypoints(cfg_.waypoints);
}

void PathEditorPanel::setInteractionModeUi(bool add_mode, bool edit_mode) {
    map_canvas_->setInteractionMode(add_mode, edit_mode);
}

} // namespace standalone_mvp

