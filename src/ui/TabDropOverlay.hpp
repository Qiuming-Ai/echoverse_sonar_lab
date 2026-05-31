#pragma once

#include <QWidget>

class TabDropOverlay : public QWidget {
    Q_OBJECT
public:
    enum class DisplayMode {
        Detailed = 0,
        SingleTarget,
    };

    enum class DropZone {
        None = 0,
        Center,
        Left,
        Right,
        Top,
        Bottom,
        TopLeft,
        TopRight,
        BottomLeft,
        BottomRight,
    };

    explicit TabDropOverlay(QWidget* parent = nullptr);

    DropZone zoneAt(const QPoint& local_pos) const;
    void setHoveredZone(DropZone zone);
    DropZone hoveredZone() const;
    void setDisplayMode(DisplayMode mode);
    DisplayMode displayMode() const;

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QRect centerRect() const;
    QRect leftRect() const;
    QRect rightRect() const;
    QRect topRect() const;
    QRect bottomRect() const;
    QRect topLeftRect() const;
    QRect topRightRect() const;
    QRect bottomLeftRect() const;
    QRect bottomRightRect() const;

    DropZone hovered_zone_ = DropZone::None;
    DisplayMode display_mode_ = DisplayMode::Detailed;
};

