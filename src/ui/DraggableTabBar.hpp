#pragma once

#include <QPoint>
#include <QTabBar>

class DraggableTabBar : public QTabBar {
    Q_OBJECT
public:
    explicit DraggableTabBar(QWidget* parent = nullptr);

signals:
    void startDragRequested(int tab_index, const QPoint& global_pos);

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;

private:
    QPoint press_pos_;
    int press_tab_index_ = -1;
};

