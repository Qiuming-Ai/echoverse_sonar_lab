#include "ui/DraggableTabBar.hpp"

#include <QApplication>
#include <QMouseEvent>

DraggableTabBar::DraggableTabBar(QWidget* parent)
    : QTabBar(parent) {
    setMovable(false);
    setAcceptDrops(false);
}

void DraggableTabBar::mousePressEvent(QMouseEvent* event) {
    press_pos_ = event->pos();
    press_tab_index_ = tabAt(press_pos_);
    QTabBar::mousePressEvent(event);
}

void DraggableTabBar::mouseMoveEvent(QMouseEvent* event) {
    if (!(event->buttons() & Qt::LeftButton) || press_tab_index_ < 0) {
        QTabBar::mouseMoveEvent(event);
        return;
    }
    if ((event->pos() - press_pos_).manhattanLength() < QApplication::startDragDistance()) {
        QTabBar::mouseMoveEvent(event);
        return;
    }
    emit startDragRequested(press_tab_index_, mapToGlobal(press_pos_));
    press_tab_index_ = -1;
    QTabBar::mouseMoveEvent(event);
}

