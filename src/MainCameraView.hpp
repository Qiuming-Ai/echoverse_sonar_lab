#pragma once

#include <QImage>
#include <QWidget>

#include <osg/Image>
#include <osg/ref_ptr>

namespace osg {
class GraphicsContext;
class Texture2D;
}
namespace osgViewer {
class Viewer;
}

// Displays the OSG main camera rendered offscreen as a QImage inside the Qt UI.
// This avoids native-window sharing and therefore works on both X11 and Wayland.
// Mouse and wheel input are forwarded to the viewer so the trackball remains interactive.
class MainCameraView : public QWidget {
public:
    explicit MainCameraView(QWidget* parent = nullptr);
    ~MainCameraView() override;

    bool setup(osgViewer::Viewer* viewer, int width, int height);
    void refresh();
    void resizeRenderTarget(int width, int height);

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

private:
    struct CaptureCallback;

    osgViewer::Viewer* viewer_ = nullptr;
    osg::ref_ptr<osg::GraphicsContext> gfx_;
    osg::ref_ptr<osg::Texture2D> tex_;
    osg::ref_ptr<CaptureCallback> capture_cb_;
    osg::ref_ptr<osg::Image> captured_;
    QImage display_image_;
    int rt_width_ = 0;
    int rt_height_ = 0;
    bool setup_ok_ = false;
};

