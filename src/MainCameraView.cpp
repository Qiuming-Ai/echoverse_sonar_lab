#include "MainCameraView.hpp"

#include <osg/Camera>
#include <osg/GraphicsContext>
#include <osg/Image>
#include <osg/RenderInfo>
#include <osg/Texture2D>
#include <osgGA/EventQueue>
#include <osgGA/GUIEventAdapter>
#include <osgViewer/Viewer>

#include <QMouseEvent>
#include <QPainter>
#include <QWheelEvent>

#include <algorithm>

namespace {

int osgMouseButton(Qt::MouseButton button) {
    switch (button) {
    case Qt::LeftButton:
        return 1;
    case Qt::MiddleButton:
        return 2;
    case Qt::RightButton:
        return 3;
    default:
        return 1;
    }
}

} // namespace

struct MainCameraView::CaptureCallback : public osg::Camera::DrawCallback {
    CaptureCallback(MainCameraView* owner, osg::Texture2D* tex) : owner_(owner), tex_(tex) {}

    void operator()(osg::RenderInfo& render_info) const override {
        if (!owner_) {
            return;
        }
        const int width = std::max(1, tex_->getTextureWidth());
        const int height = std::max(1, tex_->getTextureHeight());
        render_info.getState()->applyTextureAttribute(0, tex_.get());
        osg::ref_ptr<osg::Image> image = new osg::Image;
        image->allocateImage(width, height, 1, GL_RGBA, GL_UNSIGNED_BYTE);
        image->readImageFromCurrentTexture(render_info.getContextID(), true, GL_UNSIGNED_BYTE);
        // The viewer is single-threaded, so the callback and refresh() share one thread.
        owner_->captured_ = image;
    }

    MainCameraView* owner_;
    osg::ref_ptr<osg::Texture2D> tex_;
};

MainCameraView::MainCameraView(QWidget* parent) : QWidget(parent) {
    setAttribute(Qt::WA_OpaquePaintEvent);
}

MainCameraView::~MainCameraView() {
    if (viewer_) {
        viewer_->getCamera()->setPostDrawCallback(nullptr);
    }
}

bool MainCameraView::setup(osgViewer::Viewer* viewer, int width, int height) {
    viewer_ = viewer;
    const int render_width = std::max(1, width);
    const int render_height = std::max(1, height);

    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 0;
    traits->y = 0;
    traits->width = render_width;
    traits->height = render_height;
    traits->pbuffer = true;
    traits->readDISPLAY();
    gfx_ = osg::GraphicsContext::createGraphicsContext(traits.get());
    if (!gfx_.valid()) {
        return false;
    }

    osg::ref_ptr<osg::Texture2D> texture = new osg::Texture2D;
    texture->setTextureSize(render_width, render_height);
    texture->setInternalFormat(GL_RGBA);
    texture->setSourceFormat(GL_RGBA);
    texture->setSourceType(GL_UNSIGNED_BYTE);
    texture->setResizeNonPowerOfTwoHint(false);
    texture->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
    texture->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
    tex_ = texture;

    osg::Camera* camera = viewer->getCamera();
    camera->setGraphicsContext(gfx_.get());
    camera->setViewport(0, 0, render_width, render_height);
    camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    camera->setRenderOrder(osg::Camera::PRE_RENDER, 0);
    camera->attach(osg::Camera::COLOR_BUFFER0, tex_.get());
    camera->setDrawBuffer(GL_FRONT);
    camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);

    capture_cb_ = new CaptureCallback(this, tex_.get());
    // MainCameraFileWriter owns the FinalDraw slot, so use PostDraw here.
    camera->setPostDrawCallback(capture_cb_.get());

    rt_width_ = render_width;
    rt_height_ = render_height;
    setup_ok_ = true;
    return true;
}

void MainCameraView::refresh() {
    if (!setup_ok_ || !captured_.valid() || !captured_->data()) {
        return;
    }
    const int width = captured_->s();
    const int height = captured_->t();
    if (width <= 0 || height <= 0) {
        return;
    }
    QImage image(static_cast<const uchar*>(captured_->data()),
                 width,
                 height,
                 width * 4,
                 QImage::Format_RGBA8888);
    if (captured_->getOrigin() == osg::Image::BOTTOM_LEFT) {
        image = image.mirrored(false, true);
    }
    display_image_ = image.copy();
    update();
}

void MainCameraView::resizeRenderTarget(int width, int height) {
    if (!setup_ok_ || !tex_.valid()) {
        return;
    }
    const int render_width = std::max(1, width);
    const int render_height = std::max(1, height);
    if (render_width == rt_width_ && render_height == rt_height_) {
        return;
    }
    rt_width_ = render_width;
    rt_height_ = render_height;
    tex_->setTextureSize(render_width, render_height);
}

void MainCameraView::paintEvent(QPaintEvent*) {
    QPainter painter(this);
    painter.fillRect(rect(), QColor(0, 0, 0));
    if (!display_image_.isNull()) {
        painter.setRenderHint(QPainter::SmoothPixmapTransform);
        painter.drawImage(rect(), display_image_);
    }
}

void MainCameraView::mousePressEvent(QMouseEvent* event) {
    if (osgGA::EventQueue* queue = viewer_ ? viewer_->getEventQueue() : nullptr) {
        queue->mouseButtonPress(event->position().x(),
                                height() - event->position().y(),
                                osgMouseButton(event->button()));
    }
    event->accept();
}

void MainCameraView::mouseMoveEvent(QMouseEvent* event) {
    if (osgGA::EventQueue* queue = viewer_ ? viewer_->getEventQueue() : nullptr) {
        queue->mouseMotion(event->position().x(), height() - event->position().y());
    }
    event->accept();
}

void MainCameraView::mouseReleaseEvent(QMouseEvent* event) {
    if (osgGA::EventQueue* queue = viewer_ ? viewer_->getEventQueue() : nullptr) {
        queue->mouseButtonRelease(event->position().x(),
                                  height() - event->position().y(),
                                  osgMouseButton(event->button()));
    }
    event->accept();
}

void MainCameraView::wheelEvent(QWheelEvent* event) {
    if (osgGA::EventQueue* queue = viewer_ ? viewer_->getEventQueue() : nullptr) {
        queue->mouseScroll(event->angleDelta().y() > 0
                               ? osgGA::GUIEventAdapter::SCROLL_UP
                               : osgGA::GUIEventAdapter::SCROLL_DOWN);
    }
    event->accept();
}

