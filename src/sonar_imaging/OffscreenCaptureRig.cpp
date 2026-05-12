#include "OffscreenCaptureRig.hpp"

#include <OpenThreads/ScopedLock>
#include <algorithm>
#include <cmath>

namespace sonar_imaging {
namespace {

constexpr unsigned kMaxRttTextureDim = 4096u;

void clampRenderSizePreservingAspect(unsigned& width, unsigned& height) {
    width = std::max(1u, width);
    height = std::max(1u, height);
    const unsigned max_dim = std::max(width, height);
    if (max_dim <= kMaxRttTextureDim) {
        return;
    }
    const double scale = static_cast<double>(kMaxRttTextureDim) / static_cast<double>(max_dim);
    width = std::max(1u, static_cast<unsigned>(std::floor(width * scale)));
    height = std::max(1u, static_cast<unsigned>(std::floor(height * scale)));
}

} // namespace

OffscreenCaptureRig::OffscreenCaptureRig(uint width, uint height) {
    setupViewer(width, height);
}

OffscreenCaptureRig::OffscreenCaptureRig(double fovY, double fovX, uint value, bool isHeight) {
    unsigned width = 0;
    unsigned height = 0;

    if (isHeight) {
        height = value;
        const double ratio = std::tan(fovX * 0.5) / std::max(std::tan(fovY * 0.5), 1e-6);
        width = static_cast<unsigned>(std::max(1.0, std::floor(static_cast<double>(height) * ratio)));
    } else {
        width = value;
        const double ratio = std::tan(fovY * 0.5) / std::max(std::tan(fovX * 0.5), 1e-6);
        height = static_cast<unsigned>(std::max(1.0, std::floor(static_cast<double>(width) * ratio)));
    }

    clampRenderSizePreservingAspect(width, height);
    setupViewer(width, height, fovY);
}

osg::Camera* OffscreenCaptureRig::createRTTCamera(osg::Camera* cam,
                                                      osg::Camera::BufferComponent buffer,
                                                      osg::Texture2D* tex,
                                                      osg::GraphicsContext* gfxc) {
    osg::ref_ptr<osg::Camera> camera = cam;
    camera->setClearColor(osg::Vec4(0, 0, 0, 1));
    camera->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    camera->setRenderOrder(osg::Camera::PRE_RENDER, 0);
    camera->setViewport(0, 0, tex->getTextureWidth(), tex->getTextureHeight());
    camera->setGraphicsContext(gfxc);
    camera->setDrawBuffer(GL_FRONT);
    camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    camera->attach(buffer, tex);
    return camera.release();
}

osg::Texture2D* OffscreenCaptureRig::createFloatTexture(uint width, uint height) {
    osg::ref_ptr<osg::Texture2D> tex2D = new osg::Texture2D;
    tex2D->setTextureSize(width, height);
    tex2D->setInternalFormat(GL_RGB32F_ARB);
    tex2D->setSourceFormat(GL_RGBA);
    tex2D->setSourceType(GL_FLOAT);
    tex2D->setResizeNonPowerOfTwoHint(false);
    tex2D->setFilter(osg::Texture2D::MIN_FILTER, osg::Texture2D::LINEAR);
    tex2D->setFilter(osg::Texture2D::MAG_FILTER, osg::Texture2D::LINEAR);
    return tex2D.release();
}

void OffscreenCaptureRig::setupViewer(uint width, uint height, double fovY) {
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->x = 0;
    traits->y = 0;
    traits->width = width;
    traits->height = height;
    traits->pbuffer = true;
    traits->readDISPLAY();
    osg::ref_ptr<osg::GraphicsContext> gfxc = osg::GraphicsContext::createGraphicsContext(traits.get());

    _viewer = new osgViewer::Viewer;
    osg::ref_ptr<osg::Texture2D> tex = createFloatTexture(width, height);
    osg::ref_ptr<osg::Camera> cam = createRTTCamera(_viewer->getCamera(), osg::Camera::COLOR_BUFFER0, tex, gfxc);
    cam->setProjectionMatrixAsPerspective(osg::RadiansToDegrees(fovY), (width * 1.0 / height), 0.1, 1000.0);

    _capture = new WindowCaptureScreen(gfxc, tex);
    cam->setFinalDrawCallback(_capture);
}

osg::ref_ptr<osg::Image> OffscreenCaptureRig::captureNodeImage(osg::ref_ptr<osg::Node> node) {
    _viewer->setSceneData(node);
    if (_viewer->getCamera()->getViewMatrix().isNaN()) {
        _viewer->getCamera()->setViewMatrix(osg::Matrixd::identity());
    }
    _viewer->frame();
    return _capture->captureImage();
}

WindowCaptureScreen::WindowCaptureScreen(osg::ref_ptr<osg::GraphicsContext> gfxc, osg::Texture2D* tex)
    : _mutex(new OpenThreads::Mutex()),
      _condition(new OpenThreads::Condition()),
      _ready(false),
      _image(new osg::Image()),
      _tex(tex) {
    if (gfxc.valid() && gfxc->getTraits()) {
        _image->allocateImage(gfxc->getTraits()->width, gfxc->getTraits()->height, 1, GL_RGBA, GL_FLOAT);
    }
}

WindowCaptureScreen::~WindowCaptureScreen() {
    delete _condition;
    delete _mutex;
}

osg::ref_ptr<osg::Image> WindowCaptureScreen::captureImage() {
    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(*_mutex);
    while (!_ready) {
        _condition->wait(_mutex);
    }
    _ready = false;
    return _image;
}

void WindowCaptureScreen::operator()(osg::RenderInfo& renderInfo) const {
    osg::ref_ptr<osg::GraphicsContext> gfxc = renderInfo.getState()->getGraphicsContext();
    if (!gfxc.valid() || !gfxc->getTraits()) {
        return;
    }

    OpenThreads::ScopedLock<OpenThreads::Mutex> lock(*_mutex);
    renderInfo.getState()->applyTextureAttribute(0, _tex);
    _image->readImageFromCurrentTexture(renderInfo.getContextID(), true, GL_FLOAT);
    _ready = true;
    _condition->signal();
}

} // namespace sonar_imaging
