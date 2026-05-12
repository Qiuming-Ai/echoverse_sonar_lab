#ifndef SIMULATION_NORMAL_DEPTH_MAP_SRC_IMAGECAPTURETOOL_HPP_
#define SIMULATION_NORMAL_DEPTH_MAP_SRC_IMAGECAPTURETOOL_HPP_

#include <osg/Texture2D>
#include <osgViewer/Viewer>
#include <OpenThreads/Condition>
#include <OpenThreads/Mutex>
#include <cmath>

namespace sonar_imaging {

class WindowCaptureScreen : public osg::Camera::DrawCallback {
public:
    WindowCaptureScreen(osg::ref_ptr<osg::GraphicsContext> gfxc, osg::Texture2D* tex);
    ~WindowCaptureScreen();

    osg::ref_ptr<osg::Image> captureImage();

private:
    void operator()(osg::RenderInfo& renderInfo) const;

    mutable OpenThreads::Mutex* _mutex;
    mutable OpenThreads::Condition* _condition;
    mutable bool _ready;

    osg::ref_ptr<osg::Image> _image;
    osg::ref_ptr<osg::Texture2D> _tex;
};

class OffscreenCaptureRig {
public:
    OffscreenCaptureRig(uint width = 640, uint height = 480);
    OffscreenCaptureRig(double fovY, double fovX, uint value, bool isHeight = true);

    osg::ref_ptr<osg::Image> captureNodeImage(osg::ref_ptr<osg::Node> node);

    void setViewPose(const osg::Vec3d& eye, const osg::Vec3d& center, const osg::Vec3d& up) {
        _viewer->getCamera()->setViewMatrixAsLookAt(eye, center, up);
    }

    void getViewPose(osg::Vec3d& eye, osg::Vec3d& center, osg::Vec3d& up) {
        _viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
    }

    void setBackgroundColor(osg::Vec4d color) {
        _viewer->getCamera()->setClearColor(color);
    }

protected:
    void setupViewer(uint width, uint height, double fovY = (M_PI / 3));
    osg::Texture2D* createFloatTexture(uint width, uint height);
    osg::Camera* createRTTCamera(osg::Camera* cam,
                                 osg::Camera::BufferComponent buffer,
                                 osg::Texture2D* tex,
                                 osg::GraphicsContext* gfxc);

    osg::ref_ptr<WindowCaptureScreen> _capture;
    osg::ref_ptr<osgViewer::Viewer> _viewer;
};

} // namespace sonar_imaging

#endif
