#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include <osg/ref_ptr>

namespace osg {
class Camera;
}

namespace osgViewer {
class Viewer;
}

namespace cv {
class Mat;
class VideoWriter;
}

namespace standalone_mvp {

class MainCameraCaptureCallback;

class MainCameraFileWriter {
public:
    MainCameraFileWriter();
    ~MainCameraFileWriter();
    void applyConfig(bool file_output_enabled, const std::string& file_output_path);
    void setSessionActive(bool active);
    void setOutputFps(double fps);
    void attachToCamera(osg::Camera* camera);
    void detachFromCamera();
    void close();

    bool outputEnabled() const;
    std::uint64_t framesWritten() const;

    bool writeCapturedFrameIfReady();
    bool writeFrame(const cv::Mat& bgr_frame);

private:
    bool openWriter(int width, int height);

    bool file_output_enabled_ = false;
    bool session_active_ = false;
    std::string file_output_path_;
    double output_fps_ = 20.0;
    std::unique_ptr<cv::VideoWriter> writer_;
    int writer_width_ = 0;
    int writer_height_ = 0;
    std::uint64_t frames_written_ = 0;
    osg::Camera* attached_camera_ = nullptr;
    osg::ref_ptr<MainCameraCaptureCallback> capture_callback_;
};

} // namespace standalone_mvp
