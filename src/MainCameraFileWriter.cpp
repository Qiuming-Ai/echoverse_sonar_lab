#include "MainCameraFileWriter.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <osg/Camera>
#include <osg/Image>
#include <osgViewer/Viewer>

#include <OpenThreads/Mutex>

#include <algorithm>
#include <iostream>
#include <utility>

namespace standalone_mvp {

MainCameraFileWriter::MainCameraFileWriter() = default;
MainCameraFileWriter::~MainCameraFileWriter() = default;

class MainCameraCaptureCallback : public osg::Camera::DrawCallback {
public:
    bool takeFrame(cv::Mat& out_bgr) {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(mutex_);
        if (!has_frame_) {
            return false;
        }
        out_bgr = pending_bgr_.clone();
        has_frame_ = false;
        return !out_bgr.empty();
    }

    void operator()(osg::RenderInfo& renderInfo) const override {
        const osg::Camera* camera = renderInfo.getCurrentCamera();
        const osg::Viewport* viewport = camera ? camera->getViewport() : nullptr;
        if (!viewport) {
            return;
        }
        const int width = static_cast<int>(viewport->width());
        const int height = static_cast<int>(viewport->height());
        if (width <= 0 || height <= 0) {
            return;
        }

        osg::ref_ptr<osg::Image> image = new osg::Image();
        image->readPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE);
        if (!image.valid() || image->data() == nullptr) {
            return;
        }

        cv::Mat rgba(height, width, CV_8UC4, image->data());
        cv::Mat flipped;
        cv::flip(rgba, flipped, 0);
        cv::Mat bgr;
        cv::cvtColor(flipped, bgr, cv::COLOR_RGBA2BGR);

        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(mutex_);
        pending_bgr_ = std::move(bgr);
        has_frame_ = true;
    }

private:
    mutable OpenThreads::Mutex mutex_;
    mutable cv::Mat pending_bgr_;
    mutable bool has_frame_ = false;
};

void MainCameraFileWriter::applyConfig(bool file_output_enabled, const std::string& file_output_path) {
    file_output_enabled_ = file_output_enabled;
    file_output_path_ = file_output_path;
    if (!file_output_enabled_) {
        detachFromCamera();
        close();
    }
}

void MainCameraFileWriter::setSessionActive(bool active) {
    session_active_ = active;
    if (!session_active_) {
        detachFromCamera();
        close();
    } else {
        frames_written_ = 0;
    }
}

void MainCameraFileWriter::attachToCamera(osg::Camera* camera) {
    detachFromCamera();
    if (!camera) {
        return;
    }
    if (!capture_callback_) {
        capture_callback_ = new MainCameraCaptureCallback();
    }
    camera->setFinalDrawCallback(capture_callback_.get());
    attached_camera_ = camera;
}

void MainCameraFileWriter::detachFromCamera() {
    if (attached_camera_) {
        attached_camera_->setFinalDrawCallback(nullptr);
        attached_camera_ = nullptr;
    }
}

void MainCameraFileWriter::setOutputFps(double fps) {
    output_fps_ = std::clamp(fps, 1.0, 240.0);
}

void MainCameraFileWriter::close() {
    if (writer_) {
        writer_->release();
        writer_.reset();
        if (frames_written_ > 0) {
            std::cout << "[main_camera] closed path=" << file_output_path_ << " frames=" << frames_written_
                      << std::endl;
        }
    }
    writer_width_ = 0;
    writer_height_ = 0;
}

bool MainCameraFileWriter::outputEnabled() const {
    return file_output_enabled_ && session_active_ && !file_output_path_.empty();
}

std::uint64_t MainCameraFileWriter::framesWritten() const {
    return frames_written_;
}

bool MainCameraFileWriter::writeCapturedFrameIfReady() {
    if (!outputEnabled() || !capture_callback_) {
        return false;
    }
    cv::Mat frame_bgr;
    if (!capture_callback_->takeFrame(frame_bgr)) {
        return false;
    }
    return writeFrame(frame_bgr);
}

bool MainCameraFileWriter::openWriter(int width, int height) {
    close();
    const cv::Size frame_size(width, height);
    writer_ = std::make_unique<cv::VideoWriter>(
        file_output_path_,
        cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
        output_fps_,
        frame_size,
        true);
    if (!writer_->isOpened()) {
        std::cout << "[main_camera] video writer open failed path=" << file_output_path_ << std::endl;
        writer_.reset();
        writer_width_ = 0;
        writer_height_ = 0;
        return false;
    }
    writer_width_ = width;
    writer_height_ = height;
    std::cout << "[main_camera] recording path=" << file_output_path_ << " size=" << writer_width_ << "x"
              << writer_height_ << " fps=" << output_fps_ << std::endl;
    return true;
}

bool MainCameraFileWriter::writeFrame(const cv::Mat& bgr_frame) {
    if (!outputEnabled() || bgr_frame.empty()) {
        return false;
    }

    if (!writer_ || bgr_frame.cols != writer_width_ || bgr_frame.rows != writer_height_) {
        if (!openWriter(bgr_frame.cols, bgr_frame.rows)) {
            return false;
        }
    }

    writer_->write(bgr_frame);
    ++frames_written_;
    return true;
}

} // namespace standalone_mvp
