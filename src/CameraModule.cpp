#include "CameraModule.hpp"

#include <QImage>
#include <QPixmap>

#include <osg/Geode>
#include <osg/Shape>
#include <osg/ShapeDrawable>

#include <algorithm>
#include <cmath>

namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

osg::ref_ptr<osg::MatrixTransform> createCameraMarkerNode(const osg::Vec4& color) {
    constexpr float kConeLength = 1.2f;
    constexpr float kConeRadius = 0.28f;
    osg::ref_ptr<osg::Cone> cone = new osg::Cone(osg::Vec3(0.0f, 0.0f, -kConeLength * 0.5f), kConeRadius, kConeLength);
    osg::ref_ptr<osg::ShapeDrawable> cone_drawable = new osg::ShapeDrawable(cone.get());
    cone_drawable->setColor(color);
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(cone_drawable.get());
    osg::StateSet* ss = geode->getOrCreateStateSet();
    ss->setMode(GL_LIGHTING, osg::StateAttribute::ON);
    ss->setMode(GL_CULL_FACE, osg::StateAttribute::ON);
    osg::ref_ptr<osg::MatrixTransform> local_align = new osg::MatrixTransform();
    local_align->setMatrix(osg::Matrix::rotate(-osg::PI_2, osg::Vec3(0.0f, 1.0f, 0.0f)));
    local_align->addChild(geode.get());
    osg::ref_ptr<osg::MatrixTransform> xform = new osg::MatrixTransform();
    xform->addChild(local_align.get());
    return xform;
}

Eigen::Quaterniond quaternionFromPose(double yaw, double pitch) {
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
}

Eigen::Quaterniond quaternionWithOffset(double yaw, double pitch, const standalone_mvp::SubCameraConfig& cfg) {
    const Eigen::Quaterniond q_main = quaternionFromPose(yaw, pitch);
    const Eigen::Quaterniond q_offset =
        Eigen::AngleAxisd(cfg.yaw_offset_deg * kDegToRad, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(cfg.pitch_offset_deg * kDegToRad, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(cfg.roll_offset_deg * kDegToRad, Eigen::Vector3d::UnitX());
    return q_main * q_offset;
}

void setCameraViewFromPositionAndQuaternion(osg::Camera* camera,
                                            const Eigen::Vector3d& position,
                                            const Eigen::Quaterniond& q) {
    if (!camera) {
        return;
    }
    const Eigen::Vector3d forward = q * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d up = q * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d look_at = position + forward;
    camera->setViewMatrixAsLookAt(
        osg::Vec3d(position.x(), position.y(), position.z()),
        osg::Vec3d(look_at.x(), look_at.y(), look_at.z()),
        osg::Vec3d(up.x(), up.y(), up.z()));
}

} // namespace

CameraModule::CameraModule(const standalone_mvp::AppConfigData& app_cfg) {
    for (std::size_t i = 0; i < app_cfg.camera_system.sub_cameras.size(); ++i) {
        standalone_mvp::SubCameraConfig cfg = app_cfg.camera_system.sub_cameras[i];
        if (cfg.name.trimmed().isEmpty()) {
            cfg.name = QString("Camera %1").arg(static_cast<int>(i + 1));
        }
        sub_names.push_back(cfg.name.toStdString());
        sub_configs.push_back(cfg);
        labels.push_back(nullptr);
    }
}

CameraModule::~CameraModule() = default;

const standalone_mvp::SubCameraConfig* CameraModule::findSubCameraByName(const std::string& name) const {
    if (name.empty()) {
        return nullptr;
    }
    for (std::size_t i = 0; i < sub_names.size(); ++i) {
        if (sub_names[i] == name) {
            return &sub_configs[i];
        }
    }
    return nullptr;
}

void CameraModule::buildSubCameras(osgViewer::Viewer& viewer,
                                   osg::Group* camera_debug_group,
                                   unsigned int scene_mask,
                                   const std::string& slot1_name,
                                   const std::string& slot2_name) {
    sub_cameras.clear();
    if (!camera_debug_group) {
        return;
    }
    for (std::size_t i = 0; i < sub_configs.size(); ++i) {
        const auto& cfg = sub_configs[i];
        QLabel* label = i < labels.size() ? labels[i] : nullptr;
        if (!cfg.enabled) {
            if (label) label->setText("disabled");
            continue;
        }
        osg::ref_ptr<osg::Camera> cam = new osg::Camera();
        cam->setGraphicsContext(viewer.getCamera()->getGraphicsContext());
        cam->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
        cam->setRenderOrder(osg::Camera::POST_RENDER);
        cam->setCullMask(scene_mask);
        cam->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        cam->setAllowEventFocus(false);
        cam->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
        cam->setClearColor(viewer.getCamera()->getClearColor());
        osg::ref_ptr<osg::Image> image = new osg::Image();
        image->allocateImage(320, 180, 1, GL_RGBA, GL_UNSIGNED_BYTE);
        cam->attach(osg::Camera::COLOR_BUFFER, image.get());
        cam->setViewMatrix(viewer.getCamera()->getViewMatrix());
        cam->setProjectionMatrixAsPerspective(cfg.vertical_fov_deg, 1.0, 0.1, 10000.0);
        viewer.addSlave(cam.get(), osg::Matrixd(), osg::Matrixd(), true);
        SubCameraRuntime runtime;
        runtime.name = sub_names[i];
        runtime.config = cfg;
        runtime.camera = cam;
        runtime.image = image;
        runtime.label = label;
        if (runtime.name == slot1_name) {
            runtime.marker_xform = createCameraMarkerNode(osg::Vec4(0.95f, 0.2f, 0.2f, 1.0f));
        } else if (runtime.name == slot2_name) {
            runtime.marker_xform = createCameraMarkerNode(osg::Vec4(0.2f, 0.95f, 0.2f, 1.0f));
        } else {
            runtime.marker_xform = createCameraMarkerNode(osg::Vec4(0.2f, 0.6f, 1.0f, 1.0f));
        }
        camera_debug_group->addChild(runtime.marker_xform.get());
        if (label) label->setText("running");
        sub_cameras.push_back(runtime);
    }
    main_camera_marker = createCameraMarkerNode(osg::Vec4(1.0f, 1.0f, 0.25f, 1.0f));
    camera_debug_group->addChild(main_camera_marker.get());
}

void CameraModule::updateViews(const Eigen::Vector3d& position, double yaw, double pitch) {
    const Eigen::Quaterniond q_main = quaternionFromPose(yaw, pitch);
    if (main_camera_marker.valid()) {
        const osg::Quat main_q(static_cast<float>(q_main.x()), static_cast<float>(q_main.y()),
                               static_cast<float>(q_main.z()), static_cast<float>(q_main.w()));
        main_camera_marker->setMatrix(
            osg::Matrix::rotate(main_q) *
            osg::Matrix::translate(position.x(), position.y(), position.z()));
    }
    for (auto& sc : sub_cameras) {
        const Eigen::Quaterniond q_sub = quaternionWithOffset(yaw, pitch, sc.config);
        setCameraViewFromPositionAndQuaternion(sc.camera.get(), position, q_sub);
        const double aspect = std::tan(sc.config.horizontal_fov_deg * kDegToRad * 0.5) /
                              std::tan(sc.config.vertical_fov_deg * kDegToRad * 0.5);
        sc.camera->setProjectionMatrixAsPerspective(sc.config.vertical_fov_deg, std::max(0.1, aspect), 0.1, 10000.0);
        if (sc.marker_xform.valid()) {
            const osg::Quat q(static_cast<float>(q_sub.x()), static_cast<float>(q_sub.y()),
                              static_cast<float>(q_sub.z()), static_cast<float>(q_sub.w()));
            sc.marker_xform->setMatrix(
                osg::Matrix::rotate(q) *
                osg::Matrix::translate(position.x(), position.y(), position.z()));
        }
    }
}

void CameraModule::updateWidgets() {
    for (auto& sc : sub_cameras) {
        if (!sc.label || !sc.image.valid() || !sc.image->data() || sc.image->s() <= 0 || sc.image->t() <= 0) {
            continue;
        }
        const QSize target_size = sc.label->size();
        if (target_size.width() <= 0 || target_size.height() <= 0) {
            continue;
        }
        const int bytes_per_line = std::max(1, static_cast<int>(sc.image->getRowSizeInBytes()));
        const QImage img(sc.image->data(), sc.image->s(), sc.image->t(), bytes_per_line, QImage::Format_RGBA8888);
        const QImage stable = img.copy().mirrored();
        sc.label->setPixmap(QPixmap::fromImage(stable).scaled(
            target_size, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    }
}

void CameraModule::updateViewports() {
    for (auto& sc : sub_cameras) {
        const int tw = std::max(1, sc.label ? sc.label->width() : 320);
        const int th = std::max(1, sc.label ? sc.label->height() : 180);
        const double source_aspect = std::max(
            0.1,
            std::tan(sc.config.horizontal_fov_deg * kDegToRad * 0.5) /
                std::tan(sc.config.vertical_fov_deg * kDegToRad * 0.5));
        int rw = tw;
        int rh = static_cast<int>(std::lround(static_cast<double>(rw) / source_aspect));
        if (rh > th) {
            rh = th;
            rw = static_cast<int>(std::lround(static_cast<double>(rh) * source_aspect));
        }
        rw = std::max(1, rw);
        rh = std::max(1, rh);
        sc.camera->setViewport(new osg::Viewport(0, 0, rw, rh));
        if (sc.image.valid() && (sc.image->s() != rw || sc.image->t() != rh)) {
            sc.image->allocateImage(rw, rh, 1, GL_RGBA, GL_UNSIGNED_BYTE);
            sc.camera->attach(osg::Camera::COLOR_BUFFER, sc.image.get());
        }
    }
}

const SubCameraRuntime* CameraModule::findRuntimeByName(const std::string& name) const {
    for (const auto& sc : sub_cameras) {
        if (sc.name == name) {
            return &sc;
        }
    }
    return nullptr;
}
