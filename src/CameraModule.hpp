#pragma once

#include "AppConfig.hpp"

#include <QLabel>

#include <Eigen/Geometry>

#include <osg/Camera>
#include <osg/Group>
#include <osg/Image>
#include <osg/Matrixd>
#include <osg/MatrixTransform>
#include <osg/Viewport>
#include <osg/ref_ptr>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

#include <string>
#include <vector>

struct SubCameraRuntime {
    std::string name;
    standalone_mvp::SubCameraConfig config;
    osg::ref_ptr<osg::Camera> camera;
    osg::ref_ptr<osg::Image> image;
    QLabel* label = nullptr;
    osg::ref_ptr<osg::MatrixTransform> marker_xform;
};

class CameraModule {
public:
    explicit CameraModule(const standalone_mvp::AppConfigData& app_cfg);
    ~CameraModule();

    const standalone_mvp::SubCameraConfig* findSubCameraByName(const std::string& name) const;
    void buildSubCameras(osgViewer::Viewer& viewer,
                         osg::Group* camera_debug_group,
                         unsigned int scene_mask,
                         const std::string& slot1_name,
                         const std::string& slot2_name);
    void updateViews(const Eigen::Vector3d& position, double yaw, double pitch);
    void updateWidgets();
    void updateViewports();
    const SubCameraRuntime* findRuntimeByName(const std::string& name) const;

    std::vector<std::string> sub_names;
    std::vector<standalone_mvp::SubCameraConfig> sub_configs;
    std::vector<QLabel*> labels;
    std::vector<SubCameraRuntime> sub_cameras;
    osg::ref_ptr<osg::MatrixTransform> main_camera_marker;
};
