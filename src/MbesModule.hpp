#pragma once

#include "AppConfig.hpp"
#include "PointCloudSonarSimulation.hpp"
#include "PointCloudTcpStreamer.hpp"

#include <Eigen/Geometry>

#include <sonar_types_v2/echoverse_sonar_types.hpp>

#include <memory>
#include <string>
#include <QPointer>

namespace sonar_core {
class AcousticRaySimulator;
}

class SonarControlPanel;
class QTabWidget;
namespace osg {
class Group;
template <class T>
class ref_ptr;
}

namespace standalone_mvp {
class PointCloudViewerWindow;
}

class MbesModule {
public:
    explicit MbesModule(const standalone_mvp::SonarModuleConfig& module_config);
    ~MbesModule();
    MbesModule(const MbesModule&) = delete;
    MbesModule& operator=(const MbesModule&) = delete;
    MbesModule(MbesModule&&) noexcept = default;
    MbesModule& operator=(MbesModule&&) noexcept = default;
    void setModuleConfig(const standalone_mvp::SonarModuleConfig& module_config);
    void setEnvironmentConfig(const standalone_mvp::EnvironmentConfig& env_config);
    bool sonarEnabledByBinding() const;
    bool initSimulation(osg::ref_ptr<osg::Group> root, float resolution_constant);
    bool initPointCloudRuntime(
        osg::ref_ptr<osg::Group> root,
        const Eigen::Affine3d& initial_pose,
        int x,
        int y,
        const QString& project_dir,
        QTabWidget* tabs = nullptr);
    void consumePointCloudUiConfig();
    void setPointCloudRenderBlocked(bool blocked);
    void tickPointCloud(const Eigen::Affine3d& pose, bool emit_single_frame_tcp);
    void setupWidget(QTabWidget* tabs, const QString& title);
    void connectWidgetSignals();
    bool tick(const Eigen::Affine3d& pose,
              int frame_index,
              int image_update_stride,
              sonar_types_v2::samples::Sonar* out_sample = nullptr);

    std::unique_ptr<sonar_core::AcousticRaySimulator> sonar;
    QPointer<SonarControlPanel> rock_sonar_ui;
    std::unique_ptr<standalone_mvp::PointCloudSonarSimulation> bottom_cloud_sim;
    QPointer<standalone_mvp::PointCloudViewerWindow> bottom_cloud_window;
    standalone_mvp::PointCloudTcpStreamer bottom_tcp_streamer;
    standalone_mvp::PointCloudSonarConfig bottom_cfg_runtime;
    QString point_cloud_project_dir_;
    QString point_cloud_sonar_json_path_;
    float runtime_range_m = 0.0f;
    float runtime_gain = 0.0f;
    bool pending_range_update = false;
    bool pending_gain_update = false;
    float pending_range_m = 0.0f;
    float pending_gain = 0.0f;
    standalone_mvp::SonarModuleConfig module_cfg;
    standalone_mvp::EnvironmentConfig env_cfg;
};
