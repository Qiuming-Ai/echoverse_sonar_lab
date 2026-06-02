#pragma once

#include "AppConfig.hpp"
#include "Esl2dFileWriter.hpp"
#include "CameraModule.hpp"
#include "SonarOutputUtil.hpp"
#include "SonarTcpHub.hpp"

#include <Eigen/Geometry>
#include <osg/Group>
#include <osg/ref_ptr>

#include <memory>
#include <string>
#include <QPointer>

namespace sonar_core {
class AcousticRaySimulator;
}

class SideScanControlPanel;
class QWidget;

class SssModule {
public:
    explicit SssModule(const standalone_mvp::SonarModuleConfig& module_config);
    ~SssModule();
    SssModule(const SssModule&) = delete;
    SssModule& operator=(const SssModule&) = delete;
    SssModule(SssModule&&) noexcept = default;
    SssModule& operator=(SssModule&&) noexcept = default;
    void setModuleConfig(const standalone_mvp::SonarModuleConfig& module_config);
    bool sonarEnabledByBinding() const;
    void setupStripWidget(QWidget* parent_widget);
    void connectStripSignals();
    void initEsl2dRecording(const QString& project_dir);
    void beginOutputSession(standalone_mvp::SonarTcpHub* hub, const standalone_mvp::ModuleOutputSession& session);
    void endOutputSession();
    standalone_mvp::ModuleRecordingStats collectRecordingStats() const;
    bool outputSessionActive() const { return !output_session_.module_dir.isEmpty(); }
    int updateStride() const;
    void tickFromCameraRuntimes(const std::vector<SubCameraRuntime>& sub_cameras,
                                const standalone_mvp::EnvironmentConfig& env_cfg,
                                const Eigen::Vector3d& vehicle_position,
                                int frame_index,
                                int image_update_stride);

    std::unique_ptr<sonar_core::AcousticRaySimulator> sonar_a;
    std::unique_ptr<sonar_core::AcousticRaySimulator> sonar_b;
    QPointer<SideScanControlPanel> strip_widget;
    standalone_mvp::Esl2dFileWriter esl2d_file_writer_;
    QString esl2d_project_dir_;
    standalone_mvp::ModuleOutputSession output_session_;
    float runtime_range_m = 0.0f;
    float runtime_gain = 0.0f;
    standalone_mvp::SonarModuleConfig module_cfg;
    osg::ref_ptr<osg::Group> scene_a;
    osg::ref_ptr<osg::Group> scene_b;
};
