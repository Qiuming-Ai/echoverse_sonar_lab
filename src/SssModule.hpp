#pragma once

#include "AppConfig.hpp"
#include "CameraModule.hpp"

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
    void tickFromCameraRuntimes(const std::vector<SubCameraRuntime>& sub_cameras,
                                const standalone_mvp::EnvironmentConfig& env_cfg,
                                const Eigen::Vector3d& vehicle_position,
                                int frame_index,
                                int image_update_stride);

    std::unique_ptr<sonar_core::AcousticRaySimulator> sonar_a;
    std::unique_ptr<sonar_core::AcousticRaySimulator> sonar_b;
    QPointer<SideScanControlPanel> strip_widget;
    float runtime_range_m = 0.0f;
    float runtime_gain = 0.0f;
    bool pending_range_update = false;
    bool pending_gain_update = false;
    float pending_range_m = 0.0f;
    float pending_gain = 0.0f;
    standalone_mvp::SonarModuleConfig module_cfg;
    osg::ref_ptr<osg::Group> scene_a;
    osg::ref_ptr<osg::Group> scene_b;
};
