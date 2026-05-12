#include "SssModule.hpp"

#include "RockSonarPlotView.hpp"
#include "SideScanControlPanel.hpp"

#include <sonar_types_v2/echoverse_sonar_types.hpp>
#include <sonar_core/AcousticRaySimulator.hpp>

#include <QObject>
#include <QWidget>

#include <algorithm>
#include <cmath>

namespace {

Eigen::Affine3d bodyAffineFromCameraViewMatrix(const osg::Matrixd& view_matrix) {
    osg::Vec3d eye_osg;
    osg::Vec3d center_osg;
    osg::Vec3d up_osg;
    view_matrix.getLookAt(eye_osg, center_osg, up_osg, 1.0);

    const Eigen::Vector3d t(eye_osg.x(), eye_osg.y(), eye_osg.z());
    Eigen::Vector3d forward(center_osg.x() - eye_osg.x(),
                            center_osg.y() - eye_osg.y(),
                            center_osg.z() - eye_osg.z());
    if (forward.norm() < 1e-10) {
        forward = Eigen::Vector3d::UnitX();
    } else {
        forward.normalize();
    }

    const Eigen::Vector3d up_raw(up_osg.x(), up_osg.y(), up_osg.z());
    Eigen::Vector3d z = up_raw - forward * forward.dot(up_raw);
    if (z.norm() < 1e-10) {
        z = Eigen::Vector3d::UnitZ();
        z = z - forward * forward.dot(z);
    }
    z.normalize();
    const Eigen::Vector3d y = z.cross(forward).normalized();
    z = forward.cross(y).normalized();

    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = t;
    pose.linear().col(0) = forward;
    pose.linear().col(1) = y;
    pose.linear().col(2) = z;
    return pose;
}

} // namespace

SssModule::SssModule(const standalone_mvp::SonarModuleConfig& module_config)
    : runtime_range_m(static_cast<float>(module_config.sss_config.range_m)),
      runtime_gain(static_cast<float>(module_config.sss_config.gain)),
      pending_range_m(static_cast<float>(module_config.sss_config.range_m)),
      pending_gain(static_cast<float>(module_config.sss_config.gain)),
      module_cfg(module_config) {}

SssModule::~SssModule() = default;

void SssModule::setModuleConfig(const standalone_mvp::SonarModuleConfig& module_config) {
    module_cfg = module_config;
    runtime_range_m = static_cast<float>(module_cfg.sss_config.range_m);
    runtime_gain = static_cast<float>(module_cfg.sss_config.gain);
    pending_range_m = runtime_range_m;
    pending_gain = runtime_gain;
}

bool SssModule::sonarEnabledByBinding() const {
    return module_cfg.enabled &&
           module_cfg.sss_config.enabled &&
           !module_cfg.sss_camera_slot1.trimmed().isEmpty() &&
           !module_cfg.sss_camera_slot2.trimmed().isEmpty();
}

void SssModule::setupStripWidget(QWidget* parent_widget) {
    if (strip_widget) {
        strip_widget->deleteLater();
        strip_widget = nullptr;
    }
    strip_widget = new SideScanControlPanel(parent_widget);
    strip_widget->setWindowFlags(Qt::Widget);
    strip_widget->setWindowTitle(QString::fromLatin1("Side scan strip"));
    strip_widget->setMinRange(1);
    strip_widget->setMaxRange(1000);
    strip_widget->setRange(static_cast<int>(std::lround(static_cast<double>(runtime_range_m))));
    const int gain_pct = static_cast<int>(std::lround(static_cast<double>(runtime_gain) * 100.0));
    strip_widget->setGain(std::clamp(gain_pct, 0, 100));
    strip_widget->setSonarPalette(1);
}

void SssModule::connectStripSignals() {
    if (!strip_widget) {
        return;
    }
    QObject::connect(strip_widget, &SideScanControlPanel::rangeChanged, [this](int meters) {
        pending_range_m = std::clamp(static_cast<float>(meters), 1.0f, 1000.0f);
        pending_range_update = true;
    });
    QObject::connect(strip_widget, &SideScanControlPanel::gainChanged, [this](int g_pct) {
        pending_gain = std::clamp(static_cast<float>(g_pct) / 100.0f, 0.0f, 1.0f);
        pending_gain_update = true;
    });
}

void SssModule::tickFromCameraRuntimes(const std::vector<SubCameraRuntime>& sub_cameras,
                                       const standalone_mvp::EnvironmentConfig& env_cfg,
                                       const Eigen::Vector3d& vehicle_position,
                                       int frame_index,
                                       int image_update_stride) {
    if (!sonar_a || !sonar_b) {
        return;
    }
    if (pending_range_update) {
        runtime_range_m = pending_range_m;
        module_cfg.sss_config.range_m = static_cast<double>(runtime_range_m);
        pending_range_update = false;
    }
    if (pending_gain_update) {
        runtime_gain = pending_gain;
        module_cfg.sss_config.gain = static_cast<double>(runtime_gain);
        pending_gain_update = false;
    }
    const auto it_slot1 = std::find_if(sub_cameras.begin(), sub_cameras.end(), [&](const SubCameraRuntime& sc) {
        return sc.name == module_cfg.sss_camera_slot1.toStdString();
    });
    const auto it_slot2 = std::find_if(sub_cameras.begin(), sub_cameras.end(), [&](const SubCameraRuntime& sc) {
        return sc.name == module_cfg.sss_camera_slot2.toStdString();
    });
    if (it_slot1 == sub_cameras.end() || it_slot2 == sub_cameras.end() || !it_slot1->camera || !it_slot2->camera) {
        return;
    }

    const Eigen::Affine3d pose_a = bodyAffineFromCameraViewMatrix(it_slot1->camera->getViewMatrix());
    const Eigen::Affine3d pose_b = bodyAffineFromCameraViewMatrix(it_slot2->camera->getViewMatrix());
    const double depth_m = std::max(0.0, -vehicle_position.z());
    sonar_a->setRange(runtime_range_m);
    sonar_b->setRange(runtime_range_m);
    sonar_a->setGain(runtime_gain);
    sonar_b->setGain(runtime_gain);
    sonar_a->setAttenuationCoefficient(module_cfg.sss_config.center_frequency_khz, env_cfg.temperature_c,
                                       depth_m, env_cfg.salinity_ppt, env_cfg.acidity_ph,
                                       env_cfg.enable_attenuation);
    sonar_b->setAttenuationCoefficient(module_cfg.sss_config.center_frequency_khz, env_cfg.temperature_c,
                                       depth_m, env_cfg.salinity_ppt, env_cfg.acidity_ph,
                                       env_cfg.enable_attenuation);
    sonar_types_v2::samples::Sonar sa = sonar_a->renderPing(pose_a);
    standalone_mvp::finalizeMultibeamSonarSample(sa, sonar_a->getSonarBeamWidth(), sonar_a->getSonarBeamCount());
    standalone_mvp::validateSonarSample(sa);
    sonar_types_v2::samples::Sonar sb = sonar_b->renderPing(pose_b);
    standalone_mvp::finalizeMultibeamSonarSample(sb, sonar_b->getSonarBeamWidth(), sonar_b->getSonarBeamCount());
    standalone_mvp::validateSonarSample(sb);
    if (strip_widget && (frame_index % std::max(1, image_update_stride)) == 0) {
        strip_widget->setPortStarboardData(sa, sb);
    }
}
