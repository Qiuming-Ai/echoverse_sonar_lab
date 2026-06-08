#include "SssModule.hpp"

#include "Esl2dFileWriter.hpp"
#include "RockSonarPlotView.hpp"
#include "SideScanControlPanel.hpp"

#include <sonar_types_v2/echoverse_sonar_types.hpp>
#include <sonar_core/AcousticRaySimulator.hpp>

#include <QObject>
#include <QWidget>

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace {

double computeYawDeg(const Eigen::Vector3d& forward) {
    return std::atan2(forward.y(), forward.x()) * 180.0 / M_PI;
}

double computePitchDeg(const Eigen::Vector3d& forward) {
    const double horiz = std::hypot(forward.x(), forward.y());
    return std::atan2(forward.z(), horiz) * 180.0 / M_PI;
}

standalone_mvp::Esl2dPoseSnapshot poseFromAffine(const Eigen::Affine3d& pose) {
    standalone_mvp::Esl2dPoseSnapshot out;
    out.x = pose.translation().x();
    out.y = pose.translation().y();
    out.z = pose.translation().z();
    Eigen::Vector3d forward = pose.linear().col(0);
    if (forward.norm() > 1e-10) {
        forward.normalize();
    } else {
        forward = Eigen::Vector3d::UnitX();
    }
    out.yaw_deg = computeYawDeg(forward);
    out.pitch_deg = computePitchDeg(forward);
    const Eigen::Quaterniond q(pose.linear());
    out.quat_w = q.w();
    out.quat_x = q.x();
    out.quat_y = q.y();
    out.quat_z = q.z();
    return out;
}

bool esl2dOutputEnabled(bool config_enabled) {
    return config_enabled || (std::getenv("STANDALONE_ESL2D_OUTPUT") != nullptr);
}

QJsonObject sssSonarConfigJson(const standalone_mvp::SideScanSonarConfigUi& cfg) {
    QJsonObject o;
    o["range_m"] = cfg.range_m;
    o["gain"] = cfg.gain;
    o["center_frequency_khz"] = cfg.center_frequency_khz;
    o["bandwidth_khz"] = cfg.bandwidth_khz;
    o["beam_width_deg"] = cfg.beam_width_deg;
    o["beam_height_deg"] = cfg.beam_height_deg;
    o["angular_resolution_deg"] = cfg.angular_resolution_deg;
    o["update_stride"] = cfg.update_stride;
    return o;
}

QJsonObject environmentConfigJson(const standalone_mvp::EnvironmentConfig& cfg) {
    QJsonObject o;
    o["temperature_c"] = cfg.temperature_c;
    o["salinity_ppt"] = cfg.salinity_ppt;
    o["acidity_ph"] = cfg.acidity_ph;
    o["enable_reverb"] = cfg.enable_reverb;
    o["enable_speckle"] = cfg.enable_speckle;
    o["enable_attenuation"] = cfg.enable_attenuation;
    o["enable_logistic_response"] = cfg.enable_logistic_response;
    o["sound_speed_mps"] = cfg.sound_speed_mps;
    return o;
}

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

unsigned int computeDerivedBinCount(const standalone_mvp::SonarConfigUi& s) {
    const double bandwidth_hz = std::max(1.0, s.bandwidth_khz * 1000.0);
    const double range_m = std::max(0.1, s.range_m);
    const double c_mps = std::max(1.0, s.sound_speed_mps);
    const double value = (bandwidth_hz * 2.0 * range_m) / c_mps;
    return static_cast<unsigned int>(std::max(1.0, std::floor(value)));
}

unsigned int computeDerivedBeamCount(const standalone_mvp::SonarConfigUi& s) {
    const double beam_w = std::max(standalone_mvp::kMinSonarBeamDeg, s.beam_width_deg);
    const double ang_res = std::max(0.01, s.angular_resolution_deg);
    const double value = beam_w / ang_res;
    return static_cast<unsigned int>(std::max(1.0, std::floor(value)));
}

standalone_mvp::SonarConfigUi sonarConfigFromSss(const standalone_mvp::SideScanSonarConfigUi& ss) {
    standalone_mvp::SonarConfigUi s;
    s.range_m = ss.range_m;
    s.gain = ss.gain;
    s.center_frequency_khz = ss.center_frequency_khz;
    s.bandwidth_khz = std::min(ss.bandwidth_khz, ss.center_frequency_khz - 0.1);
    s.angular_resolution_deg = std::clamp(ss.angular_resolution_deg, 0.05, 30.0);
    s.beam_width_deg = std::clamp(ss.beam_width_deg, standalone_mvp::kMinSonarBeamDeg, standalone_mvp::kMaxSonarBeamDeg);
    s.beam_height_deg = std::clamp(ss.beam_height_deg, standalone_mvp::kMinSonarBeamDeg, standalone_mvp::kMaxSonarBeamDeg);
    s.attenuation_frequency_khz = ss.center_frequency_khz;
    return s;
}

void applySssConfigToSimulators(standalone_mvp::SideScanSonarConfigUi& sss_cfg,
                                std::unique_ptr<sonar_core::AcousticRaySimulator>& sonar_a,
                                std::unique_ptr<sonar_core::AcousticRaySimulator>& sonar_b,
                                float& runtime_range_m,
                                float& runtime_gain) {
    const double safe_range_m = std::clamp(sss_cfg.range_m, 0.1, 1000.0);
    const double safe_gain = std::clamp(sss_cfg.gain, 0.0, 1.0);
    const double safe_center_frequency_khz = std::clamp(sss_cfg.center_frequency_khz, 1.0, 2000.0);
    const double safe_bandwidth_khz =
        std::clamp(sss_cfg.bandwidth_khz, 0.1, std::max(0.1, safe_center_frequency_khz - 0.1));
    const double safe_beam_width_deg =
        std::clamp(sss_cfg.beam_width_deg, standalone_mvp::kMinSonarBeamDeg, standalone_mvp::kMaxSonarBeamDeg);
    const double safe_beam_height_deg =
        std::clamp(sss_cfg.beam_height_deg, standalone_mvp::kMinSonarBeamDeg, standalone_mvp::kMaxSonarBeamDeg);
    const double safe_angle_resolution_deg = std::clamp(sss_cfg.angular_resolution_deg, 0.01, 30.0);
    const int safe_update_stride = std::clamp(sss_cfg.update_stride, 1, 30);

    sss_cfg.range_m = safe_range_m;
    sss_cfg.gain = safe_gain;
    sss_cfg.center_frequency_khz = safe_center_frequency_khz;
    sss_cfg.bandwidth_khz = safe_bandwidth_khz;
    sss_cfg.beam_width_deg = safe_beam_width_deg;
    sss_cfg.beam_height_deg = safe_beam_height_deg;
    sss_cfg.angular_resolution_deg = safe_angle_resolution_deg;
    sss_cfg.update_stride = safe_update_stride;

    runtime_range_m = static_cast<float>(safe_range_m);
    runtime_gain = static_cast<float>(safe_gain);

    const standalone_mvp::SonarConfigUi derived = sonarConfigFromSss(sss_cfg);
    const unsigned int bin_count = computeDerivedBinCount(derived);
    const unsigned int beam_count = computeDerivedBeamCount(derived);

    if (sonar_a) {
        sonar_a->setRange(runtime_range_m);
        sonar_a->setGain(runtime_gain);
        sonar_a->setSonarBinCount(bin_count);
        sonar_a->setSonarBeamCount(beam_count);
        sonar_a->setSonarBeamWidth(
            sonar_types_v2::Angle::fromDeg(static_cast<float>(sss_cfg.beam_width_deg)));
        sonar_a->setSonarBeamHeight(
            sonar_types_v2::Angle::fromDeg(static_cast<float>(sss_cfg.beam_height_deg)));
    }
    if (sonar_b) {
        sonar_b->setRange(runtime_range_m);
        sonar_b->setGain(runtime_gain);
        sonar_b->setSonarBinCount(bin_count);
        sonar_b->setSonarBeamCount(beam_count);
        sonar_b->setSonarBeamWidth(
            sonar_types_v2::Angle::fromDeg(static_cast<float>(sss_cfg.beam_width_deg)));
        sonar_b->setSonarBeamHeight(
            sonar_types_v2::Angle::fromDeg(static_cast<float>(sss_cfg.beam_height_deg)));
    }
}

} // namespace

SssModule::SssModule(const standalone_mvp::SonarModuleConfig& module_config)
    : runtime_range_m(static_cast<float>(module_config.sss_config.range_m)),
      runtime_gain(static_cast<float>(module_config.sss_config.gain)),
      module_cfg(module_config) {}

SssModule::~SssModule() = default;

void SssModule::setModuleConfig(const standalone_mvp::SonarModuleConfig& module_config) {
    module_cfg = module_config;
    runtime_range_m = static_cast<float>(module_cfg.sss_config.range_m);
    runtime_gain = static_cast<float>(module_cfg.sss_config.gain);
}

bool SssModule::sonarEnabledByBinding() const {
    return module_cfg.enabled &&
           module_cfg.sss_config.enabled &&
           !module_cfg.sss_camera_slot1.trimmed().isEmpty() &&
           !module_cfg.sss_camera_slot2.trimmed().isEmpty();
}

int SssModule::updateStride() const {
    return std::max(1, module_cfg.sss_config.update_stride);
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
    strip_widget->setAdvancedPanelEnabled(true);
    strip_widget->setRange(static_cast<int>(std::lround(static_cast<double>(runtime_range_m))));
    const int gain_pct = static_cast<int>(std::lround(static_cast<double>(runtime_gain) * 100.0));
    strip_widget->setGain(std::clamp(gain_pct, 0, 100));
    strip_widget->setAdvancedSssConfig(
        module_cfg.sss_config.range_m,
        module_cfg.sss_config.gain,
        module_cfg.sss_config.center_frequency_khz,
        module_cfg.sss_config.bandwidth_khz,
        module_cfg.sss_config.beam_width_deg,
        module_cfg.sss_config.beam_height_deg,
        module_cfg.sss_config.angular_resolution_deg,
        module_cfg.sss_config.update_stride,
        module_cfg.sss_config.tcp_output_enabled,
        module_cfg.sss_config.file_output_enabled,
        module_cfg.sss_config.tcp_host,
        module_cfg.sss_config.tcp_port);
    strip_widget->setSonarPalette(1);
}

void SssModule::initEsl2dRecording(const QString& project_dir) {
    esl2d_project_dir_ = project_dir;
}

void SssModule::beginOutputSession(standalone_mvp::SonarTcpHub* hub,
                                   const standalone_mvp::ModuleOutputSession& session) {
    output_session_ = session;
    esl2d_file_writer_.resetFrameCount();
    esl2d_file_writer_.setTcpHub(hub);
    esl2d_file_writer_.setSessionActive(true);

    const auto& c = module_cfg.sss_config;
    esl2d_file_writer_.applyConfig(
        c.tcp_output_enabled,
        c.tcp_host.toStdString(),
        static_cast<std::uint16_t>(std::clamp(c.tcp_port, 1, 65535)),
        c.file_output_enabled,
        session.esl2d_path.toStdString());
}

void SssModule::endOutputSession() {
    esl2d_file_writer_.setSessionActive(false);
    esl2d_file_writer_.close();
    output_session_ = {};
}

standalone_mvp::ModuleRecordingStats SssModule::collectRecordingStats() const {
    standalone_mvp::ModuleRecordingStats stats;
    stats.module_name = module_cfg.name;
    stats.type = standalone_mvp::SonarModuleType::SSS;
    stats.config = module_cfg;
    stats.esl2d_frames = esl2d_file_writer_.framesWritten();
    return stats;
}

void SssModule::connectStripSignals() {
    if (!strip_widget) {
        return;
    }
    QObject::connect(
        strip_widget,
        &SideScanControlPanel::advancedSssConfigChanged,
        [this](double range_m,
               double gain,
               double center_frequency_khz,
               double bandwidth_khz,
               double beam_width_deg,
               double beam_height_deg,
               double angle_resolution_deg,
               int update_stride,
               bool tcp_output_enabled,
               bool file_output_enabled,
               const QString& tcp_host,
               int tcp_port) {
            module_cfg.sss_config.range_m = range_m;
            module_cfg.sss_config.gain = gain;
            module_cfg.sss_config.center_frequency_khz = center_frequency_khz;
            module_cfg.sss_config.bandwidth_khz = bandwidth_khz;
            module_cfg.sss_config.beam_width_deg = beam_width_deg;
            module_cfg.sss_config.beam_height_deg = beam_height_deg;
            module_cfg.sss_config.angular_resolution_deg = angle_resolution_deg;
            module_cfg.sss_config.update_stride = update_stride;
            module_cfg.sss_config.tcp_output_enabled = tcp_output_enabled;
            module_cfg.sss_config.file_output_enabled = file_output_enabled;
            module_cfg.sss_config.tcp_host = tcp_host.trimmed().isEmpty() ? QStringLiteral("0.0.0.0") : tcp_host.trimmed();
            module_cfg.sss_config.tcp_port = std::clamp(tcp_port, 1, 65535);
            applySssConfigToSimulators(module_cfg.sss_config, sonar_a, sonar_b, runtime_range_m, runtime_gain);
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
    if (esl2d_file_writer_.outputEnabled()) {
        Eigen::Affine3d vehicle_pose = Eigen::Affine3d::Identity();
        vehicle_pose.translation() = vehicle_position;
        vehicle_pose.linear() = pose_b.linear();
        standalone_mvp::Esl2dWriteParams params;
        params.sonar_kind = standalone_mvp::Esl2dSonarKind::SSS;
        params.max_range_m = runtime_range_m;
        params.pose = poseFromAffine(vehicle_pose);
        params.sonar_config = sssSonarConfigJson(module_cfg.sss_config);
        params.environment = environmentConfigJson(env_cfg);
        params.sonar_module_name = module_cfg.name;
        esl2d_file_writer_.writeSssFrame(sb, sa, params);
    }
}
