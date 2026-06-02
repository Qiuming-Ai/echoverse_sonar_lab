#include "MbesModule.hpp"

#include "RockSonarPlotView.hpp"
#include "PointCloudViewerWindow.hpp"
#include "SonarControlPanel.hpp"
#include "SonarOutputUtil.hpp"
#include "ui/DockWorkspace.hpp"

#include <QApplication>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProgressDialog>
#include <QProcess>
#include <QRegularExpression>

#include <sonar_types_v2/echoverse_math_types.hpp>
#include <sonar_core/AcousticRaySimulator.hpp>

#include <osg/Group>
#include <osg/ref_ptr>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

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

QJsonObject mbesSonarConfigJson(const standalone_mvp::SonarConfigUi& cfg) {
    QJsonObject o;
    o["range_m"] = cfg.range_m;
    o["gain"] = cfg.gain;
    o["center_frequency_khz"] = cfg.center_frequency_khz;
    o["bandwidth_khz"] = cfg.bandwidth_khz;
    o["beam_width_deg"] = cfg.beam_width_deg;
    o["beam_height_deg"] = cfg.beam_height_deg;
    o["angular_resolution_deg"] = cfg.angular_resolution_deg;
    o["beam_count"] = cfg.beam_count;
    o["bin_count"] = cfg.bin_count;
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
    o["sound_speed_mps"] = cfg.sound_speed_mps;
    return o;
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

QString buildPointCloudOutputPath(const QString& project_dir, const QString& sonar_name) {
    (void)project_dir;
    (void)sonar_name;
    return QString();
}

} // namespace

MbesModule::MbesModule(const standalone_mvp::SonarModuleConfig& module_config)
    : runtime_range_m(static_cast<float>(module_config.mbes_config.range_m)),
      runtime_gain(static_cast<float>(module_config.mbes_config.gain)),
      pending_range_m(static_cast<float>(module_config.mbes_config.range_m)),
      pending_gain(static_cast<float>(module_config.mbes_config.gain)),
      module_cfg(module_config) {}

MbesModule::~MbesModule() = default;

void MbesModule::setModuleConfig(const standalone_mvp::SonarModuleConfig& module_config) {
    module_cfg = module_config;
    runtime_range_m = static_cast<float>(module_cfg.mbes_config.range_m);
    runtime_gain = static_cast<float>(module_cfg.mbes_config.gain);
    pending_range_m = runtime_range_m;
    pending_gain = runtime_gain;
}

void MbesModule::setEnvironmentConfig(const standalone_mvp::EnvironmentConfig& env_config) {
    env_cfg = env_config;
}

bool MbesModule::sonarEnabledByBinding() const {
    return module_cfg.enabled &&
           module_cfg.mbes_config.enable_2d_fls &&
           !module_cfg.camera_binding.trimmed().isEmpty();
}

bool MbesModule::initSimulation(osg::ref_ptr<osg::Group> root, float resolution_constant) {
    if (!sonarEnabledByBinding()) {
        sonar.reset();
        return false;
    }
    const unsigned int bin_count = computeDerivedBinCount(module_cfg.mbes_config);
    const unsigned int beam_count = computeDerivedBeamCount(module_cfg.mbes_config);
    const unsigned int resolution = static_cast<unsigned int>(static_cast<float>(bin_count) * resolution_constant);
    sonar = std::make_unique<sonar_core::AcousticRaySimulator>(
        static_cast<float>(module_cfg.mbes_config.range_m),
        static_cast<float>(module_cfg.mbes_config.gain),
        bin_count,
        sonar_types_v2::Angle::fromDeg(static_cast<float>(module_cfg.mbes_config.beam_width_deg)),
        sonar_types_v2::Angle::fromDeg(static_cast<float>(module_cfg.mbes_config.beam_height_deg)),
        resolution,
        false,
        root);
    sonar->setSonarBeamCount(beam_count);
    sonar->enableReverb(env_cfg.enable_reverb);
    sonar->enableSpeckleNoise(env_cfg.enable_speckle);
    runtime_range_m = static_cast<float>(module_cfg.mbes_config.range_m);
    runtime_gain = static_cast<float>(module_cfg.mbes_config.gain);
    return true;
}

void MbesModule::setupWidget(DockWorkspace* workspace, const QString& title) {
    if (!sonar || !workspace) {
        if (rock_sonar_ui) {
            rock_sonar_ui->deleteLater();
            rock_sonar_ui = nullptr;
        }
        return;
    }
    rock_sonar_ui = new SonarControlPanel(workspace);
    workspace->addTab(rock_sonar_ui, title);
    rock_sonar_ui->setMinimumSize(320, 220);
    rock_sonar_ui->setMinRange(1);
    rock_sonar_ui->setMaxRange(150);
    rock_sonar_ui->setAdvancedPanelEnabled(true);
    rock_sonar_ui->setRange(static_cast<int>(std::lround(static_cast<double>(runtime_range_m))));
    const int gain_pct = static_cast<int>(std::lround(static_cast<double>(runtime_gain) * 100.0));
    rock_sonar_ui->setGain(std::clamp(gain_pct, 0, 100));
    rock_sonar_ui->setAdvancedSonarConfig(
        module_cfg.mbes_config.range_m,
        module_cfg.mbes_config.gain,
        module_cfg.mbes_config.center_frequency_khz,
        module_cfg.mbes_config.bandwidth_khz,
        module_cfg.mbes_config.beam_width_deg,
        module_cfg.mbes_config.beam_height_deg,
        module_cfg.mbes_config.angular_resolution_deg,
        module_cfg.mbes_config.tcp_output_enabled,
        module_cfg.mbes_config.file_output_enabled,
        module_cfg.mbes_config.tcp_host,
        module_cfg.mbes_config.tcp_port);
    rock_sonar_ui->setSonarPalette(1);
}

void MbesModule::connectWidgetSignals() {
    if (!rock_sonar_ui) {
        return;
    }
    QObject::connect(
        rock_sonar_ui,
        &SonarControlPanel::advancedSonarConfigChanged,
        [this](double range_m,
               double gain,
               double center_frequency_khz,
               double bandwidth_khz,
               double beam_width_deg,
               double beam_height_deg,
               double angle_resolution_deg,
               bool tcp_output_enabled,
               bool file_output_enabled,
               const QString& tcp_host,
               int tcp_port) {
            const double safe_range_m = std::clamp(range_m, 0.1, 500.0);
            const double safe_gain = std::clamp(gain, 0.0, 1.0);
            const double safe_center_frequency_khz = std::clamp(center_frequency_khz, 1.0, 2000.0);
            const double safe_bandwidth_khz =
                std::clamp(bandwidth_khz, 0.1, std::max(0.1, safe_center_frequency_khz - 0.1));
            const double safe_beam_width_deg =
                std::clamp(beam_width_deg, standalone_mvp::kMinSonarBeamDeg, standalone_mvp::kMaxSonarBeamDeg);
            const double safe_beam_height_deg =
                std::clamp(beam_height_deg, standalone_mvp::kMinSonarBeamDeg, standalone_mvp::kMaxSonarBeamDeg);
            const double safe_angle_resolution_deg = std::clamp(angle_resolution_deg, 0.01, 30.0);

            runtime_range_m = static_cast<float>(safe_range_m);
            runtime_gain = static_cast<float>(safe_gain);
            module_cfg.mbes_config.range_m = safe_range_m;
            module_cfg.mbes_config.gain = safe_gain;
            module_cfg.mbes_config.center_frequency_khz = safe_center_frequency_khz;
            module_cfg.mbes_config.bandwidth_khz = safe_bandwidth_khz;
            module_cfg.mbes_config.beam_width_deg = safe_beam_width_deg;
            module_cfg.mbes_config.beam_height_deg = safe_beam_height_deg;
            module_cfg.mbes_config.angular_resolution_deg = safe_angle_resolution_deg;
            module_cfg.mbes_config.bin_count = static_cast<int>(computeDerivedBinCount(module_cfg.mbes_config));
            module_cfg.mbes_config.beam_count = static_cast<int>(computeDerivedBeamCount(module_cfg.mbes_config));
            module_cfg.mbes_config.tcp_output_enabled = tcp_output_enabled;
            module_cfg.mbes_config.file_output_enabled = file_output_enabled;
            module_cfg.mbes_config.tcp_host = tcp_host.trimmed().isEmpty() ? QStringLiteral("0.0.0.0") : tcp_host.trimmed();
            module_cfg.mbes_config.tcp_port = std::clamp(tcp_port, 1, 65535);

            if (sonar) {
                sonar->setRange(runtime_range_m);
                sonar->setGain(runtime_gain);
                sonar->setSonarBinCount(static_cast<unsigned int>(module_cfg.mbes_config.bin_count));
                sonar->setSonarBeamCount(static_cast<unsigned int>(module_cfg.mbes_config.beam_count));
                sonar->setSonarBeamWidth(
                    sonar_types_v2::Angle::fromDeg(static_cast<float>(module_cfg.mbes_config.beam_width_deg)));
                sonar->setSonarBeamHeight(
                    sonar_types_v2::Angle::fromDeg(static_cast<float>(module_cfg.mbes_config.beam_height_deg)));
            }
        });
}

void MbesModule::initEsl2dRecording(const QString& project_dir) {
    esl2d_project_dir_ = project_dir;
}

void MbesModule::beginOutputSession(standalone_mvp::SonarTcpHub* hub,
                                    const standalone_mvp::ModuleOutputSession& session) {
    output_session_ = session;
    esl2d_file_writer_.resetFrameCount();
    bottom_tcp_streamer.resetFrameCount();
    esl2d_file_writer_.setTcpHub(hub);
    esl2d_file_writer_.setSessionActive(true);
    bottom_tcp_streamer.setTcpHub(hub);
    bottom_tcp_streamer.setSessionActive(true);

    const auto& c = module_cfg.mbes_config;
    esl2d_file_writer_.applyConfig(
        c.tcp_output_enabled,
        c.tcp_host.toStdString(),
        static_cast<std::uint16_t>(std::clamp(c.tcp_port, 1, 65535)),
        c.file_output_enabled,
        session.esl2d_path.toStdString());

    if (bottom_cloud_sim) {
        const auto& pc = module_cfg.point_cloud_config;
        bottom_cfg_runtime.file_output_path = session.esl3d_path.toStdString();
        bottom_tcp_streamer.applyConfig(
            pc.tcp_output_enabled,
            pc.tcp_host.toStdString(),
            static_cast<std::uint16_t>(std::clamp(pc.tcp_port, 1, 65535)),
            pc.file_output_enabled,
            session.esl3d_path.toStdString());
    }
}

void MbesModule::endOutputSession() {
    const bool run_post = module_cfg.point_cloud_config.file_output_enabled && !output_session_.esl3d_path.isEmpty();
    const QString esl3d_path = output_session_.esl3d_path;
    const QString waveform_dir = output_session_.waveform_dir;

    esl2d_file_writer_.setSessionActive(false);
    esl2d_file_writer_.close();
    bottom_tcp_streamer.setSessionActive(false);
    bottom_tcp_streamer.stop();
    output_session_ = {};

    if (run_post && !point_cloud_sonar_json_path_.isEmpty()) {
        standalone_mvp::runPointCloudPostProcess(esl3d_path, point_cloud_sonar_json_path_, waveform_dir);
    }
}

standalone_mvp::ModuleRecordingStats MbesModule::collectRecordingStats() const {
    standalone_mvp::ModuleRecordingStats stats;
    stats.module_name = module_cfg.name;
    stats.type = standalone_mvp::SonarModuleType::MBES;
    stats.config = module_cfg;
    stats.esl2d_frames = esl2d_file_writer_.framesWritten();
    stats.esl3d_frames = bottom_tcp_streamer.framesWritten();
    return stats;
}

bool MbesModule::tick(const Eigen::Affine3d& pose,
                      int frame_index,
                      int image_update_stride,
                      sonar_types_v2::samples::Sonar* out_sample) {
    if (!sonar) {
        return false;
    }
    const double depth_m = std::max(0.0, -pose.translation().z());
    sonar->setRange(runtime_range_m);
    sonar->setGain(runtime_gain);
    sonar->setAttenuationCoefficient(module_cfg.mbes_config.center_frequency_khz,
                                     env_cfg.temperature_c,
                                     depth_m,
                                     env_cfg.salinity_ppt,
                                     env_cfg.acidity_ph,
                                     env_cfg.enable_attenuation);
    sonar_types_v2::samples::Sonar sample = sonar->renderPing(pose);
    standalone_mvp::finalizeMultibeamSonarSample(sample, sonar->getSonarBeamWidth(), sonar->getSonarBeamCount());
    standalone_mvp::validateSonarSample(sample);
    if (rock_sonar_ui && (frame_index % std::max(1, image_update_stride)) == 0) {
        rock_sonar_ui->setData(sample);
    }
    if (esl2d_file_writer_.outputEnabled()) {
        standalone_mvp::Esl2dWriteParams params;
        params.sonar_kind = standalone_mvp::Esl2dSonarKind::FLS;
        params.max_range_m = runtime_range_m;
        params.pose = poseFromAffine(pose);
        params.sonar_config = mbesSonarConfigJson(module_cfg.mbes_config);
        params.environment = environmentConfigJson(env_cfg);
        params.sonar_module_name = module_cfg.name;
        esl2d_file_writer_.writeFlsFrame(sample, params);
    }
    if (out_sample) {
        *out_sample = sample;
    }
    return true;
}

bool MbesModule::initPointCloudRuntime(
    osg::ref_ptr<osg::Group> root,
    const Eigen::Affine3d& initial_pose,
    int x,
    int y,
    const QString& project_dir,
    DockWorkspace* workspace) {
    point_cloud_project_dir_ = project_dir;
    const QString sonar_json_rel = module_cfg.sonar_param_json_name.trimmed();
    point_cloud_sonar_json_path_ = sonar_json_rel.isEmpty()
                                       ? QString()
                                       : QDir(project_dir).filePath(QDir::fromNativeSeparators(sonar_json_rel));
    bottom_cfg_runtime.enabled = module_cfg.point_cloud_config.enabled && module_cfg.enabled && !module_cfg.camera_binding.trimmed().isEmpty();
    bottom_cfg_runtime.range_m = module_cfg.point_cloud_config.range_m;
    bottom_cfg_runtime.frequency_khz = module_cfg.point_cloud_config.frequency_khz;
    bottom_cfg_runtime.bandwidth_khz = module_cfg.point_cloud_config.bandwidth_khz;
    bottom_cfg_runtime.horizontal_angle_resolution_deg = module_cfg.point_cloud_config.horizontal_angle_resolution_deg;
    bottom_cfg_runtime.vertical_angle_resolution_deg = module_cfg.point_cloud_config.vertical_angle_resolution_deg;
    bottom_cfg_runtime.horizontal_fov_deg = module_cfg.point_cloud_config.horizontal_fov_deg;
    bottom_cfg_runtime.vertical_fov_deg = module_cfg.point_cloud_config.vertical_fov_deg;
    bottom_cfg_runtime.max_point_count = static_cast<std::size_t>(std::max(1, module_cfg.point_cloud_config.max_point_count));
    bottom_cfg_runtime.palette_index = module_cfg.point_cloud_config.palette_index;
    bottom_cfg_runtime.show_coordinate_overlay = module_cfg.point_cloud_config.show_coordinate_overlay;
    bottom_cfg_runtime.tcp_output_enabled = module_cfg.point_cloud_config.tcp_output_enabled;
    bottom_cfg_runtime.file_output_enabled = module_cfg.point_cloud_config.file_output_enabled;
    bottom_cfg_runtime.tcp_host = module_cfg.point_cloud_config.tcp_host.toStdString();
    bottom_cfg_runtime.tcp_port = static_cast<std::uint16_t>(std::clamp(module_cfg.point_cloud_config.tcp_port, 1, 65535));
    bottom_cfg_runtime.file_output_path.clear();
    bottom_cfg_runtime.enable_reverb = env_cfg.enable_reverb;
    bottom_cfg_runtime.enable_speckle = env_cfg.enable_speckle;
    bottom_cfg_runtime.enable_attenuation = env_cfg.enable_attenuation;
    bottom_cfg_runtime.temperature_c = env_cfg.temperature_c;
    bottom_cfg_runtime.salinity_ppt = env_cfg.salinity_ppt;
    bottom_cfg_runtime.acidity_ph = env_cfg.acidity_ph;
    bottom_cfg_runtime.attenuation_frequency_khz = env_cfg.attenuation_frequency_khz;
    bottom_cfg_runtime.sound_speed_mps = env_cfg.sound_speed_mps;
    if (!bottom_cfg_runtime.enabled || !sonar) {
        bottom_cloud_sim.reset();
        if (bottom_cloud_window) {
            bottom_cloud_window->close();
            bottom_cloud_window->deleteLater();
            bottom_cloud_window = nullptr;
        }
        return false;
    }
    bottom_cloud_sim = std::make_unique<standalone_mvp::PointCloudSonarSimulation>(bottom_cfg_runtime, root);
    bottom_cloud_window = new standalone_mvp::PointCloudViewerWindow();
    bottom_cloud_window->setWindowTitle(QString("%1 Point Cloud").arg(module_cfg.name));
    bottom_cloud_window->setInitialViewFromPose(initial_pose);
    bottom_cloud_window->setConfig(bottom_cfg_runtime);
    if (workspace) {
        workspace->addTab(bottom_cloud_window, QString("%1 Point Cloud").arg(module_cfg.name));
    } else {
        bottom_cloud_window->move(x, y);
        bottom_cloud_window->show();
    }
    bottom_tcp_streamer.applyConfig(false, bottom_cfg_runtime.tcp_host, bottom_cfg_runtime.tcp_port, false,
                                    bottom_cfg_runtime.file_output_path);
    const standalone_mvp::PointCloudTcpRuntimeStatus st = bottom_tcp_streamer.status();
    bottom_cloud_window->setTcpRuntimeStatus(st.running, st.client_connected, st.last_sent_seq, st.last_payload_bytes);
    return true;
}

void MbesModule::consumePointCloudUiConfig() {
    if (!bottom_cloud_window) {
        return;
    }
    standalone_mvp::PointCloudSonarConfig cfg_from_ui;
    if (!bottom_cloud_window->consumePendingConfig(cfg_from_ui)) {
        return;
    }
    bottom_cfg_runtime.range_m = cfg_from_ui.range_m;
    bottom_cfg_runtime.frequency_khz = cfg_from_ui.frequency_khz;
    bottom_cfg_runtime.bandwidth_khz = cfg_from_ui.bandwidth_khz;
    bottom_cfg_runtime.horizontal_angle_resolution_deg = cfg_from_ui.horizontal_angle_resolution_deg;
    bottom_cfg_runtime.vertical_angle_resolution_deg = cfg_from_ui.vertical_angle_resolution_deg;
    bottom_cfg_runtime.horizontal_fov_deg = cfg_from_ui.horizontal_fov_deg;
    bottom_cfg_runtime.vertical_fov_deg = cfg_from_ui.vertical_fov_deg;
    bottom_cfg_runtime.palette_index = cfg_from_ui.palette_index;
    bottom_cfg_runtime.show_coordinate_overlay = cfg_from_ui.show_coordinate_overlay;
    bottom_cfg_runtime.tcp_output_enabled = cfg_from_ui.tcp_output_enabled;
    bottom_cfg_runtime.file_output_enabled = cfg_from_ui.file_output_enabled;
    bottom_cfg_runtime.tcp_host = cfg_from_ui.tcp_host;
    bottom_cfg_runtime.tcp_port = cfg_from_ui.tcp_port;
    module_cfg.point_cloud_config.tcp_output_enabled = cfg_from_ui.tcp_output_enabled;
    module_cfg.point_cloud_config.file_output_enabled = cfg_from_ui.file_output_enabled;
    module_cfg.point_cloud_config.tcp_host = QString::fromStdString(cfg_from_ui.tcp_host);
    module_cfg.point_cloud_config.tcp_port = static_cast<int>(cfg_from_ui.tcp_port);
    bottom_cloud_sim->setConfig(bottom_cfg_runtime);
    if (output_session_.module_dir.isEmpty()) {
        bottom_tcp_streamer.applyConfig(false, bottom_cfg_runtime.tcp_host, bottom_cfg_runtime.tcp_port, false,
                                        bottom_cfg_runtime.file_output_path);
    } else {
        bottom_tcp_streamer.applyConfig(
            bottom_cfg_runtime.tcp_output_enabled,
            bottom_cfg_runtime.tcp_host,
            bottom_cfg_runtime.tcp_port,
            bottom_cfg_runtime.file_output_enabled,
            bottom_cfg_runtime.file_output_path);
    }
    const standalone_mvp::PointCloudTcpRuntimeStatus st = bottom_tcp_streamer.status();
    bottom_cloud_window->setTcpRuntimeStatus(st.running, st.client_connected, st.last_sent_seq, st.last_payload_bytes);
}

void MbesModule::setPointCloudRenderBlocked(bool blocked) {
    if (bottom_cloud_window) {
        bottom_cloud_window->setRenderBlockedByMainViewer(blocked);
    }
}

void MbesModule::tickPointCloud(const Eigen::Affine3d& pose, bool emit_single_frame_tcp) {
    if (!bottom_cloud_sim || !bottom_cfg_runtime.enabled) {
        return;
    }
    // Manual mode: keep point-cloud Res/FOV from point-cloud UI config.
    // Only range remains linked to MBES sonar runtime range.
    bottom_cfg_runtime.range_m = std::clamp(static_cast<double>(runtime_range_m), 0.1, 100.0);
    bottom_cfg_runtime.depth_m = std::max(0.0, -pose.translation().z());
    bottom_cfg_runtime.enable_reverb = env_cfg.enable_reverb;
    bottom_cfg_runtime.enable_speckle = env_cfg.enable_speckle;
    bottom_cfg_runtime.enable_attenuation = env_cfg.enable_attenuation;
    bottom_cfg_runtime.temperature_c = env_cfg.temperature_c;
    bottom_cfg_runtime.salinity_ppt = env_cfg.salinity_ppt;
    bottom_cfg_runtime.acidity_ph = env_cfg.acidity_ph;
    bottom_cfg_runtime.attenuation_frequency_khz = env_cfg.attenuation_frequency_khz;
    bottom_cfg_runtime.sound_speed_mps = env_cfg.sound_speed_mps;
    bottom_cloud_sim->setConfig(bottom_cfg_runtime);
    standalone_mvp::PointCloudFrame frame = bottom_cloud_sim->simulatePointCloud(pose);
    frame.coordinate_frame = "world (mbes_depth_composer_full_frame)";
    if (bottom_cloud_window) {
        bottom_cloud_window->setRangeMeters(bottom_cfg_runtime.range_m);
        bottom_cloud_window->updatePointCloudFrame(frame);
    }
    if (outputSessionActive()) {
        if (bottom_cfg_runtime.tcp_output_enabled || bottom_cfg_runtime.file_output_enabled) {
            bottom_tcp_streamer.sendFrame(frame);
        }
    } else if (emit_single_frame_tcp &&
               (bottom_cfg_runtime.tcp_output_enabled || bottom_cfg_runtime.file_output_enabled)) {
        bottom_tcp_streamer.sendFrame(frame);
    }
    if (bottom_cloud_window) {
        const standalone_mvp::PointCloudTcpRuntimeStatus st = bottom_tcp_streamer.status();
        bottom_cloud_window->setTcpRuntimeStatus(st.running, st.client_connected, st.last_sent_seq, st.last_payload_bytes);
    }
}
