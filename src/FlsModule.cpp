#include "FlsModule.hpp"

#include "PointCloudViewerWindow.hpp"
#include "RockSonarPlotView.hpp"
#include "SonarControlPanel.hpp"
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
#include <sonar_core/AcousticRaySimulator.hpp>

#include <sonar_types_v2/echoverse_math_types.hpp>

#include <osg/Group>
#include <osg/ref_ptr>

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

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
    const QString safe_name = QString(sonar_name).replace(QRegularExpression(R"([\\/:*?"<>|])"), "_");
    const QString date = QDateTime::currentDateTime().toString("yyyyMMdd_HHmm");
    const QString out_dir = QDir(project_dir).filePath("Point Cloud");
    QDir().mkpath(out_dir);
    return QDir(out_dir).filePath(QString("%1_%2.esl3d").arg(safe_name, date));
}

QString matlabRootPath() {
    const QString rel = QStringLiteral("src/matlab_point2file2image");
    const QString from_cwd = QDir::cleanPath(QDir::currentPath() + QStringLiteral("/") + rel);
    if (QDir(from_cwd).exists()) {
        return from_cwd;
    }
    return QDir::cleanPath(QCoreApplication::applicationDirPath() + QStringLiteral("/../") + rel);
}

void updateSonarJsonAndRunMatlab(const QString& esl3d_path, const QString& project_dir, const QString& sonar_json_path) {
    const QString kMatlabRootPath = matlabRootPath();
    const QString kPointcloud2fileExe = QDir(kMatlabRootPath).filePath(QStringLiteral("pointcloud2file.exe"));
    const QString kFile2imageExe = QDir(kMatlabRootPath).filePath(QStringLiteral("file2image.exe"));

    const QString sonar_data_dir = QDir(project_dir).filePath("Sonar Data");
    QDir().mkpath(sonar_data_dir);

    QJsonObject root;
    QFile in_file(sonar_json_path);
    if (in_file.exists() && in_file.open(QIODevice::ReadOnly)) {
        const QJsonDocument doc = QJsonDocument::fromJson(in_file.readAll());
        if (doc.isObject()) {
            root = doc.object();
        }
    }
    in_file.close();

    QJsonObject file_opt_params = root.value("file_opt_params").toObject();
    file_opt_params["esl3d_path"] = QDir::fromNativeSeparators(esl3d_path);
    file_opt_params["output_path"] = QDir::fromNativeSeparators(sonar_data_dir);
    root["file_opt_params"] = file_opt_params;

    QFile out_file(sonar_json_path);
    if (out_file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        const QJsonDocument doc(root);
        out_file.write(doc.toJson(QJsonDocument::Indented));
        out_file.close();
    }

    QProgressDialog waiting(QStringLiteral("Generating sonar data, please wait..."), QString(), 0, 0);
    waiting.setWindowTitle(QStringLiteral("Sonar Processing"));
    waiting.setCancelButton(nullptr);
    waiting.setMinimumDuration(0);
    waiting.setWindowModality(Qt::ApplicationModal);
    waiting.show();
    QApplication::processEvents();

    QProcess p2f_process;
    const QString sonar_json_arg = QDir::fromNativeSeparators(sonar_json_path);
    QStringList args;
    args << sonar_json_arg;
    std::cout << "[fls][cmd] " << kPointcloud2fileExe.toStdString() << " " << sonar_json_arg.toStdString() << std::endl;
    p2f_process.setWorkingDirectory(QDir::fromNativeSeparators(kMatlabRootPath));
    p2f_process.start(kPointcloud2fileExe, args);
    while (!p2f_process.waitForFinished(100)) {
        QApplication::processEvents();
    }
    const QString p2f_stdout = QString::fromLocal8Bit(p2f_process.readAllStandardOutput());
    const QString p2f_stderr = QString::fromLocal8Bit(p2f_process.readAllStandardError());
    std::cout << "[fls][pointcloud2file] exit_code=" << p2f_process.exitCode()
              << " status=" << static_cast<int>(p2f_process.exitStatus()) << std::endl;
    if (!p2f_stdout.trimmed().isEmpty()) {
        std::cout << "[fls][pointcloud2file][stdout]\n" << p2f_stdout.toStdString() << std::endl;
    }
    if (!p2f_stderr.trimmed().isEmpty()) {
        std::cerr << "[fls][pointcloud2file][stderr]\n" << p2f_stderr.toStdString() << std::endl;
    }
    waiting.close();

    const QString h5_path = QDir(sonar_data_dir).filePath(
        QStringLiteral("%1.h5").arg(QFileInfo(esl3d_path).completeBaseName()));
    QProgressDialog waiting_image(QStringLiteral("Generating sonar image, please wait..."), QString(), 0, 0);
    waiting_image.setWindowTitle(QStringLiteral("Sonar Processing"));
    waiting_image.setCancelButton(nullptr);
    waiting_image.setMinimumDuration(0);
    waiting_image.setWindowModality(Qt::ApplicationModal);
    waiting_image.show();
    QApplication::processEvents();

    QProcess f2i_process;
    const QString h5_arg = QDir::fromNativeSeparators(h5_path);
    QStringList image_args;
    image_args << h5_arg;
    std::cout << "[fls][cmd] " << kFile2imageExe.toStdString() << " " << h5_arg.toStdString() << std::endl;
    f2i_process.setWorkingDirectory(QDir::fromNativeSeparators(kMatlabRootPath));
    f2i_process.start(kFile2imageExe, image_args);
    while (!f2i_process.waitForFinished(100)) {
        QApplication::processEvents();
    }
    const QString image_stdout = QString::fromLocal8Bit(f2i_process.readAllStandardOutput());
    const QString image_stderr = QString::fromLocal8Bit(f2i_process.readAllStandardError());
    std::cout << "[fls][file2image] exit_code=" << f2i_process.exitCode()
              << " status=" << static_cast<int>(f2i_process.exitStatus()) << std::endl;
    if (!image_stdout.trimmed().isEmpty()) {
        std::cout << "[fls][file2image][stdout]\n" << image_stdout.toStdString() << std::endl;
    }
    if (!image_stderr.trimmed().isEmpty()) {
        std::cerr << "[fls][file2image][stderr]\n" << image_stderr.toStdString() << std::endl;
    }
    waiting_image.close();
}

} // namespace

FlsModule::FlsModule(const standalone_mvp::SonarModuleConfig& module_config)
    : runtime_range_m(static_cast<float>(module_config.fls_config.range_m)),
      runtime_gain(static_cast<float>(module_config.fls_config.gain)),
      pending_range_m(static_cast<float>(module_config.fls_config.range_m)),
      pending_gain(static_cast<float>(module_config.fls_config.gain)),
      module_cfg(module_config) {}

FlsModule::~FlsModule() = default;

void FlsModule::setModuleConfig(const standalone_mvp::SonarModuleConfig& module_config) {
    module_cfg = module_config;
    runtime_range_m = static_cast<float>(module_cfg.fls_config.range_m);
    runtime_gain = static_cast<float>(module_cfg.fls_config.gain);
    pending_range_m = runtime_range_m;
    pending_gain = runtime_gain;
}

void FlsModule::setEnvironmentConfig(const standalone_mvp::EnvironmentConfig& env_config) {
    env_cfg = env_config;
}

bool FlsModule::sonarEnabledByBinding() const {
    return module_cfg.enabled &&
           module_cfg.fls_config.enable_2d_fls &&
           !module_cfg.camera_binding.trimmed().isEmpty();
}

bool FlsModule::pointCloudEnabledByBinding() const {
    return module_cfg.enabled &&
           module_cfg.point_cloud_config.enabled &&
           !module_cfg.camera_binding.trimmed().isEmpty();
}

bool FlsModule::initSimulation(osg::ref_ptr<osg::Group> root, float resolution_constant) {
    if (!sonarEnabledByBinding()) {
        sonar.reset();
        return false;
    }
    const unsigned int bin_count = computeDerivedBinCount(module_cfg.fls_config);
    const unsigned int beam_count = computeDerivedBeamCount(module_cfg.fls_config);
    const unsigned int resolution = static_cast<unsigned int>(static_cast<float>(bin_count) * resolution_constant);
    sonar = std::make_unique<sonar_core::AcousticRaySimulator>(
        static_cast<float>(module_cfg.fls_config.range_m),
        static_cast<float>(module_cfg.fls_config.gain),
        bin_count,
        sonar_types_v2::Angle::fromDeg(static_cast<float>(module_cfg.fls_config.beam_width_deg)),
        sonar_types_v2::Angle::fromDeg(static_cast<float>(module_cfg.fls_config.beam_height_deg)),
        resolution,
        false,
        root);
    sonar->setSonarBeamCount(beam_count);
    sonar->enableReverb(env_cfg.enable_reverb);
    sonar->enableSpeckleNoise(env_cfg.enable_speckle);
    runtime_range_m = static_cast<float>(module_cfg.fls_config.range_m);
    runtime_gain = static_cast<float>(module_cfg.fls_config.gain);
    return true;
}

void FlsModule::setupWidget(DockWorkspace* workspace, const QString& title) {
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
        module_cfg.fls_config.range_m,
        module_cfg.fls_config.gain,
        module_cfg.fls_config.center_frequency_khz,
        module_cfg.fls_config.bandwidth_khz,
        module_cfg.fls_config.beam_width_deg,
        module_cfg.fls_config.beam_height_deg,
        module_cfg.fls_config.angular_resolution_deg);
    rock_sonar_ui->setSonarPalette(1);
}

void FlsModule::connectWidgetSignals() {
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
               double angle_resolution_deg) {
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
            module_cfg.fls_config.range_m = safe_range_m;
            module_cfg.fls_config.gain = safe_gain;
            module_cfg.fls_config.center_frequency_khz = safe_center_frequency_khz;
            module_cfg.fls_config.bandwidth_khz = safe_bandwidth_khz;
            module_cfg.fls_config.beam_width_deg = safe_beam_width_deg;
            module_cfg.fls_config.beam_height_deg = safe_beam_height_deg;
            module_cfg.fls_config.angular_resolution_deg = safe_angle_resolution_deg;
            module_cfg.fls_config.bin_count = static_cast<int>(computeDerivedBinCount(module_cfg.fls_config));
            module_cfg.fls_config.beam_count = static_cast<int>(computeDerivedBeamCount(module_cfg.fls_config));

            if (sonar) {
                sonar->setRange(runtime_range_m);
                sonar->setGain(runtime_gain);
                sonar->setSonarBinCount(static_cast<unsigned int>(module_cfg.fls_config.bin_count));
                sonar->setSonarBeamCount(static_cast<unsigned int>(module_cfg.fls_config.beam_count));
                sonar->setSonarBeamWidth(
                    sonar_types_v2::Angle::fromDeg(static_cast<float>(module_cfg.fls_config.beam_width_deg)));
                sonar->setSonarBeamHeight(
                    sonar_types_v2::Angle::fromDeg(static_cast<float>(module_cfg.fls_config.beam_height_deg)));
            }
            // Keep PointCloud range synced with FLS range changes.
            const double synced_pc_range = std::clamp(safe_range_m, 0.1, 100.0);
            module_cfg.point_cloud_config.range_m = synced_pc_range;
            point_cloud_cfg_runtime.range_m = synced_pc_range;
            if (point_cloud_sim) {
                point_cloud_sim->setConfig(point_cloud_cfg_runtime);
            }
            if (point_cloud_window) {
                point_cloud_window->setRangeMeters(synced_pc_range);
            }
        });
}

bool FlsModule::tick(const Eigen::Affine3d& pose,
                     int frame_index,
                     int image_update_stride,
                     sonar_types_v2::samples::Sonar* out_sample) {
    if (!sonar) {
        return false;
    }
    const double depth_m = std::max(0.0, -pose.translation().z());
    sonar->setRange(runtime_range_m);
    sonar->setGain(runtime_gain);
    sonar->setAttenuationCoefficient(module_cfg.fls_config.center_frequency_khz,
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
    if (out_sample) {
        *out_sample = sample;
    }
    return true;
}

bool FlsModule::initPointCloudRuntime(
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
    point_cloud_cfg_runtime.enabled = pointCloudEnabledByBinding();
    // Keep startup point-cloud range aligned with current FLS runtime range.
    point_cloud_cfg_runtime.range_m = std::clamp(static_cast<double>(runtime_range_m), 0.1, 100.0);
    point_cloud_cfg_runtime.frequency_khz = module_cfg.point_cloud_config.frequency_khz;
    point_cloud_cfg_runtime.bandwidth_khz = module_cfg.point_cloud_config.bandwidth_khz;
    point_cloud_cfg_runtime.horizontal_angle_resolution_deg = module_cfg.point_cloud_config.horizontal_angle_resolution_deg;
    point_cloud_cfg_runtime.vertical_angle_resolution_deg = module_cfg.point_cloud_config.vertical_angle_resolution_deg;
    point_cloud_cfg_runtime.horizontal_fov_deg = module_cfg.point_cloud_config.horizontal_fov_deg;
    point_cloud_cfg_runtime.vertical_fov_deg = module_cfg.point_cloud_config.vertical_fov_deg;
    point_cloud_cfg_runtime.max_point_count = static_cast<std::size_t>(std::max(1, module_cfg.point_cloud_config.max_point_count));
    point_cloud_cfg_runtime.palette_index = module_cfg.point_cloud_config.palette_index;
    point_cloud_cfg_runtime.show_coordinate_overlay = module_cfg.point_cloud_config.show_coordinate_overlay;
    point_cloud_cfg_runtime.tcp_output_enabled = module_cfg.point_cloud_config.tcp_output_enabled;
    point_cloud_cfg_runtime.file_output_enabled = module_cfg.point_cloud_config.file_output_enabled;
    point_cloud_cfg_runtime.tcp_host = module_cfg.point_cloud_config.tcp_host.toStdString();
    point_cloud_cfg_runtime.tcp_port = static_cast<std::uint16_t>(std::clamp(module_cfg.point_cloud_config.tcp_port, 1, 65535));
    point_cloud_cfg_runtime.file_output_path = buildPointCloudOutputPath(project_dir, module_cfg.name).toStdString();
    point_cloud_cfg_runtime.enable_reverb = env_cfg.enable_reverb;
    point_cloud_cfg_runtime.enable_speckle = env_cfg.enable_speckle;
    point_cloud_cfg_runtime.enable_attenuation = env_cfg.enable_attenuation;
    point_cloud_cfg_runtime.temperature_c = env_cfg.temperature_c;
    point_cloud_cfg_runtime.salinity_ppt = env_cfg.salinity_ppt;
    point_cloud_cfg_runtime.acidity_ph = env_cfg.acidity_ph;
    point_cloud_cfg_runtime.attenuation_frequency_khz = env_cfg.attenuation_frequency_khz;
    point_cloud_cfg_runtime.sound_speed_mps = env_cfg.sound_speed_mps;

    if (!point_cloud_cfg_runtime.enabled) {
        point_cloud_sim.reset();
        if (point_cloud_window) {
            point_cloud_window->close();
            point_cloud_window->deleteLater();
            point_cloud_window = nullptr;
        }
        point_cloud_runtime_enabled = false;
        return false;
    }
    point_cloud_sim = std::make_unique<standalone_mvp::PointCloudSonarSimulation>(point_cloud_cfg_runtime, root);
    point_cloud_window = new standalone_mvp::PointCloudViewerWindow();
    point_cloud_window->setWindowTitle(QString("%1 Point Cloud").arg(module_cfg.name));
    point_cloud_window->setInitialViewFromPose(initial_pose);
    point_cloud_window->setConfig(point_cloud_cfg_runtime);
    if (workspace) {
        workspace->addTab(point_cloud_window, QString("%1 Point Cloud").arg(module_cfg.name));
    } else {
        point_cloud_window->move(x, y);
        point_cloud_window->show();
    }
    point_cloud_tcp_streamer.applyConfig(
        point_cloud_cfg_runtime.tcp_output_enabled,
        point_cloud_cfg_runtime.tcp_host,
        point_cloud_cfg_runtime.tcp_port,
        point_cloud_cfg_runtime.file_output_enabled,
        point_cloud_cfg_runtime.file_output_path);
    const standalone_mvp::PointCloudTcpRuntimeStatus tcp_status = point_cloud_tcp_streamer.status();
    point_cloud_window->setTcpRuntimeStatus(
        tcp_status.running, tcp_status.client_connected, tcp_status.last_sent_seq, tcp_status.last_payload_bytes);
    point_cloud_runtime_enabled = true;
    return true;
}

void FlsModule::consumePointCloudUiConfig() {
    if (!point_cloud_runtime_enabled || !point_cloud_window) {
        return;
    }
    standalone_mvp::PointCloudSonarConfig cfg_from_ui;
    if (!point_cloud_window->consumePendingConfig(cfg_from_ui)) {
        return;
    }
    const bool was_file_output_enabled = point_cloud_cfg_runtime.file_output_enabled;
    point_cloud_cfg_runtime.enabled = true;
    const double synced_range = std::clamp(cfg_from_ui.range_m, 0.1, 100.0);
    point_cloud_cfg_runtime.range_m = synced_range;
    point_cloud_cfg_runtime.frequency_khz = cfg_from_ui.frequency_khz;
    point_cloud_cfg_runtime.bandwidth_khz = cfg_from_ui.bandwidth_khz;
    point_cloud_cfg_runtime.horizontal_angle_resolution_deg = cfg_from_ui.horizontal_angle_resolution_deg;
    point_cloud_cfg_runtime.vertical_angle_resolution_deg = cfg_from_ui.vertical_angle_resolution_deg;
    point_cloud_cfg_runtime.horizontal_fov_deg = cfg_from_ui.horizontal_fov_deg;
    point_cloud_cfg_runtime.vertical_fov_deg = cfg_from_ui.vertical_fov_deg;
    point_cloud_cfg_runtime.palette_index = cfg_from_ui.palette_index;
    point_cloud_cfg_runtime.show_coordinate_overlay = cfg_from_ui.show_coordinate_overlay;
    point_cloud_cfg_runtime.tcp_output_enabled = cfg_from_ui.tcp_output_enabled;
    point_cloud_cfg_runtime.file_output_enabled = cfg_from_ui.file_output_enabled;
    point_cloud_cfg_runtime.tcp_host = cfg_from_ui.tcp_host;
    point_cloud_cfg_runtime.tcp_port = cfg_from_ui.tcp_port;
    // PointCloud -> FLS range sync (bidirectional).
    runtime_range_m = static_cast<float>(synced_range);
    module_cfg.fls_config.range_m = synced_range;
    module_cfg.point_cloud_config.range_m = synced_range;
    if (sonar) {
        sonar->setRange(runtime_range_m);
    }
    if (rock_sonar_ui) {
        rock_sonar_ui->setRange(static_cast<int>(std::lround(synced_range)));
    }
    point_cloud_sim->setConfig(point_cloud_cfg_runtime);
    point_cloud_tcp_streamer.applyConfig(
        point_cloud_cfg_runtime.tcp_output_enabled,
        point_cloud_cfg_runtime.tcp_host,
        point_cloud_cfg_runtime.tcp_port,
        point_cloud_cfg_runtime.file_output_enabled,
        point_cloud_cfg_runtime.file_output_path);
    if (was_file_output_enabled && !point_cloud_cfg_runtime.file_output_enabled && !point_cloud_project_dir_.isEmpty()) {
        if (QFile::exists(point_cloud_sonar_json_path_)) {
            updateSonarJsonAndRunMatlab(
                QString::fromStdString(point_cloud_cfg_runtime.file_output_path), point_cloud_project_dir_, point_cloud_sonar_json_path_);
        }
    }
    const standalone_mvp::PointCloudTcpRuntimeStatus st = point_cloud_tcp_streamer.status();
    point_cloud_window->setTcpRuntimeStatus(st.running, st.client_connected, st.last_sent_seq, st.last_payload_bytes);
}

void FlsModule::setPointCloudRenderBlocked(bool blocked) {
    if (point_cloud_window) {
        point_cloud_window->setRenderBlockedByMainViewer(blocked);
    }
}

void FlsModule::tickPointCloud(const Eigen::Affine3d& pose) {
    if (!point_cloud_runtime_enabled || !point_cloud_sim || !point_cloud_cfg_runtime.enabled) {
        return;
    }
    point_cloud_cfg_runtime.range_m = std::clamp(static_cast<double>(runtime_range_m), 0.1, 100.0);
    point_cloud_cfg_runtime.depth_m = std::max(0.0, -pose.translation().z());
    point_cloud_cfg_runtime.temperature_c = env_cfg.temperature_c;
    point_cloud_cfg_runtime.salinity_ppt = env_cfg.salinity_ppt;
    point_cloud_cfg_runtime.acidity_ph = env_cfg.acidity_ph;
    point_cloud_cfg_runtime.enable_attenuation = env_cfg.enable_attenuation;
    point_cloud_cfg_runtime.enable_reverb = env_cfg.enable_reverb;
    point_cloud_cfg_runtime.enable_speckle = env_cfg.enable_speckle;
    point_cloud_cfg_runtime.attenuation_frequency_khz = env_cfg.attenuation_frequency_khz;
    point_cloud_cfg_runtime.sound_speed_mps = env_cfg.sound_speed_mps;
    point_cloud_sim->setConfig(point_cloud_cfg_runtime);
    const standalone_mvp::PointCloudFrame frame = point_cloud_sim->simulatePointCloud(pose);
    point_cloud_window->setRangeMeters(point_cloud_cfg_runtime.range_m);
    point_cloud_window->updatePointCloudFrame(frame);
    point_cloud_tcp_streamer.sendFrame(frame);
    const standalone_mvp::PointCloudTcpRuntimeStatus st = point_cloud_tcp_streamer.status();
    point_cloud_window->setTcpRuntimeStatus(st.running, st.client_connected, st.last_sent_seq, st.last_payload_bytes);
}
