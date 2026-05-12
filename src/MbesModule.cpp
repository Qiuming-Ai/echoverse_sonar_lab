#include "MbesModule.hpp"

#include "RockSonarPlotView.hpp"
#include "PointCloudViewerWindow.hpp"
#include "SonarControlPanel.hpp"

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
#include <QTabWidget>

#include <sonar_types_v2/echoverse_math_types.hpp>
#include <sonar_core/AcousticRaySimulator.hpp>

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
    std::cout << "[mbes][cmd] " << kPointcloud2fileExe.toStdString() << " " << sonar_json_arg.toStdString() << std::endl;
    p2f_process.setWorkingDirectory(QDir::fromNativeSeparators(kMatlabRootPath));
    p2f_process.start(kPointcloud2fileExe, args);
    while (!p2f_process.waitForFinished(100)) {
        QApplication::processEvents();
    }
    const QString p2f_stdout = QString::fromLocal8Bit(p2f_process.readAllStandardOutput());
    const QString p2f_stderr = QString::fromLocal8Bit(p2f_process.readAllStandardError());
    std::cout << "[mbes][pointcloud2file] exit_code=" << p2f_process.exitCode()
              << " status=" << static_cast<int>(p2f_process.exitStatus()) << std::endl;
    if (!p2f_stdout.trimmed().isEmpty()) {
        std::cout << "[mbes][pointcloud2file][stdout]\n" << p2f_stdout.toStdString() << std::endl;
    }
    if (!p2f_stderr.trimmed().isEmpty()) {
        std::cerr << "[mbes][pointcloud2file][stderr]\n" << p2f_stderr.toStdString() << std::endl;
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
    std::cout << "[mbes][cmd] " << kFile2imageExe.toStdString() << " " << h5_arg.toStdString() << std::endl;
    f2i_process.setWorkingDirectory(QDir::fromNativeSeparators(kMatlabRootPath));
    f2i_process.start(kFile2imageExe, image_args);
    while (!f2i_process.waitForFinished(100)) {
        QApplication::processEvents();
    }
    const QString image_stdout = QString::fromLocal8Bit(f2i_process.readAllStandardOutput());
    const QString image_stderr = QString::fromLocal8Bit(f2i_process.readAllStandardError());
    std::cout << "[mbes][file2image] exit_code=" << f2i_process.exitCode()
              << " status=" << static_cast<int>(f2i_process.exitStatus()) << std::endl;
    if (!image_stdout.trimmed().isEmpty()) {
        std::cout << "[mbes][file2image][stdout]\n" << image_stdout.toStdString() << std::endl;
    }
    if (!image_stderr.trimmed().isEmpty()) {
        std::cerr << "[mbes][file2image][stderr]\n" << image_stderr.toStdString() << std::endl;
    }
    waiting_image.close();
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

void MbesModule::setupWidget(QTabWidget* tabs, const QString& title) {
    if (!sonar || !tabs) {
        if (rock_sonar_ui) {
            rock_sonar_ui->deleteLater();
            rock_sonar_ui = nullptr;
        }
        return;
    }
    rock_sonar_ui = new SonarControlPanel(tabs);
    tabs->addTab(rock_sonar_ui, title);
    rock_sonar_ui->setMinimumSize(640, 420);
    rock_sonar_ui->setMinRange(1);
    rock_sonar_ui->setMaxRange(150);
    rock_sonar_ui->setRange(static_cast<int>(std::lround(static_cast<double>(runtime_range_m))));
    const int gain_pct = static_cast<int>(std::lround(static_cast<double>(runtime_gain) * 100.0));
    rock_sonar_ui->setGain(std::clamp(gain_pct, 0, 100));
    rock_sonar_ui->setSonarPalette(1);
}

void MbesModule::connectWidgetSignals() {
    if (!rock_sonar_ui) {
        return;
    }
    QObject::connect(rock_sonar_ui, &SonarControlPanel::rangeChanged, [this](int meters) {
        runtime_range_m = std::clamp(static_cast<float>(meters), 1.0f, 500.0f);
        module_cfg.mbes_config.range_m = static_cast<double>(runtime_range_m);
        if (sonar) {
            sonar->setRange(runtime_range_m);
        }
    });
    QObject::connect(rock_sonar_ui, &SonarControlPanel::gainChanged, [this](int g_pct) {
        runtime_gain = std::clamp(static_cast<float>(g_pct) / 100.0f, 0.0f, 1.0f);
        module_cfg.mbes_config.gain = static_cast<double>(runtime_gain);
        if (sonar) {
            sonar->setGain(runtime_gain);
        }
    });
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
    QTabWidget* tabs) {
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
    bottom_cfg_runtime.file_output_path = buildPointCloudOutputPath(project_dir, module_cfg.name).toStdString();
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
    if (tabs) {
        tabs->addTab(bottom_cloud_window, QString("%1 Point Cloud").arg(module_cfg.name));
    } else {
        bottom_cloud_window->move(x, y);
        bottom_cloud_window->show();
    }
    bottom_tcp_streamer.applyConfig(
        bottom_cfg_runtime.tcp_output_enabled,
        bottom_cfg_runtime.tcp_host,
        bottom_cfg_runtime.tcp_port,
        bottom_cfg_runtime.file_output_enabled,
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
    const bool was_file_output_enabled = bottom_cfg_runtime.file_output_enabled;
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
    bottom_cloud_sim->setConfig(bottom_cfg_runtime);
    bottom_tcp_streamer.applyConfig(
        bottom_cfg_runtime.tcp_output_enabled,
        bottom_cfg_runtime.tcp_host,
        bottom_cfg_runtime.tcp_port,
        bottom_cfg_runtime.file_output_enabled,
        bottom_cfg_runtime.file_output_path);
    if (was_file_output_enabled && !bottom_cfg_runtime.file_output_enabled && !point_cloud_project_dir_.isEmpty()) {
        if (QFile::exists(point_cloud_sonar_json_path_)) {
            updateSonarJsonAndRunMatlab(
                QString::fromStdString(bottom_cfg_runtime.file_output_path), point_cloud_project_dir_, point_cloud_sonar_json_path_);
        }
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
    if (emit_single_frame_tcp && (bottom_cfg_runtime.tcp_output_enabled || bottom_cfg_runtime.file_output_enabled)) {
        bottom_tcp_streamer.sendFrame(frame);
    }
    if (bottom_cloud_window) {
        const standalone_mvp::PointCloudTcpRuntimeStatus st = bottom_tcp_streamer.status();
        bottom_cloud_window->setTcpRuntimeStatus(st.running, st.client_connected, st.last_sent_seq, st.last_payload_bytes);
    }
}
