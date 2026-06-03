#include "SonarOutputUtil.hpp"

#include <QApplication>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProcess>
#include <QProgressDialog>
#include <QRegularExpression>

#include <algorithm>
#include <iostream>

namespace standalone_mvp {
namespace {

QString matlabRootPath() {
    const QString rel = QStringLiteral("src/matlab_point2file2image");
    const QString from_cwd = QDir::cleanPath(QDir::currentPath() + QStringLiteral("/") + rel);
    if (QDir(from_cwd).exists()) {
        return from_cwd;
    }
    return QDir::cleanPath(QCoreApplication::applicationDirPath() + QStringLiteral("/../") + rel);
}

QString quoteCommandPath(const QString& path) {
    QString native = QDir::toNativeSeparators(path);
    native.replace(QLatin1Char('"'), QStringLiteral("\\\""));
    return QStringLiteral("\"%1\"").arg(native);
}

QString formatQuotedExeCommand(const QString& exe, const QStringList& args) {
    QString cmd = QDir::toNativeSeparators(exe);
    for (const QString& arg : args) {
        cmd += QLatin1Char(' ');
        cmd += quoteCommandPath(arg);
    }
    return cmd;
}

bool esl2dFileOutputEnabled(const SonarModuleConfig& mod) {
    switch (mod.type) {
    case SonarModuleType::FLS:
        return mod.fls_config.file_output_enabled;
    case SonarModuleType::MBES:
        return mod.mbes_config.file_output_enabled;
    case SonarModuleType::SSS:
        return mod.sss_config.file_output_enabled;
    }
    return false;
}

bool esl2dTcpOutputEnabled(const SonarModuleConfig& mod) {
    switch (mod.type) {
    case SonarModuleType::FLS:
        return mod.fls_config.tcp_output_enabled;
    case SonarModuleType::MBES:
        return mod.mbes_config.tcp_output_enabled;
    case SonarModuleType::SSS:
        return mod.sss_config.tcp_output_enabled;
    }
    return false;
}

QString esl2dFormatLabel(SonarModuleType type) {
    switch (type) {
    case SonarModuleType::FLS:
        return QStringLiteral("ESL2D (FLS)");
    case SonarModuleType::MBES:
        return QStringLiteral("ESL2D (MBES)");
    case SonarModuleType::SSS:
        return QStringLiteral("ESL2D (SSS)");
    }
    return QStringLiteral("ESL2D");
}

int moduleTcpPort(const SonarModuleConfig& mod) {
    switch (mod.type) {
    case SonarModuleType::FLS:
        return mod.fls_config.tcp_port;
    case SonarModuleType::MBES:
        return mod.mbes_config.tcp_port;
    case SonarModuleType::SSS:
        return mod.sss_config.tcp_port;
    }
    return 30001;
}

QString moduleTcpHost(const SonarModuleConfig& mod) {
    switch (mod.type) {
    case SonarModuleType::FLS:
        return mod.fls_config.tcp_host;
    case SonarModuleType::MBES:
        return mod.mbes_config.tcp_host;
    case SonarModuleType::SSS:
        return mod.sss_config.tcp_host;
    }
    return QStringLiteral("0.0.0.0");
}

void setModuleTcpPort(SonarModuleConfig& mod, int port) {
    const int clamped = std::clamp(port, 1, 65535);
    switch (mod.type) {
    case SonarModuleType::FLS:
        mod.fls_config.tcp_port = clamped;
        mod.point_cloud_config.tcp_port = clamped;
        break;
    case SonarModuleType::MBES:
        mod.mbes_config.tcp_port = clamped;
        mod.point_cloud_config.tcp_port = clamped;
        break;
    case SonarModuleType::SSS:
        mod.sss_config.tcp_port = clamped;
        break;
    }
}

} // namespace

QString safeModuleDirName(const QString& name) {
    return QString(name).replace(QRegularExpression(R"([\\/:*?"<>|])"), "_");
}

QString sonarTypeLabel(const SonarModuleType type) {
    switch (type) {
    case SonarModuleType::FLS:
        return QStringLiteral("FLS");
    case SonarModuleType::MBES:
        return QStringLiteral("MBES");
    case SonarModuleType::SSS:
        return QStringLiteral("SSS");
    }
    return QStringLiteral("Unknown");
}

QString buildOutputSessionRoot(const QString& project_dir) {
    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    const QString root = QDir(project_dir).filePath(QStringLiteral("Sonar Data/%1").arg(timestamp));
    QDir().mkpath(root);
    return root;
}

QString buildModuleOutputDir(const QString& session_root, const QString& module_name) {
    const QString dir = QDir(session_root).filePath(safeModuleDirName(module_name));
    QDir().mkpath(dir);
    return dir;
}

QString buildModuleWaveformDir(const QString& module_dir) {
    const QString dir = QDir(module_dir).filePath(QStringLiteral("Waveform Data"));
    QDir().mkpath(dir);
    return dir;
}

std::vector<OutputFileRow> collectFileOutputRows(const std::vector<SonarModuleConfig>& modules) {
    std::vector<OutputFileRow> rows;
    for (const auto& mod : modules) {
        if (!mod.enabled) {
            continue;
        }
        if (esl2dFileOutputEnabled(mod)) {
            OutputFileRow row;
            row.module_name = mod.name;
            row.file_format = esl2dFormatLabel(mod.type);
            row.sonar_type = sonarTypeLabel(mod.type);
            rows.push_back(row);
        }
        if ((mod.type == SonarModuleType::FLS || mod.type == SonarModuleType::MBES) &&
            mod.point_cloud_config.enabled && mod.point_cloud_config.file_output_enabled) {
            OutputFileRow row;
            row.module_name = mod.name;
            row.file_format = QStringLiteral("ESL3D (Point Cloud)");
            row.sonar_type = sonarTypeLabel(mod.type);
            rows.push_back(row);
        }
    }
    return rows;
}

std::vector<OutputTcpRow> collectTcpOutputRows(const std::vector<SonarModuleConfig>& modules) {
    std::vector<OutputTcpRow> rows;
    for (const auto& mod : modules) {
        if (!mod.enabled) {
            continue;
        }
        if (esl2dTcpOutputEnabled(mod)) {
            OutputTcpRow row;
            row.module_name = mod.name;
            row.packet_format = esl2dFormatLabel(mod.type);
            row.sonar_type = sonarTypeLabel(mod.type);
            row.tcp_port = moduleTcpPort(mod);
            rows.push_back(row);
        }
        if ((mod.type == SonarModuleType::FLS || mod.type == SonarModuleType::MBES) &&
            mod.point_cloud_config.enabled && mod.point_cloud_config.tcp_output_enabled) {
            OutputTcpRow row;
            row.module_name = mod.name;
            row.packet_format = QStringLiteral("ESL3D (Point Cloud)");
            row.sonar_type = sonarTypeLabel(mod.type);
            row.tcp_port = mod.point_cloud_config.tcp_port;
            rows.push_back(row);
        }
    }
    return rows;
}

void applyTcpPortEdit(std::vector<SonarModuleConfig>& modules,
                      const QString& module_name,
                      const QString& packet_format,
                      const int tcp_port) {
    for (auto& mod : modules) {
        if (mod.name != module_name) {
            continue;
        }
        if (packet_format.startsWith(QStringLiteral("ESL3D"))) {
            mod.point_cloud_config.tcp_port = std::clamp(tcp_port, 1, 65535);
        } else {
            setModuleTcpPort(mod, tcp_port);
        }
        break;
    }
}

bool runPointCloudPostProcess(const QString& esl3d_path,
                              const QString& sonar_json_path,
                              const QString& waveform_output_dir) {
    if (esl3d_path.isEmpty() || sonar_json_path.isEmpty() || !QFile::exists(esl3d_path) ||
        !QFile::exists(sonar_json_path)) {
        return false;
    }
    const QString kMatlabRootPath = matlabRootPath();
    const QString kPointcloud2fileExe = QDir(kMatlabRootPath).filePath(QStringLiteral("pointcloud2file.exe"));
    const QString kFile2imageExe = QDir(kMatlabRootPath).filePath(QStringLiteral("file2image.exe"));

    QDir().mkpath(waveform_output_dir);

    QJsonObject root;
    QFile in_file(sonar_json_path);
    if (in_file.open(QIODevice::ReadOnly)) {
        const QJsonDocument doc = QJsonDocument::fromJson(in_file.readAll());
        if (doc.isObject()) {
            root = doc.object();
        }
    }
    in_file.close();

    QJsonObject file_opt_params = root.value(QStringLiteral("file_opt_params")).toObject();
    file_opt_params[QStringLiteral("esl3d_path")] = QDir::fromNativeSeparators(esl3d_path);
    file_opt_params[QStringLiteral("output_path")] = QDir::fromNativeSeparators(waveform_output_dir);
    root[QStringLiteral("file_opt_params")] = file_opt_params;

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
    const QString p2f_cmd = formatQuotedExeCommand(kPointcloud2fileExe, {sonar_json_arg});
    std::cout << "[output][cmd] " << p2f_cmd.toStdString() << std::endl;
    p2f_process.setWorkingDirectory(QDir::fromNativeSeparators(kMatlabRootPath));
    p2f_process.startCommand(p2f_cmd);
    while (!p2f_process.waitForFinished(100)) {
        QApplication::processEvents();
    }
    waiting.close();

    const QString h5_path = QDir(waveform_output_dir).filePath(
        QStringLiteral("%1.h5").arg(QFileInfo(esl3d_path).completeBaseName()));
    if (!QFile::exists(h5_path)) {
        std::cerr << "[output] pointcloud2file did not produce h5 at " << h5_path.toStdString() << std::endl;
        return false;
    }

    QProgressDialog waiting_image(QStringLiteral("Generating sonar image, please wait..."), QString(), 0, 0);
    waiting_image.setWindowTitle(QStringLiteral("Sonar Processing"));
    waiting_image.setCancelButton(nullptr);
    waiting_image.setMinimumDuration(0);
    waiting_image.setWindowModality(Qt::ApplicationModal);
    waiting_image.show();
    QApplication::processEvents();

    QProcess f2i_process;
    const QString h5_arg = QDir::fromNativeSeparators(h5_path);
    const QString f2i_cmd = formatQuotedExeCommand(kFile2imageExe, {h5_arg});
    std::cout << "[output][cmd] " << f2i_cmd.toStdString() << std::endl;
    f2i_process.setWorkingDirectory(QDir::fromNativeSeparators(kMatlabRootPath));
    f2i_process.startCommand(f2i_cmd);
    while (!f2i_process.waitForFinished(100)) {
        QApplication::processEvents();
    }
    waiting_image.close();
    return f2i_process.exitCode() == 0;
}

bool moduleWantsOutput(const SonarModuleConfig& mod) {
    if (!mod.enabled) {
        return false;
    }
    switch (mod.type) {
    case SonarModuleType::FLS:
        if (mod.fls_config.file_output_enabled || mod.fls_config.tcp_output_enabled) {
            return true;
        }
        return mod.point_cloud_config.enabled &&
               (mod.point_cloud_config.file_output_enabled || mod.point_cloud_config.tcp_output_enabled);
    case SonarModuleType::MBES:
        if (mod.mbes_config.file_output_enabled || mod.mbes_config.tcp_output_enabled) {
            return true;
        }
        return mod.point_cloud_config.enabled &&
               (mod.point_cloud_config.file_output_enabled || mod.point_cloud_config.tcp_output_enabled);
    case SonarModuleType::SSS:
        return mod.sss_config.file_output_enabled || mod.sss_config.tcp_output_enabled;
    }
    return false;
}

bool anyModuleOutputEnabled(const std::vector<SonarModuleConfig>& modules) {
    for (const auto& mod : modules) {
        if (moduleWantsOutput(mod)) {
            return true;
        }
    }
    return false;
}

QJsonObject sonarConfigToJson(const SonarModuleConfig& mod) {
    QJsonObject root;
    root["name"] = mod.name;
    root["type"] = sonarTypeLabel(mod.type);
    root["camera_binding"] = mod.camera_binding;
    root["enabled"] = mod.enabled;
    root["sonar_param_json_name"] = mod.sonar_param_json_name;

    auto writeSonarUi = [](const SonarConfigUi& cfg) {
        QJsonObject o;
        o["range_m"] = cfg.range_m;
        o["gain"] = cfg.gain;
        o["center_frequency_khz"] = cfg.center_frequency_khz;
        o["bandwidth_khz"] = cfg.bandwidth_khz;
        o["beam_width_deg"] = cfg.beam_width_deg;
        o["beam_height_deg"] = cfg.beam_height_deg;
        o["angular_resolution_deg"] = cfg.angular_resolution_deg;
        o["tcp_output_enabled"] = cfg.tcp_output_enabled;
        o["file_output_enabled"] = cfg.file_output_enabled;
        o["tcp_host"] = cfg.tcp_host;
        o["tcp_port"] = cfg.tcp_port;
        return o;
    };
    auto writePointCloudUi = [](const PointCloudSonarConfigUi& cfg) {
        QJsonObject o;
        o["enabled"] = cfg.enabled;
        o["range_m"] = cfg.range_m;
        o["frequency_khz"] = cfg.frequency_khz;
        o["bandwidth_khz"] = cfg.bandwidth_khz;
        o["horizontal_angle_resolution_deg"] = cfg.horizontal_angle_resolution_deg;
        o["vertical_angle_resolution_deg"] = cfg.vertical_angle_resolution_deg;
        o["horizontal_fov_deg"] = cfg.horizontal_fov_deg;
        o["vertical_fov_deg"] = cfg.vertical_fov_deg;
        o["max_point_count"] = cfg.max_point_count;
        o["tcp_output_enabled"] = cfg.tcp_output_enabled;
        o["file_output_enabled"] = cfg.file_output_enabled;
        o["tcp_host"] = cfg.tcp_host;
        o["tcp_port"] = cfg.tcp_port;
        return o;
    };
    auto writeSssUi = [](const SideScanSonarConfigUi& cfg) {
        QJsonObject o;
        o["range_m"] = cfg.range_m;
        o["gain"] = cfg.gain;
        o["center_frequency_khz"] = cfg.center_frequency_khz;
        o["bandwidth_khz"] = cfg.bandwidth_khz;
        o["beam_width_deg"] = cfg.beam_width_deg;
        o["beam_height_deg"] = cfg.beam_height_deg;
        o["angular_resolution_deg"] = cfg.angular_resolution_deg;
        o["update_stride"] = cfg.update_stride;
        o["tcp_output_enabled"] = cfg.tcp_output_enabled;
        o["file_output_enabled"] = cfg.file_output_enabled;
        o["tcp_host"] = cfg.tcp_host;
        o["tcp_port"] = cfg.tcp_port;
        return o;
    };

    switch (mod.type) {
    case SonarModuleType::FLS:
        root["fls_config"] = writeSonarUi(mod.fls_config);
        root["point_cloud_config"] = writePointCloudUi(mod.point_cloud_config);
        break;
    case SonarModuleType::MBES:
        root["mbes_config"] = writeSonarUi(mod.mbes_config);
        root["point_cloud_config"] = writePointCloudUi(mod.point_cloud_config);
        break;
    case SonarModuleType::SSS:
        root["sss_config"] = writeSssUi(mod.sss_config);
        root["sss_camera_slot1"] = mod.sss_camera_slot1;
        root["sss_camera_slot2"] = mod.sss_camera_slot2;
        break;
    }
    return root;
}

bool writeSessionRecordingSummary(const SessionRecordingSummaryInput& input) {
    if (input.session_root.isEmpty()) {
        return false;
    }
    QJsonObject root;
    root["session_root"] = QDir::fromNativeSeparators(input.session_root);
    root["duration_seconds"] = input.duration_seconds;
    root["file_output_active"] = input.file_output_active;
    root["tcp_output_active"] = input.tcp_output_active;
    root["recorded_at"] = QDateTime::currentDateTime().toString(Qt::ISODate);

    QJsonArray modules;
    for (const auto& mod_stats : input.modules) {
        QJsonObject entry;
        entry["module_name"] = mod_stats.module_name;
        entry["sonar_type"] = sonarTypeLabel(mod_stats.type);
        entry["esl2d_frames"] = static_cast<qint64>(mod_stats.esl2d_frames);
        entry["esl3d_frames"] = static_cast<qint64>(mod_stats.esl3d_frames);
        entry["sonar_config"] = sonarConfigToJson(mod_stats.config);
        modules.append(entry);
    }
    root["modules"] = modules;

    const QString summary_path = QDir(input.session_root).filePath(QStringLiteral("recording_summary.json"));
    QFile out_file(summary_path);
    if (!out_file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        std::cerr << "[output] failed to write recording summary path=" << summary_path.toStdString() << std::endl;
        return false;
    }
    out_file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    out_file.close();
    return true;
}

} // namespace standalone_mvp
