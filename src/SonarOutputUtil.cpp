#include "SonarOutputUtil.hpp"
#include "offline_processing/OfflineProcessingPipeline.hpp"

#include <QApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProgressDialog>
#include <QRegularExpression>

#include <algorithm>
#include <chrono>
#include <future>
#include <iostream>
#include <mutex>

namespace standalone_mvp {
namespace {

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

QString buildMainCameraOutputDir(const QString& session_root) {
    const QString dir = QDir(session_root).filePath(QStringLiteral("Main Camera"));
    QDir().mkpath(dir);
    return dir;
}

QString buildModuleWaveformDir(const QString& module_dir) {
    const QString dir = QDir(module_dir).filePath(QStringLiteral("Waveform Data"));
    QDir().mkpath(dir);
    return dir;
}

std::vector<OutputFileRow> collectFileOutputRows(const std::vector<SonarModuleConfig>& modules,
                                                 const bool main_camera_file_output_enabled) {
    std::vector<OutputFileRow> rows;
    if (main_camera_file_output_enabled) {
        OutputFileRow row;
        row.module_name = QStringLiteral("Main Camera");
        row.file_format = QStringLiteral("MP4");
        row.sonar_type = QStringLiteral("Main Camera");
        rows.push_back(row);
    }
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
    if (!QDir().mkpath(waveform_output_dir)) {
        std::cerr << "[output] cannot create waveform output directory: "
                  << waveform_output_dir.toStdString() << std::endl;
        return false;
    }

    sonar::offline::ProcessingOptions options;
    options.esl3d_path = QDir::toNativeSeparators(esl3d_path).toStdString();
    options.sonar_config_path = QDir::toNativeSeparators(sonar_json_path).toStdString();
    options.output_directory = QDir::toNativeSeparators(waveform_output_dir).toStdString();

    QProgressDialog waiting(QStringLiteral("Preparing native sonar processing..."),
                            QString(), 0, 0);
    waiting.setWindowTitle(QStringLiteral("Sonar Processing"));
    waiting.setCancelButton(nullptr);
    waiting.setMinimumDuration(0);
    waiting.setWindowModality(Qt::ApplicationModal);
    waiting.show();

    std::mutex progress_mutex;
    sonar::offline::ProcessingProgress latest_progress;
    auto worker = std::async(std::launch::async, [&]() {
        return sonar::offline::process_esl3d_to_images(
            options, [&](const sonar::offline::ProcessingProgress& update) {
                std::lock_guard<std::mutex> lock(progress_mutex);
                latest_progress = update;
            });
    });

    using namespace std::chrono_literals;
    while (worker.wait_for(75ms) != std::future_status::ready) {
        sonar::offline::ProcessingProgress snapshot;
        {
            std::lock_guard<std::mutex> lock(progress_mutex);
            snapshot = latest_progress;
        }
        waiting.setLabelText(QString::fromStdString(snapshot.message));
        if (snapshot.total_steps > 0) {
            waiting.setRange(0, snapshot.total_steps);
            waiting.setValue(snapshot.completed_steps);
        } else {
            waiting.setRange(0, 0);
        }
        QApplication::processEvents();
    }

    try {
        const sonar::offline::ProcessingResult result = worker.get();
        waiting.close();
        std::cout << "[output] native offline processing wrote " << result.hdf5_path
                  << " and " << result.image_paths.size() << " image(s)" << std::endl;
        return QFile::exists(QString::fromStdString(result.hdf5_path)) &&
               result.image_paths.size() == static_cast<size_t>(result.frame_count);
    } catch (const std::exception& error) {
        waiting.close();
        std::cerr << "[output] native offline processing failed: " << error.what()
                  << std::endl;
        return false;
    }
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

bool anyModuleOutputEnabled(const std::vector<SonarModuleConfig>& modules,
                            const bool main_camera_file_output_enabled) {
    if (main_camera_file_output_enabled) {
        return true;
    }
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
    if (input.main_camera_file_output) {
        QJsonObject main_camera;
        main_camera["module_name"] = QStringLiteral("Main Camera");
        main_camera["file_format"] = QStringLiteral("MP4");
        main_camera["video_frames"] = static_cast<qint64>(input.main_camera_frames);
        root["main_camera"] = main_camera;
    }
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
