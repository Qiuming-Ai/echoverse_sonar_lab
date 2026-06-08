#pragma once

#include "AppConfig.hpp"

#include <QString>
#include <cstdint>
#include <vector>

namespace standalone_mvp {

struct OutputFileRow {
    QString module_name;
    QString file_format;
    QString sonar_type;
};

struct OutputTcpRow {
    QString module_name;
    QString packet_format;
    QString sonar_type;
    int tcp_port = 0;
};

struct ModuleOutputSession {
    QString module_dir;
    QString esl2d_path;
    QString esl3d_path;
    QString waveform_dir;
};

struct ModuleRecordingStats {
    QString module_name;
    SonarModuleType type = SonarModuleType::FLS;
    std::uint64_t esl2d_frames = 0;
    std::uint64_t esl3d_frames = 0;
    SonarModuleConfig config;
};

struct SessionRecordingSummaryInput {
    QString session_root;
    double duration_seconds = 0.0;
    bool file_output_active = false;
    bool tcp_output_active = false;
    bool main_camera_file_output = false;
    std::uint64_t main_camera_frames = 0;
    std::vector<ModuleRecordingStats> modules;
};

QString safeModuleDirName(const QString& name);
QString sonarTypeLabel(SonarModuleType type);
QString buildOutputSessionRoot(const QString& project_dir);
QString buildModuleOutputDir(const QString& session_root, const QString& module_name);
QString buildMainCameraOutputDir(const QString& session_root);
QString buildModuleWaveformDir(const QString& module_dir);

std::vector<OutputFileRow> collectFileOutputRows(const std::vector<SonarModuleConfig>& modules,
                                                 bool main_camera_file_output_enabled = false);
std::vector<OutputTcpRow> collectTcpOutputRows(const std::vector<SonarModuleConfig>& modules);

void applyTcpPortEdit(std::vector<SonarModuleConfig>& modules,
                      const QString& module_name,
                      const QString& packet_format,
                      int tcp_port);

bool runPointCloudPostProcess(const QString& esl3d_path,
                              const QString& sonar_json_path,
                              const QString& waveform_output_dir);

bool anyModuleOutputEnabled(const std::vector<SonarModuleConfig>& modules,
                            bool main_camera_file_output_enabled = false);
bool writeSessionRecordingSummary(const SessionRecordingSummaryInput& input);

} // namespace standalone_mvp
