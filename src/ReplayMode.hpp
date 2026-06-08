#pragma once

#include "PointCloudSonarSimulation.hpp"

#include <QString>
#include <QStringList>

#include <algorithm>
#include <cmath>
#include <vector>

class QWidget;

namespace replay_mode {

enum class ReplaySonarType {
    FLS = 0,
    MBES = 1,
    SSS = 2,
};

enum class ReplayMediaKind {
    Video2d = 0,
    SssWaterfall = 1,
    PointCloud3d = 2,
};

struct ReplayModuleAssets {
    QString module_name;
    ReplaySonarType sonar_type = ReplaySonarType::FLS;
    ReplayMediaKind media_kind = ReplayMediaKind::Video2d;
    QString video_path;
    QString sss_png_path;
    QString intensity_mp4_path;
    QString range_mp4_path;
    QString esl3d_source_path;
    int frame_count = 0;
    int sss_row_count = 0;
    double output_fps = 20.0;
    std::vector<qint64> timeline_us;
    std::vector<qint64> esl3d_frame_offsets;
};

struct ReplayConversionResult {
    QString session_folder;
    QString output_dir;
    double session_duration_seconds = 0.0;
    std::vector<ReplayModuleAssets> modules;
    std::vector<qint64> timeline_us;
    bool used_cache = false;
};

QString replayTabTitle(const ReplayModuleAssets& module);

QStringList listSonarDataSessions(const QString& project_dir);

QString selectReplaySessionFolder(QWidget* parent, const QString& project_dir);

bool loadOrConvert(const QString& folder, ReplayConversionResult& out_result, QString& err, QWidget* progress_parent = nullptr);

bool readEsl3dPointCloudFrame(const QString& esl3d_path,
                              const std::vector<qint64>& frame_offsets,
                              int frame_index,
                              standalone_mvp::PointCloudFrame& out_frame);

int indexEsl3dFrameOffsets(const QString& esl3d_path, std::vector<qint64>& out_offsets);

int computeReplayMaxFrame(const ReplayConversionResult& result);

int computeReplayIntervalMs(const ReplayConversionResult& result, int replay_max_frame);

inline int mapTimestampToFrame(const std::vector<qint64>& source_timestamps, qint64 target_us) {
    if (source_timestamps.empty()) {
        return 0;
    }
    auto it = std::upper_bound(source_timestamps.begin(), source_timestamps.end(), target_us);
    if (it == source_timestamps.begin()) {
        return 0;
    }
    return static_cast<int>(std::distance(source_timestamps.begin(), it - 1));
}

inline int mapReplaySourceFrame(int replay_frame_index, int replay_max_frame, int source_frame_count) {
    if (source_frame_count <= 0) {
        return 0;
    }
    if (source_frame_count == 1 || replay_max_frame <= 0) {
        return 0;
    }
    const double ratio =
        static_cast<double>(std::clamp(replay_frame_index, 0, replay_max_frame)) /
        static_cast<double>(replay_max_frame);
    return std::clamp(
        static_cast<int>(std::lround(ratio * static_cast<double>(source_frame_count - 1))),
        0,
        source_frame_count - 1);
}

} // namespace replay_mode
