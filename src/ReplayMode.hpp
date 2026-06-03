#pragma once

#include "PointCloudSonarSimulation.hpp"

#include <QString>
#include <QStringList>

#include <algorithm>
#include <cmath>

#include <vector>

class QWidget;

namespace replay_mode {

struct ReplayConversionResult {
    QString session_folder;
    QString output_dir;
    QString fls_mp4_path;
    QString intensity_mp4_path;
    QString range_mp4_path;
    QString sss_png_path;
    QString esl3d_source_path;
    int sss_row_count = 0;
    int esl3d_frame_count = 0;
    std::vector<qint64> esl3d_frame_offsets;
    std::vector<qint64> timeline_us;
    bool has_fls = false;
    bool has_sss = false;
    bool has_3d = false;
    bool used_cache = false;
};

QStringList listSonarDataSessions(const QString& project_dir);

QString selectReplaySessionFolder(QWidget* parent, const QString& project_dir);

bool loadOrConvert(const QString& folder, ReplayConversionResult& out_result, QString& err);

bool readEsl3dPointCloudFrame(const QString& esl3d_path,
                              const std::vector<qint64>& frame_offsets,
                              int frame_index,
                              standalone_mvp::PointCloudFrame& out_frame);

int indexEsl3dFrameOffsets(const QString& esl3d_path, std::vector<qint64>& out_offsets);

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
