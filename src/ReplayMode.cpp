#include "ReplayMode.hpp"
#include "RockSonarPlotView.hpp"
#include "SonarOutputUtil.hpp"

#include <QDialog>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QApplication>
#include <QHash>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QProgressDialog>
#include <QWidget>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <vector>

namespace replay_mode {
namespace {

constexpr std::uint32_t kEsl2dMagic = 0x5032534Eu;
constexpr std::uint32_t kEsl3dMagic = 0x5033534Eu;
constexpr int kFlsPlotWidth = 960;
constexpr int kFlsPlotHeight = 540;

template <typename T>
bool readLE(std::ifstream& in, T& out) {
    in.read(reinterpret_cast<char*>(&out), sizeof(T));
    return in.good();
}

bool cacheFileReady(const QString& path) {
    return QFileInfo::exists(path) && QFileInfo(path).size() > 0;
}

QString replayCacheDirForSonarFile(const QString& sonar_file_path) {
    return QDir(QFileInfo(sonar_file_path).absolutePath()).filePath(QStringLiteral("replay_cache"));
}

QString cacheBaseName(const QString& sonar_file_path) {
    return QFileInfo(sonar_file_path).completeBaseName();
}

QString flsCachePath(const QString& cache_dir, const QString& esl2d_path) {
    return QDir(cache_dir).filePath(cacheBaseName(esl2d_path) + QStringLiteral(".mp4"));
}

QString sssCachePath(const QString& cache_dir, const QString& esl2d_path) {
    return QDir(cache_dir).filePath(cacheBaseName(esl2d_path) + QStringLiteral("_waterfall.png"));
}

QString rangeCachePath(const QString& cache_dir, const QString& esl3d_path) {
    return QDir(cache_dir).filePath(cacheBaseName(esl3d_path) + QStringLiteral("_range.mp4"));
}

QString intensityCachePath(const QString& cache_dir, const QString& esl3d_path) {
    return QDir(cache_dir).filePath(cacheBaseName(esl3d_path) + QStringLiteral("_intensity.mp4"));
}

int videoFrameCount(const QString& mp4_path) {
    cv::VideoCapture cap(mp4_path.toStdString());
    if (!cap.isOpened()) {
        return 0;
    }
    return std::max(0, static_cast<int>(cap.get(cv::CAP_PROP_FRAME_COUNT)));
}

cv::Mat normalizeFloatImage(const std::vector<float>& src, int h, int w) {
    if (src.empty() || h <= 0 || w <= 0 || static_cast<int>(src.size()) < h * w) {
        return {};
    }
    float min_v = std::numeric_limits<float>::max();
    float max_v = std::numeric_limits<float>::lowest();
    for (int i = 0; i < h * w; ++i) {
        min_v = std::min(min_v, src[i]);
        max_v = std::max(max_v, src[i]);
    }
    cv::Mat out(h, w, CV_8UC1);
    const float denom = std::max(1e-6f, max_v - min_v);
    for (int r = 0; r < h; ++r) {
        auto* row = out.ptr<uchar>(r);
        for (int c = 0; c < w; ++c) {
            const float v = src[r * w + c];
            row[c] = static_cast<uchar>(std::clamp((v - min_v) / denom, 0.0f, 1.0f) * 255.0f);
        }
    }
    return out;
}

QJsonObject parseMetadataObject(const std::vector<char>& payload, std::uint32_t metadata_bytes) {
    if (metadata_bytes == 0 || metadata_bytes > payload.size()) {
        return {};
    }
    const QJsonDocument doc =
        QJsonDocument::fromJson(QByteArray(payload.data(), static_cast<int>(metadata_bytes)));
    return doc.isObject() ? doc.object() : QJsonObject{};
}

bool buildFlsSonarFromEsl2d(const std::vector<float>& bins,
                            const std::vector<char>& payload,
                            std::uint32_t metadata_bytes,
                            std::uint32_t beam_angles_bytes,
                            std::uint32_t beam_count,
                            std::uint32_t bin_count,
                            sonar_types_v2::samples::Sonar& out_sonar) {
    if (beam_count == 0 || bin_count == 0 ||
        bins.size() < static_cast<std::size_t>(beam_count) * static_cast<std::size_t>(bin_count)) {
        return false;
    }

    const QJsonObject metadata = parseMetadataObject(payload, metadata_bytes);
    const QJsonObject sonar_cfg = metadata.value(QStringLiteral("sonar_config")).toObject();

    out_sonar = sonar_types_v2::samples::Sonar();
    out_sonar.beam_count = beam_count;
    out_sonar.bin_count = bin_count;
    out_sonar.bins = bins;

    const std::size_t angles_off = metadata_bytes;
    if (beam_angles_bytes >= 4u * beam_count &&
        angles_off + beam_angles_bytes <= payload.size()) {
        const float* angles = reinterpret_cast<const float*>(payload.data() + angles_off);
        out_sonar.bearings.reserve(beam_count);
        for (std::uint32_t i = 0; i < beam_count; ++i) {
            out_sonar.bearings.push_back(sonar_types_v2::Angle::fromDeg(angles[i]));
        }
    }

    double beam_width_deg = sonar_cfg.value(QStringLiteral("beam_width_deg")).toDouble(130.0);
    if (beam_width_deg <= 0.0 && out_sonar.bearings.size() >= 2) {
        beam_width_deg =
            out_sonar.bearings.back().getDeg() - out_sonar.bearings.front().getDeg();
    }
    if (beam_width_deg <= 0.0) {
        beam_width_deg = 130.0;
    }

    out_sonar.beam_width = sonar_types_v2::Angle::fromDeg(static_cast<float>(beam_width_deg));
    const double beam_height_deg = sonar_cfg.value(QStringLiteral("beam_height_deg")).toDouble(20.0);
    out_sonar.beam_height = sonar_types_v2::Angle::fromDeg(static_cast<float>(std::max(1.0, beam_height_deg)));

    if (out_sonar.bearings.size() != beam_count) {
        standalone_mvp::finalizeMultibeamSonarSample(out_sonar, out_sonar.beam_width, out_sonar.beam_count);
    }
    standalone_mvp::validateSonarSample(out_sonar);
    return true;
}

void reverseBeamAxisIfNeeded(sonar_types_v2::samples::Sonar& sonar) {
    if (sonar.beam_count < 2 || sonar.bearings.size() != sonar.beam_count || sonar.bin_count == 0) {
        return;
    }
    const double first = sonar.bearings.front().getDeg();
    const double last = sonar.bearings.back().getDeg();
    if (last >= first) {
        return;
    }
    std::reverse(sonar.bearings.begin(), sonar.bearings.end());
    for (std::uint32_t i = 0; i < sonar.beam_count / 2; ++i) {
        const std::uint32_t j = sonar.beam_count - 1 - i;
        const std::size_t off_i = static_cast<std::size_t>(i) * sonar.bin_count;
        const std::size_t off_j = static_cast<std::size_t>(j) * sonar.bin_count;
        for (std::uint32_t b = 0; b < sonar.bin_count; ++b) {
            std::swap(sonar.bins[off_i + b], sonar.bins[off_j + b]);
        }
    }
}

Eigen::Matrix3d rotationFromPoseJson(const QJsonObject& pose) {
    if (pose.contains(QStringLiteral("quat_w"))) {
        const Eigen::Quaterniond q(
            pose.value(QStringLiteral("quat_w")).toDouble(1.0),
            pose.value(QStringLiteral("quat_x")).toDouble(0.0),
            pose.value(QStringLiteral("quat_y")).toDouble(0.0),
            pose.value(QStringLiteral("quat_z")).toDouble(0.0));
        if (q.norm() > 1e-9) {
            return q.normalized().toRotationMatrix();
        }
    }
    const double yaw = pose.value(QStringLiteral("yaw_deg")).toDouble(0.0) * M_PI / 180.0;
    const double pitch = pose.value(QStringLiteral("pitch_deg")).toDouble(0.0) * M_PI / 180.0;
    const Eigen::AngleAxisd yaw_axis(yaw, Eigen::Vector3d::UnitZ());
    const Eigen::AngleAxisd pitch_axis(pitch, Eigen::Vector3d::UnitY());
    return (yaw_axis * pitch_axis).toRotationMatrix();
}

void fillPointCloudFromPolar(const std::vector<float>& range,
                             const std::vector<float>& intensity,
                             std::uint32_t width,
                             std::uint32_t height,
                             const QJsonObject& metadata,
                             std::uint64_t ts_us,
                             standalone_mvp::PointCloudFrame& out_frame) {
    const QJsonObject sonar_cfg = metadata.value(QStringLiteral("sonar_config")).toObject();
    const QJsonObject pose_json = metadata.value(QStringLiteral("pose")).toObject();
    const float invalid =
        static_cast<float>(metadata.value(QStringLiteral("range_invalid_value")).toDouble(-1.0));

    out_frame = standalone_mvp::PointCloudFrame{};
    out_frame.timestamp_us = ts_us;
    out_frame.pose_position_world = Eigen::Vector3d(
        pose_json.value(QStringLiteral("x")).toDouble(0.0),
        pose_json.value(QStringLiteral("y")).toDouble(0.0),
        pose_json.value(QStringLiteral("z")).toDouble(0.0));
    out_frame.pose_rotation_world = rotationFromPoseJson(pose_json);
    out_frame.pose_forward_world = out_frame.pose_rotation_world * Eigen::Vector3d::UnitX();
    out_frame.polar_frame.width = width;
    out_frame.polar_frame.height = height;
    out_frame.polar_frame.point_count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    out_frame.polar_frame.invalid_value = invalid;
    out_frame.polar_frame.range_image_m = range;
    out_frame.polar_frame.intensity_image = intensity;

    out_frame.config.enabled = true;
    out_frame.config.range_m = sonar_cfg.value(QStringLiteral("range_m")).toDouble(30.0);
    out_frame.config.frequency_khz = sonar_cfg.value(QStringLiteral("frequency_khz")).toDouble(300.0);
    out_frame.config.bandwidth_khz = sonar_cfg.value(QStringLiteral("bandwidth_khz")).toDouble(60.0);
    out_frame.config.horizontal_angle_resolution_deg =
        sonar_cfg.value(QStringLiteral("horizontal_angle_resolution_deg")).toDouble(0.75);
    out_frame.config.vertical_angle_resolution_deg =
        sonar_cfg.value(QStringLiteral("vertical_angle_resolution_deg")).toDouble(0.75);
    out_frame.config.horizontal_fov_deg = sonar_cfg.value(QStringLiteral("horizontal_fov_deg")).toDouble(90.0);
    out_frame.config.vertical_fov_deg = sonar_cfg.value(QStringLiteral("vertical_fov_deg")).toDouble(30.0);
    out_frame.config.max_point_count = static_cast<std::size_t>(
        std::max(1000, sonar_cfg.value(QStringLiteral("max_point_count")).toInt(120000)));
    out_frame.config.palette_index = sonar_cfg.value(QStringLiteral("palette_index")).toInt(1);
    out_frame.config.show_coordinate_overlay = true;

    if (width == 0 || height == 0) {
        return;
    }

    float max_range = 0.0f;
    for (float v : range) {
        if (std::isfinite(v) && v > 0.0f && std::fabs(v - invalid) > 1e-6f) {
            max_range = std::max(max_range, v);
        }
    }
    if (out_frame.config.range_m <= 0.0 && max_range > 0.0f) {
        out_frame.config.range_m = static_cast<double>(max_range);
    }

    standalone_mvp::PointCloudSonarConfig cfg = out_frame.config;
    const standalone_mvp::PointCloudSamplingInfo sampling = standalone_mvp::computePointCloudSampling(cfg);
    out_frame.config = cfg;
    out_frame.sampling = sampling;

    const int w = static_cast<int>(width);
    const int h = static_cast<int>(height);
    const double hfov_rad = out_frame.config.horizontal_fov_deg * M_PI / 180.0;
    const double vfov_rad = out_frame.config.vertical_fov_deg * M_PI / 180.0;
    const double h_step = hfov_rad / static_cast<double>(std::max<std::size_t>(1, sampling.horizontal_samples));
    const double v_step = vfov_rad / static_cast<double>(std::max<std::size_t>(1, sampling.vertical_samples));

    out_frame.points_world.reserve(sampling.budgeted_point_count);
    out_frame.point_intensities.reserve(sampling.budgeted_point_count);
    for (std::size_t v = 0; v < sampling.vertical_samples; ++v) {
        const double pitch = -vfov_rad * 0.5 + (static_cast<double>(v) + 0.5) * v_step;
        const double cos_pitch = std::cos(pitch);
        const double sin_pitch = std::sin(pitch);
        const int row = (sampling.vertical_samples <= 1)
                            ? 0
                            : static_cast<int>(std::lround(
                                  static_cast<double>(v) * static_cast<double>(h - 1) /
                                  static_cast<double>(sampling.vertical_samples - 1)));
        for (std::size_t hi = 0; hi < sampling.horizontal_samples; ++hi) {
            if (out_frame.points_world.size() >= sampling.budgeted_point_count) {
                break;
            }
            const double yaw = -hfov_rad * 0.5 + (static_cast<double>(hi) + 0.5) * h_step;
            const int col = (sampling.horizontal_samples <= 1)
                                ? 0
                                : static_cast<int>(std::lround(
                                      static_cast<double>(hi) * static_cast<double>(w - 1) /
                                      static_cast<double>(sampling.horizontal_samples - 1)));
            const std::size_t idx =
                static_cast<std::size_t>(row) * static_cast<std::size_t>(w) + static_cast<std::size_t>(col);
            if (idx >= range.size()) {
                continue;
            }
            const float r = range[idx];
            if (!std::isfinite(r) || r <= 0.0f || std::fabs(r - invalid) < 1e-6f) {
                continue;
            }
            const Eigen::Vector3d ray_sensor(
                std::cos(yaw) * cos_pitch,
                std::sin(yaw) * cos_pitch,
                sin_pitch);
            const Eigen::Vector3d p_sensor = ray_sensor * static_cast<double>(r);
            const Eigen::Vector3d pw = out_frame.pose_rotation_world * p_sensor + out_frame.pose_position_world;
            out_frame.points_world.emplace_back(
                static_cast<float>(pw.x()), static_cast<float>(pw.y()), static_cast<float>(pw.z()));
            const float inten = (idx < intensity.size()) ? intensity[idx] : 0.0f;
            out_frame.point_intensities.push_back(std::clamp(inten, 0.0f, 1.0f));
        }
    }
    out_frame.sampling.recovered_point_count = out_frame.points_world.size();
}

struct DirReplayState {
    QString cache_dir;
    QString esl2d_source;
    QString esl3d_source;
    QString module_name;
    ReplaySonarType sonar_type = ReplaySonarType::FLS;
    cv::VideoWriter fls_writer;
    cv::VideoWriter intensity_writer;
    cv::VideoWriter range_writer;
    cv::Mat sss_waterfall;
    int sss_row_count = 0;
    bool has_fls = false;
    bool has_sss = false;
    bool has_3d = false;
    QString fls_mp4_path;
    QString sss_png_path;
    QString intensity_mp4_path;
    QString range_mp4_path;
    std::vector<qint64> fls_timeline_us;
    std::vector<qint64> sss_timeline_us;
    std::vector<qint64> esl3d_timeline_us;
    double writer_fps = 20.0;
};

ReplaySonarType parseSonarTypeLabel(const QString& label) {
    if (label.compare(QStringLiteral("MBES"), Qt::CaseInsensitive) == 0) {
        return ReplaySonarType::MBES;
    }
    if (label.compare(QStringLiteral("SSS"), Qt::CaseInsensitive) == 0) {
        return ReplaySonarType::SSS;
    }
    return ReplaySonarType::FLS;
}

double computeOutputFps(const std::vector<qint64>& timeline_us,
                        double session_duration_s,
                        double fallback_fps = 20.0) {
    if (timeline_us.size() >= 2) {
        const double span_s =
            static_cast<double>(timeline_us.back() - timeline_us.front()) / 1'000'000.0;
        if (span_s > 1e-6) {
            const double span_from_session =
                session_duration_s > 1e-6 ? std::min(span_s, session_duration_s) : span_s;
            return std::clamp(
                static_cast<double>(timeline_us.size() - 1) / span_from_session, 1.0, 240.0);
        }
    }
    if (session_duration_s > 1e-6 && !timeline_us.empty()) {
        return std::clamp(static_cast<double>(timeline_us.size()) / session_duration_s, 1.0, 240.0);
    }
    return fallback_fps;
}

cv::Mat buildSssWaterfallRow(const std::vector<float>& bins, std::uint32_t bin_count) {
    if (static_cast<int>(bins.size()) < static_cast<int>(2 * bin_count) || bin_count == 0) {
        return {};
    }
    const auto* starboard = bins.data();
    const auto* port = bins.data() + bin_count;
    cv::Mat row(1, static_cast<int>(bin_count * 2), CV_8UC1);
    for (int i = 0; i < static_cast<int>(bin_count); ++i) {
        const float port_v = port[static_cast<std::size_t>(bin_count - 1 - i)];
        const float star_v = starboard[static_cast<std::size_t>(i)];
        row.at<uchar>(0, i) =
            static_cast<uchar>(std::clamp(port_v, 0.0f, 1.0f) * 255.0f);
        row.at<uchar>(0, static_cast<int>(bin_count) + i) =
            static_cast<uchar>(std::clamp(star_v, 0.0f, 1.0f) * 255.0f);
    }
    return row;
}

void indexEsl2dTimelines(const QString& file_path, std::vector<qint64>& fls_timeline_us, std::vector<qint64>& sss_timeline_us) {
    fls_timeline_us.clear();
    sss_timeline_us.clear();
    std::ifstream in(file_path.toStdString(), std::ios::binary);
    if (!in.is_open()) {
        return;
    }
    while (true) {
        std::uint32_t magic = 0;
        if (!readLE(in, magic)) {
            break;
        }
        std::uint16_t version = 0;
        std::uint16_t header_bytes = 0;
        std::uint16_t sonar_type = 0;
        std::uint16_t reserved0 = 0;
        std::uint64_t seq = 0;
        std::uint64_t ts_us = 0;
        std::uint32_t beam_count = 0;
        std::uint32_t bin_count = 0;
        float max_range_m = 0.0f;
        std::uint32_t metadata_bytes = 0;
        std::uint32_t beam_angles_bytes = 0;
        std::uint32_t intensity_bytes = 0;
        std::uint32_t payload_bytes = 0;
        std::uint32_t reserved1 = 0;
        std::uint32_t reserved2 = 0;
        if (!readLE(in, version) || !readLE(in, header_bytes) || !readLE(in, seq) || !readLE(in, ts_us) ||
            !readLE(in, sonar_type) || !readLE(in, reserved0) || !readLE(in, beam_count) ||
            !readLE(in, bin_count) || !readLE(in, max_range_m) || !readLE(in, metadata_bytes) ||
            !readLE(in, beam_angles_bytes) || !readLE(in, intensity_bytes) || !readLE(in, payload_bytes) ||
            !readLE(in, reserved1) || !readLE(in, reserved2)) {
            break;
        }
        if (magic != kEsl2dMagic || payload_bytes == 0) {
            break;
        }
        in.seekg(static_cast<std::streamoff>(payload_bytes), std::ios::cur);
        if (!in.good()) {
            break;
        }
        if (sonar_type == 0) {
            fls_timeline_us.push_back(static_cast<qint64>(ts_us));
        } else if (sonar_type == 1) {
            sss_timeline_us.push_back(static_cast<qint64>(ts_us));
        }
    }
}

void indexEsl3dTimelines(const QString& file_path, std::vector<qint64>& esl3d_timeline_us) {
    esl3d_timeline_us.clear();
    std::ifstream in(file_path.toStdString(), std::ios::binary);
    if (!in.is_open()) {
        return;
    }
    while (in.peek() != EOF) {
        std::uint32_t magic = 0;
        if (!readLE(in, magic)) {
            break;
        }
        std::uint16_t version = 0;
        std::uint16_t header_bytes = 0;
        std::uint64_t seq = 0;
        std::uint64_t ts_us = 0;
        std::uint32_t width = 0;
        std::uint32_t height = 0;
        std::uint32_t point_count = 0;
        std::uint32_t metadata_bytes = 0;
        std::uint32_t range_bytes = 0;
        std::uint32_t intensity_bytes = 0;
        std::uint32_t payload_bytes = 0;
        std::uint32_t reserved = 0;
        if (!readLE(in, version) || !readLE(in, header_bytes) || !readLE(in, seq) || !readLE(in, ts_us) ||
            !readLE(in, width) || !readLE(in, height) || !readLE(in, point_count) ||
            !readLE(in, metadata_bytes) || !readLE(in, range_bytes) || !readLE(in, intensity_bytes) ||
            !readLE(in, payload_bytes) || !readLE(in, reserved)) {
            break;
        }
        if (magic != kEsl3dMagic || payload_bytes == 0) {
            break;
        }
        esl3d_timeline_us.push_back(static_cast<qint64>(ts_us));
        in.seekg(static_cast<std::streamoff>(payload_bytes), std::ios::cur);
    }
}

QString resolveModuleName(const QString& sonar_file_path, const QHash<QString, ReplaySonarType>& type_by_dir_name) {
    const QString dir_name = QFileInfo(QFileInfo(sonar_file_path).absolutePath()).fileName();
    for (auto it = type_by_dir_name.begin(); it != type_by_dir_name.end(); ++it) {
        if (standalone_mvp::safeModuleDirName(it.key()) == dir_name) {
            return it.key();
        }
    }
    return dir_name;
}

void appendModulesFromDirState(const DirReplayState& state,
                               double session_duration_s,
                               std::vector<ReplayModuleAssets>& modules) {
    const QString module_name =
        state.module_name.isEmpty() ? QFileInfo(state.cache_dir).fileName() : state.module_name;

    if (state.has_fls && !state.fls_mp4_path.isEmpty()) {
        ReplayModuleAssets assets;
        assets.module_name = module_name;
        assets.sonar_type = state.sonar_type;
        assets.media_kind = ReplayMediaKind::Video2d;
        assets.video_path = state.fls_mp4_path;
        assets.timeline_us = state.fls_timeline_us;
        assets.frame_count = videoFrameCount(assets.video_path);
        if (assets.frame_count <= 0 && !assets.timeline_us.empty()) {
            assets.frame_count = static_cast<int>(assets.timeline_us.size());
        }
        assets.output_fps = computeOutputFps(assets.timeline_us, session_duration_s);
        modules.push_back(std::move(assets));
    }

    if (state.has_sss && !state.sss_png_path.isEmpty()) {
        ReplayModuleAssets assets;
        assets.module_name = module_name;
        assets.sonar_type = ReplaySonarType::SSS;
        assets.media_kind = ReplayMediaKind::SssWaterfall;
        assets.sss_png_path = state.sss_png_path;
        assets.timeline_us = state.sss_timeline_us;
        assets.sss_row_count = state.sss_row_count;
        if (assets.sss_row_count <= 0) {
            const cv::Mat png = cv::imread(state.sss_png_path.toStdString(), cv::IMREAD_COLOR);
            assets.sss_row_count = png.empty() ? 0 : png.rows;
        }
        assets.frame_count = std::max(assets.sss_row_count, static_cast<int>(assets.timeline_us.size()));
        assets.output_fps = computeOutputFps(assets.timeline_us, session_duration_s);
        modules.push_back(std::move(assets));
    }

    if (state.has_3d && !state.intensity_mp4_path.isEmpty() && !state.range_mp4_path.isEmpty()) {
        ReplayModuleAssets assets;
        assets.module_name = module_name;
        assets.sonar_type = state.sonar_type;
        assets.media_kind = ReplayMediaKind::PointCloud3d;
        assets.intensity_mp4_path = state.intensity_mp4_path;
        assets.range_mp4_path = state.range_mp4_path;
        assets.esl3d_source_path = state.esl3d_source;
        assets.timeline_us = state.esl3d_timeline_us;
        if (!assets.esl3d_source_path.isEmpty()) {
            assets.frame_count =
                indexEsl3dFrameOffsets(assets.esl3d_source_path, assets.esl3d_frame_offsets);
        }
        if (assets.frame_count <= 0) {
            assets.frame_count = std::max(
                videoFrameCount(assets.intensity_mp4_path),
                videoFrameCount(assets.range_mp4_path));
        }
        assets.output_fps = computeOutputFps(assets.timeline_us, session_duration_s);
        modules.push_back(std::move(assets));
    }
}

void mergeGlobalTimeline(const std::vector<ReplayModuleAssets>& modules, std::vector<qint64>& out_timeline_us) {
    out_timeline_us.clear();
    for (const ReplayModuleAssets& module : modules) {
        out_timeline_us.insert(out_timeline_us.end(), module.timeline_us.begin(), module.timeline_us.end());
    }
    std::sort(out_timeline_us.begin(), out_timeline_us.end());
    out_timeline_us.erase(
        std::unique(out_timeline_us.begin(), out_timeline_us.end()),
        out_timeline_us.end());
}

int countEsl2dPackets(const QString& file_path) {
    std::vector<qint64> fls_timeline_us;
    std::vector<qint64> sss_timeline_us;
    indexEsl2dTimelines(file_path, fls_timeline_us, sss_timeline_us);
    return static_cast<int>(std::max(fls_timeline_us.size(), sss_timeline_us.size()));
}

int countEsl3dPackets(const QString& file_path) {
    std::vector<qint64> esl3d_timeline_us;
    indexEsl3dTimelines(file_path, esl3d_timeline_us);
    return static_cast<int>(esl3d_timeline_us.size());
}

bool esl2dNeedsConversion(const QString& file_path, const QHash<QString, ReplaySonarType>& type_by_dir_name) {
    const QString cache_dir = replayCacheDirForSonarFile(file_path);
    const bool fls_ready = cacheFileReady(flsCachePath(cache_dir, file_path));
    const bool sss_ready = cacheFileReady(sssCachePath(cache_dir, file_path));
    const QString module_name = resolveModuleName(file_path, type_by_dir_name);
    const ReplaySonarType sonar_type = type_by_dir_name.value(module_name, ReplaySonarType::FLS);
    if (sonar_type == ReplaySonarType::SSS) {
        return !sss_ready;
    }
    return !fls_ready;
}

bool esl3dNeedsConversion(const QString& file_path) {
    const QString cache_dir = replayCacheDirForSonarFile(file_path);
    return !(cacheFileReady(rangeCachePath(cache_dir, file_path)) &&
             cacheFileReady(intensityCachePath(cache_dir, file_path)));
}

class ConversionProgressReporter {
public:
    ConversionProgressReporter(QWidget* parent, int total_steps, const QString& title)
        : total_(std::max(1, total_steps)) {
        if (!parent) {
            return;
        }
        dialog_ = new QProgressDialog(
            QStringLiteral("正在分析回放文件..."),
            QString(),
            0,
            total_,
            parent);
        dialog_->setWindowTitle(title);
        dialog_->setWindowModality(Qt::WindowModal);
        dialog_->setMinimumDuration(0);
        dialog_->setAutoClose(true);
        dialog_->setAutoReset(false);
        dialog_->setValue(0);
        dialog_->show();
        QApplication::processEvents();
    }

    ~ConversionProgressReporter() {
        finish();
    }

    void setLabel(const QString& text) {
        if (!dialog_) {
            return;
        }
        dialog_->setLabelText(text);
        QApplication::processEvents();
    }

    void advance(int steps = 1) {
        value_ = std::min(total_, value_ + std::max(1, steps));
        if (!dialog_) {
            return;
        }
        dialog_->setValue(value_);
        QApplication::processEvents();
    }

    void finish() {
        if (!dialog_) {
            return;
        }
        dialog_->setValue(total_);
        dialog_->close();
        delete dialog_;
        dialog_ = nullptr;
    }

private:
    QProgressDialog* dialog_ = nullptr;
    int value_ = 0;
    int total_ = 1;
};

} // namespace

int computeReplayMaxFrame(const ReplayConversionResult& result) {
    int max_frame = 0;
    for (const ReplayModuleAssets& module : result.modules) {
        max_frame = std::max(max_frame, std::max(0, module.frame_count - 1));
    }
    if (!result.timeline_us.empty()) {
        max_frame = std::max(max_frame, static_cast<int>(result.timeline_us.size()) - 1);
    }
    return std::max(0, max_frame);
}

int computeReplayIntervalMs(const ReplayConversionResult& result, int replay_max_frame) {
    if (result.session_duration_seconds > 1e-6 && replay_max_frame > 0) {
        return std::max(
            10,
            static_cast<int>(result.session_duration_seconds * 1000.0 / static_cast<double>(replay_max_frame)));
    }
    if (result.timeline_us.size() >= 2) {
        const double span_s =
            static_cast<double>(result.timeline_us.back() - result.timeline_us.front()) / 1'000'000.0;
        const double effective_span =
            result.session_duration_seconds > 1e-6 ? std::min(span_s, result.session_duration_seconds) : span_s;
        if (effective_span > 1e-6) {
            return std::max(
                10,
                static_cast<int>(effective_span * 1000.0 /
                                 static_cast<double>(result.timeline_us.size() - 1)));
        }
    }
    return 40;
}

QString replayTabTitle(const ReplayModuleAssets& module) {
    QString type_label;
    switch (module.media_kind) {
    case ReplayMediaKind::PointCloud3d:
        type_label = QStringLiteral("3D");
        break;
    case ReplayMediaKind::SssWaterfall:
        type_label = QStringLiteral("SSS");
        break;
    case ReplayMediaKind::Video2d:
        switch (module.sonar_type) {
        case ReplaySonarType::MBES:
            type_label = QStringLiteral("MBES");
            break;
        case ReplaySonarType::SSS:
            type_label = QStringLiteral("SSS");
            break;
        default:
            type_label = QStringLiteral("FLS");
            break;
        }
        break;
    }
    return QStringLiteral("%1 (%2)").arg(module.module_name, type_label);
}

QStringList listSonarDataSessions(const QString& project_dir) {
    QStringList sessions;
    const QDir sonar_data_dir(QDir(project_dir).filePath(QStringLiteral("Sonar Data")));
    if (!sonar_data_dir.exists()) {
        return sessions;
    }
    const QFileInfoList entries =
        sonar_data_dir.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Time);
    for (const QFileInfo& entry : entries) {
        if (QFileInfo::exists(QDir(entry.absoluteFilePath()).filePath(QStringLiteral("recording_summary.json")))) {
            sessions.push_back(entry.absoluteFilePath());
        }
    }
    return sessions;
}

QString selectReplaySessionFolder(QWidget* parent, const QString& project_dir) {
    const QStringList sessions = listSonarDataSessions(project_dir);
    QDialog dialog(parent);
    dialog.setWindowTitle(QStringLiteral("选择回放录制"));
    dialog.resize(520, 420);
    auto* layout = new QVBoxLayout(&dialog);
    layout->addWidget(new QLabel(QStringLiteral("请选择 Sonar Data 下的录制时间戳文件夹："), &dialog));

    auto* list = new QListWidget(&dialog);
    for (const QString& session : sessions) {
        const QString name = QFileInfo(session).fileName();
        QString detail;
        QFile summary(QDir(session).filePath(QStringLiteral("recording_summary.json")));
        if (summary.open(QIODevice::ReadOnly)) {
            const QJsonObject root = QJsonDocument::fromJson(summary.readAll()).object();
            detail = QStringLiteral("  (%1 modules)")
                         .arg(root.value(QStringLiteral("modules")).toArray().size());
        }
        auto* item = new QListWidgetItem(name + detail, list);
        item->setData(Qt::UserRole, session);
    }
    if (list->count() > 0) {
        list->setCurrentRow(0);
    } else {
        auto* hint = new QListWidgetItem(QStringLiteral("（Sonar Data 下无录制，请使用自定义文件夹）"), list);
        hint->setFlags(Qt::NoItemFlags);
    }
    layout->addWidget(list, 1);

    auto* buttons = new QHBoxLayout();
    auto* ok_btn = new QPushButton(QStringLiteral("开始回放"), &dialog);
    auto* custom_btn = new QPushButton(QStringLiteral("自定义文件夹..."), &dialog);
    auto* cancel_btn = new QPushButton(QStringLiteral("取消"), &dialog);
    buttons->addWidget(ok_btn);
    buttons->addWidget(custom_btn);
    buttons->addStretch();
    buttons->addWidget(cancel_btn);
    layout->addLayout(buttons);

    QString chosen;
    QObject::connect(ok_btn, &QPushButton::clicked, &dialog, [&]() {
        if (QListWidgetItem* item = list->currentItem()) {
            chosen = item->data(Qt::UserRole).toString();
        }
        if (chosen.isEmpty()) {
            QMessageBox::information(&dialog, QStringLiteral("回放模式"), QStringLiteral("请选择一个录制文件夹。"));
            return;
        }
        dialog.accept();
    });
    QObject::connect(custom_btn, &QPushButton::clicked, &dialog, [&]() {
        const QString folder = QFileDialog::getExistingDirectory(
            &dialog,
            QStringLiteral("选择声呐录制目录"),
            project_dir,
            QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
        if (folder.isEmpty()) {
            return;
        }
        if (!QFileInfo::exists(QDir(folder).filePath(QStringLiteral("recording_summary.json")))) {
            QMessageBox::warning(
                &dialog,
                QStringLiteral("回放模式"),
                QStringLiteral("所选文件夹必须包含 recording_summary.json。"));
            return;
        }
        chosen = folder;
        dialog.accept();
    });
    QObject::connect(cancel_btn, &QPushButton::clicked, &dialog, &QDialog::reject);
    QObject::connect(list, &QListWidget::itemDoubleClicked, ok_btn, &QPushButton::click);

    if (dialog.exec() != QDialog::Accepted) {
        return {};
    }
    return chosen;
}

int indexEsl3dFrameOffsets(const QString& esl3d_path, std::vector<qint64>& out_offsets) {
    out_offsets.clear();
    std::ifstream in(esl3d_path.toStdString(), std::ios::binary);
    if (!in.is_open()) {
        return 0;
    }
    while (in.peek() != EOF) {
        const auto pos = static_cast<qint64>(in.tellg());
        std::uint32_t magic = 0;
        if (!readLE(in, magic)) {
            break;
        }
        std::uint16_t version = 0;
        std::uint16_t header_bytes = 0;
        std::uint64_t seq = 0;
        std::uint64_t ts_us = 0;
        std::uint32_t width = 0;
        std::uint32_t height = 0;
        std::uint32_t point_count = 0;
        std::uint32_t metadata_bytes = 0;
        std::uint32_t range_bytes = 0;
        std::uint32_t intensity_bytes = 0;
        std::uint32_t payload_bytes = 0;
        std::uint32_t reserved = 0;
        if (!readLE(in, version) || !readLE(in, header_bytes) || !readLE(in, seq) || !readLE(in, ts_us) ||
            !readLE(in, width) || !readLE(in, height) || !readLE(in, point_count) || !readLE(in, metadata_bytes) ||
            !readLE(in, range_bytes) || !readLE(in, intensity_bytes) || !readLE(in, payload_bytes) ||
            !readLE(in, reserved)) {
            break;
        }
        if (magic != kEsl3dMagic || payload_bytes == 0) {
            break;
        }
        out_offsets.push_back(pos);
        in.seekg(static_cast<std::streamoff>(payload_bytes), std::ios::cur);
    }
    return static_cast<int>(out_offsets.size());
}

bool readEsl3dPointCloudFrame(const QString& esl3d_path,
                              const std::vector<qint64>& frame_offsets,
                              int frame_index,
                              standalone_mvp::PointCloudFrame& out_frame) {
    if (esl3d_path.isEmpty() || frame_index < 0 ||
        frame_index >= static_cast<int>(frame_offsets.size())) {
        return false;
    }
    std::ifstream in(esl3d_path.toStdString(), std::ios::binary);
    if (!in.is_open()) {
        return false;
    }
    in.seekg(static_cast<std::streamoff>(frame_offsets[static_cast<std::size_t>(frame_index)]));

    std::uint32_t magic = 0;
    if (!readLE(in, magic) || magic != kEsl3dMagic) {
        return false;
    }
    std::uint16_t version = 0;
    std::uint16_t header_bytes = 0;
    std::uint64_t seq = 0;
    std::uint64_t ts_us = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t point_count = 0;
    std::uint32_t metadata_bytes = 0;
    std::uint32_t range_bytes = 0;
    std::uint32_t intensity_bytes = 0;
    std::uint32_t payload_bytes = 0;
    std::uint32_t reserved = 0;
    if (!readLE(in, version) || !readLE(in, header_bytes) || !readLE(in, seq) || !readLE(in, ts_us) ||
        !readLE(in, width) || !readLE(in, height) || !readLE(in, point_count) || !readLE(in, metadata_bytes) ||
        !readLE(in, range_bytes) || !readLE(in, intensity_bytes) || !readLE(in, payload_bytes) ||
        !readLE(in, reserved)) {
        return false;
    }
    std::vector<char> payload(payload_bytes);
    in.read(payload.data(), static_cast<std::streamsize>(payload.size()));
    if (!in.good() || metadata_bytes + range_bytes + intensity_bytes > payload.size()) {
        return false;
    }
    const QJsonObject metadata = parseMetadataObject(payload, metadata_bytes);
    const float* range_ptr = reinterpret_cast<const float*>(payload.data() + metadata_bytes);
    const float* inten_ptr = reinterpret_cast<const float*>(payload.data() + metadata_bytes + range_bytes);
    const int elem_count = static_cast<int>(
        std::min<std::uint32_t>(width * height, std::min(range_bytes, intensity_bytes) / 4));
    std::vector<float> range_data(range_ptr, range_ptr + elem_count);
    std::vector<float> inten_data(inten_ptr, inten_ptr + elem_count);
    fillPointCloudFromPolar(range_data, inten_data, width, height, metadata, ts_us, out_frame);
    return !out_frame.points_world.empty();
}

bool loadOrConvert(const QString& folder, ReplayConversionResult& out_result, QString& err, QWidget* progress_parent) {
    out_result = {};
    const QString summary_path = QDir(folder).filePath(QStringLiteral("recording_summary.json"));
    if (!QFileInfo::exists(summary_path)) {
        err = QStringLiteral("请选择包含 recording_summary.json 的录制根目录。");
        return false;
    }
    out_result.session_folder = QFileInfo(folder).absoluteFilePath();

    QHash<QString, ReplaySonarType> type_by_dir_name;
    QFile summary_file(summary_path);
    if (summary_file.open(QIODevice::ReadOnly)) {
        const QJsonObject root = QJsonDocument::fromJson(summary_file.readAll()).object();
        out_result.session_duration_seconds = root.value(QStringLiteral("duration_seconds")).toDouble(0.0);
        const QJsonArray modules = root.value(QStringLiteral("modules")).toArray();
        for (const QJsonValue& value : modules) {
            const QJsonObject entry = value.toObject();
            const QString module_name = entry.value(QStringLiteral("module_name")).toString();
            if (module_name.isEmpty()) {
                continue;
            }
            type_by_dir_name[module_name] =
                parseSonarTypeLabel(entry.value(QStringLiteral("sonar_type")).toString());
        }
    }

    QStringList esl2d_paths;
    QStringList esl3d_paths;
    QDirIterator it(
        folder,
        QStringList() << QStringLiteral("*.esl2d") << QStringLiteral("*.esl3d"),
        QDir::Files,
        QDirIterator::Subdirectories);
    while (it.hasNext()) {
        const QString path = it.next();
        if (path.endsWith(QStringLiteral(".esl2d"), Qt::CaseInsensitive)) {
            esl2d_paths.push_back(path);
        } else if (path.endsWith(QStringLiteral(".esl3d"), Qt::CaseInsensitive)) {
            esl3d_paths.push_back(path);
        }
    }
    if (esl2d_paths.isEmpty() && esl3d_paths.isEmpty()) {
        err = QStringLiteral("recording_summary.json存在，但子目录中未检测到 .esl2d 或 .esl3d 文件。");
        return false;
    }

    int total_progress_steps = 0;
    int converting_file_count = 0;
    for (const QString& file_path : esl2d_paths) {
        if (esl2dNeedsConversion(file_path, type_by_dir_name)) {
            total_progress_steps += std::max(1, countEsl2dPackets(file_path));
            ++converting_file_count;
        }
    }
    for (const QString& file_path : esl3d_paths) {
        if (esl3dNeedsConversion(file_path)) {
            total_progress_steps += std::max(1, countEsl3dPackets(file_path));
            ++converting_file_count;
        }
    }
    if (total_progress_steps == 0) {
        total_progress_steps = std::max(1, static_cast<int>(esl2d_paths.size() + esl3d_paths.size()));
    } else {
        total_progress_steps += converting_file_count;
    }
    total_progress_steps += 2;

    ConversionProgressReporter progress(
        progress_parent,
        total_progress_steps,
        QStringLiteral("回放转码"));
    progress.setLabel(converting_file_count > 0 ? QStringLiteral("正在准备转码...")
                                                : QStringLiteral("正在加载缓存回放数据..."));

    QHash<QString, DirReplayState> dir_states;
    bool any_used_cache = false;

    for (const QString& file_path : esl2d_paths) {
        const QString cache_dir = replayCacheDirForSonarFile(file_path);
        QDir().mkpath(cache_dir);
        DirReplayState& state = dir_states[cache_dir];
        state.cache_dir = cache_dir;
        state.esl2d_source = file_path;
        if (state.module_name.isEmpty()) {
            state.module_name = resolveModuleName(file_path, type_by_dir_name);
            state.sonar_type = type_by_dir_name.value(
                state.module_name, ReplaySonarType::FLS);
        }

        const QString fls_cached = flsCachePath(cache_dir, file_path);
        const QString sss_cached = sssCachePath(cache_dir, file_path);
        const bool fls_ready = cacheFileReady(fls_cached);
        const bool sss_ready = cacheFileReady(sss_cached);
        if (fls_ready) {
            state.fls_mp4_path = fls_cached;
            state.has_fls = true;
            any_used_cache = true;
            indexEsl2dTimelines(file_path, state.fls_timeline_us, state.sss_timeline_us);
        }
        if (sss_ready) {
            state.sss_png_path = sss_cached;
            state.has_sss = true;
            any_used_cache = true;
            if (state.sss_timeline_us.empty()) {
                indexEsl2dTimelines(file_path, state.fls_timeline_us, state.sss_timeline_us);
            }
            const cv::Mat png = cv::imread(sss_cached.toStdString(), cv::IMREAD_COLOR);
            state.sss_row_count = png.empty() ? 0 : png.rows;
        }
        if (fls_ready && sss_ready) {
            progress.advance(1);
            continue;
        }
        if (fls_ready && state.sonar_type != ReplaySonarType::SSS) {
            progress.setLabel(QStringLiteral("加载 2D 缓存：%1").arg(state.module_name));
            progress.advance(1);
            continue;
        }
        if (sss_ready && state.sonar_type == ReplaySonarType::SSS) {
            progress.setLabel(QStringLiteral("加载 SSS 缓存：%1").arg(state.module_name));
            progress.advance(1);
            continue;
        }

        int converted_frames = 0;
        progress.setLabel(QStringLiteral("转码 2D：%1").arg(state.module_name));
        {
            std::vector<qint64> preview_fls;
            std::vector<qint64> preview_sss;
            indexEsl2dTimelines(file_path, preview_fls, preview_sss);
            const std::vector<qint64>& preview_timeline =
                state.sonar_type == ReplaySonarType::SSS ? preview_sss : preview_fls;
            state.writer_fps = computeOutputFps(preview_timeline, out_result.session_duration_seconds);
        }

        std::ifstream in(file_path.toStdString(), std::ios::binary);
        if (!in.is_open()) {
            continue;
        }
        while (true) {
            std::uint32_t magic = 0;
            if (!readLE(in, magic)) {
                break;
            }
            std::uint16_t version = 0;
            std::uint16_t header_bytes = 0;
            std::uint16_t sonar_type = 0;
            std::uint16_t reserved0 = 0;
            std::uint64_t seq = 0;
            std::uint64_t ts_us = 0;
            std::uint32_t beam_count = 0;
            std::uint32_t bin_count = 0;
            std::uint32_t metadata_bytes = 0;
            std::uint32_t beam_angles_bytes = 0;
            std::uint32_t intensity_bytes = 0;
            std::uint32_t payload_bytes = 0;
            std::uint32_t reserved1 = 0;
            std::uint32_t reserved2 = 0;
            float max_range_m = 0.0f;
            if (!readLE(in, version) || !readLE(in, header_bytes) || !readLE(in, seq) || !readLE(in, ts_us) ||
                !readLE(in, sonar_type) || !readLE(in, reserved0) || !readLE(in, beam_count) ||
                !readLE(in, bin_count) || !readLE(in, max_range_m) || !readLE(in, metadata_bytes) ||
                !readLE(in, beam_angles_bytes) || !readLE(in, intensity_bytes) || !readLE(in, payload_bytes) ||
                !readLE(in, reserved1) || !readLE(in, reserved2)) {
                break;
            }
            if (magic != kEsl2dMagic || payload_bytes == 0) {
                in.seekg(0, std::ios::end);
                break;
            }
            std::vector<char> payload(payload_bytes);
            in.read(payload.data(), static_cast<std::streamsize>(payload.size()));
            if (!in.good()) {
                break;
            }
            const std::size_t inten_off = static_cast<std::size_t>(metadata_bytes + beam_angles_bytes);
            if (inten_off + intensity_bytes > payload.size()) {
                continue;
            }
            const float* intensity = reinterpret_cast<const float*>(payload.data() + inten_off);
            const std::size_t total = static_cast<std::size_t>(beam_count) * static_cast<std::size_t>(bin_count);
            std::vector<float> bins(
                intensity, intensity + std::min(total, static_cast<std::size_t>(intensity_bytes / 4)));
            const QJsonObject metadata = parseMetadataObject(payload, metadata_bytes);
            if (state.module_name.isEmpty()) {
                state.module_name = metadata.value(QStringLiteral("sonar_module_name")).toString();
            }

            if (sonar_type == 0 && !fls_ready) {
                state.fls_timeline_us.push_back(static_cast<qint64>(ts_us));
                sonar_types_v2::samples::Sonar sonar;
                if (!buildFlsSonarFromEsl2d(
                        bins, payload, metadata_bytes, beam_angles_bytes, beam_count, bin_count, sonar)) {
                    continue;
                }
                reverseBeamAxisIfNeeded(sonar);
                const int overlay_range_m = std::max(1, static_cast<int>(std::lround(max_range_m)));
                cv::Mat frame = standalone_mvp::renderSonarLikeSonarWidget(
                    sonar, kFlsPlotWidth, kFlsPlotHeight, overlay_range_m);
                if (frame.empty()) {
                    continue;
                }
                if (!state.fls_writer.isOpened()) {
                    state.fls_mp4_path = fls_cached;
                    state.fls_writer.open(
                        state.fls_mp4_path.toStdString(),
                        cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                        state.writer_fps,
                        frame.size(),
                        true);
                }
                state.fls_writer.write(frame);
                state.has_fls = true;
                ++converted_frames;
                progress.setLabel(QStringLiteral("转码 2D：%1（%2 帧）")
                                      .arg(state.module_name)
                                      .arg(converted_frames));
                progress.advance(1);
            } else if (sonar_type == 1 && !sss_ready &&
                       static_cast<int>(bins.size()) >= static_cast<int>(2 * bin_count)) {
                state.sss_timeline_us.push_back(static_cast<qint64>(ts_us));
                cv::Mat row = buildSssWaterfallRow(bins, bin_count);
                if (row.empty()) {
                    continue;
                }
                if (state.sss_waterfall.empty()) {
                    state.sss_waterfall = row.clone();
                } else {
                    cv::vconcat(state.sss_waterfall, row, state.sss_waterfall);
                }
                state.sss_row_count = state.sss_waterfall.rows;
                state.has_sss = true;
                ++converted_frames;
                progress.setLabel(QStringLiteral("转码 SSS：%1（%2 ping）")
                                      .arg(state.module_name)
                                      .arg(converted_frames));
                progress.advance(1);
            }
        }
        if (converted_frames > 0) {
            progress.advance(1);
        }
    }

    for (const QString& file_path : esl3d_paths) {
        const QString cache_dir = replayCacheDirForSonarFile(file_path);
        QDir().mkpath(cache_dir);
        DirReplayState& state = dir_states[cache_dir];
        state.cache_dir = cache_dir;
        state.esl3d_source = file_path;
        if (state.module_name.isEmpty()) {
            state.module_name = resolveModuleName(file_path, type_by_dir_name);
            state.sonar_type = type_by_dir_name.value(
                state.module_name, ReplaySonarType::FLS);
        }

        const QString range_cached = rangeCachePath(cache_dir, file_path);
        const QString intensity_cached = intensityCachePath(cache_dir, file_path);
        const bool range_ready = cacheFileReady(range_cached);
        const bool intensity_ready = cacheFileReady(intensity_cached);
        if (range_ready && intensity_ready) {
            state.range_mp4_path = range_cached;
            state.intensity_mp4_path = intensity_cached;
            state.has_3d = true;
            any_used_cache = true;
            indexEsl3dTimelines(file_path, state.esl3d_timeline_us);
            progress.setLabel(QStringLiteral("加载 3D 缓存：%1").arg(state.module_name));
            progress.advance(1);
            continue;
        }

        int converted_frames = 0;
        progress.setLabel(QStringLiteral("转码 3D：%1").arg(state.module_name));
        {
            std::vector<qint64> preview_3d;
            indexEsl3dTimelines(file_path, preview_3d);
            state.writer_fps = computeOutputFps(preview_3d, out_result.session_duration_seconds);
        }
        state.esl3d_timeline_us.clear();

        std::ifstream in(file_path.toStdString(), std::ios::binary);
        if (!in.is_open()) {
            continue;
        }
        while (true) {
            std::uint32_t magic = 0;
            if (!readLE(in, magic)) {
                break;
            }
            std::uint16_t version = 0;
            std::uint16_t header_bytes = 0;
            std::uint64_t seq = 0;
            std::uint64_t ts_us = 0;
            std::uint32_t width = 0;
            std::uint32_t height = 0;
            std::uint32_t point_count = 0;
            std::uint32_t metadata_bytes = 0;
            std::uint32_t range_bytes = 0;
            std::uint32_t intensity_bytes = 0;
            std::uint32_t payload_bytes = 0;
            std::uint32_t reserved = 0;
            if (!readLE(in, version) || !readLE(in, header_bytes) || !readLE(in, seq) || !readLE(in, ts_us) ||
                !readLE(in, width) || !readLE(in, height) || !readLE(in, point_count) ||
                !readLE(in, metadata_bytes) || !readLE(in, range_bytes) || !readLE(in, intensity_bytes) ||
                !readLE(in, payload_bytes) || !readLE(in, reserved)) {
                break;
            }
            if (magic != kEsl3dMagic || payload_bytes == 0) {
                in.seekg(0, std::ios::end);
                break;
            }
            std::vector<char> payload(payload_bytes);
            in.read(payload.data(), static_cast<std::streamsize>(payload.size()));
            if (!in.good()) {
                break;
            }
            if (metadata_bytes + range_bytes + intensity_bytes > payload.size()) {
                continue;
            }
            const float* range_ptr = reinterpret_cast<const float*>(payload.data() + metadata_bytes);
            const float* inten_ptr = reinterpret_cast<const float*>(payload.data() + metadata_bytes + range_bytes);
            const int elem_count = static_cast<int>(
                std::min<std::uint32_t>(width * height, std::min(range_bytes, intensity_bytes) / 4));
            std::vector<float> range_data(range_ptr, range_ptr + elem_count);
            std::vector<float> inten_data(inten_ptr, inten_ptr + elem_count);
            cv::Mat range_img = normalizeFloatImage(range_data, static_cast<int>(height), static_cast<int>(width));
            cv::Mat inten_img = normalizeFloatImage(inten_data, static_cast<int>(height), static_cast<int>(width));
            if (range_img.empty() || inten_img.empty()) {
                continue;
            }
            cv::cvtColor(range_img, range_img, cv::COLOR_GRAY2BGR);
            cv::cvtColor(inten_img, inten_img, cv::COLOR_GRAY2BGR);
            state.esl3d_timeline_us.push_back(static_cast<qint64>(ts_us));
            if (!state.range_writer.isOpened()) {
                state.range_mp4_path = range_cached;
                state.intensity_mp4_path = intensity_cached;
                state.range_writer.open(
                    state.range_mp4_path.toStdString(),
                    cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                    state.writer_fps,
                    range_img.size(),
                    true);
                state.intensity_writer.open(
                    state.intensity_mp4_path.toStdString(),
                    cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                    state.writer_fps,
                    inten_img.size(),
                    true);
            }
            state.range_writer.write(range_img);
            state.intensity_writer.write(inten_img);
            state.has_3d = true;
            ++converted_frames;
            progress.setLabel(QStringLiteral("转码 3D：%1（%2 帧）")
                                  .arg(state.module_name)
                                  .arg(converted_frames));
            progress.advance(1);
        }
        if (converted_frames > 0) {
            progress.advance(1);
        }
    }

    progress.setLabel(QStringLiteral("正在整理回放模块..."));
    for (auto it = dir_states.begin(); it != dir_states.end(); ++it) {
        DirReplayState& state = it.value();
        state.fls_writer.release();
        state.range_writer.release();
        state.intensity_writer.release();
        if (state.has_sss && !state.sss_waterfall.empty() && !cacheFileReady(state.sss_png_path)) {
            progress.setLabel(QStringLiteral("保存 SSS 瀑布图：%1").arg(state.module_name));
            cv::Mat colored;
            cv::applyColorMap(state.sss_waterfall, colored, cv::COLORMAP_JET);
            state.sss_png_path = sssCachePath(state.cache_dir, state.esl2d_source);
            cv::imwrite(state.sss_png_path.toStdString(), colored);
            progress.advance(1);
        }
        if (out_result.output_dir.isEmpty()) {
            out_result.output_dir = state.cache_dir;
        }
        appendModulesFromDirState(state, out_result.session_duration_seconds, out_result.modules);
    }
    progress.advance(1);

    mergeGlobalTimeline(out_result.modules, out_result.timeline_us);
    std::sort(out_result.modules.begin(), out_result.modules.end(), [](const ReplayModuleAssets& a, const ReplayModuleAssets& b) {
        if (a.module_name != b.module_name) {
            return a.module_name < b.module_name;
        }
        return static_cast<int>(a.media_kind) < static_cast<int>(b.media_kind);
    });
    if (out_result.timeline_us.size() < 2 && out_result.session_duration_seconds > 0.0) {
        int max_frames = 0;
        for (const ReplayModuleAssets& module : out_result.modules) {
            max_frames = std::max(max_frames, module.frame_count);
        }
        if (max_frames > 1) {
            out_result.timeline_us.resize(static_cast<std::size_t>(max_frames));
            const qint64 duration_us =
                static_cast<qint64>(out_result.session_duration_seconds * 1'000'000.0);
            for (int i = 0; i < max_frames; ++i) {
                out_result.timeline_us[static_cast<std::size_t>(i)] =
                    static_cast<qint64>(static_cast<double>(i) * static_cast<double>(duration_us) /
                                        static_cast<double>(max_frames - 1));
            }
        }
    }
    out_result.used_cache = any_used_cache;

    progress.setLabel(QStringLiteral("回放数据已就绪。"));
    progress.finish();

    const QString timeline_dir =
        out_result.output_dir.isEmpty() ? QFileInfo(summary_path).absolutePath() : out_result.output_dir;
    if (!any_used_cache && !out_result.timeline_us.empty()) {
        QFile timeline(QDir(timeline_dir).filePath(QStringLiteral("timeline.csv")));
        if (timeline.open(QIODevice::WriteOnly | QIODevice::Text)) {
            timeline.write("frame,timestamp_us\n");
            for (int i = 0; i < static_cast<int>(out_result.timeline_us.size()); ++i) {
                timeline.write(
                    QString("%1,%2\n")
                        .arg(i)
                        .arg(out_result.timeline_us[static_cast<std::size_t>(i)])
                        .toUtf8());
            }
        }
    }

    return !out_result.modules.empty();
}

} // namespace replay_mode
