#include "ReplayMode.hpp"
#include "RockSonarPlotView.hpp"

#include <QDialog>
#include <QDir>
#include <QDirIterator>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QHash>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
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
};

void mergeDirState(const DirReplayState& state, ReplayConversionResult& out_result) {
    if (out_result.output_dir.isEmpty()) {
        out_result.output_dir = state.cache_dir;
    }
    if (state.has_fls && out_result.fls_mp4_path.isEmpty()) {
        out_result.fls_mp4_path = state.fls_mp4_path;
        out_result.has_fls = true;
    }
    if (state.has_sss && out_result.sss_png_path.isEmpty()) {
        out_result.sss_png_path = state.sss_png_path;
        const cv::Mat png = cv::imread(state.sss_png_path.toStdString(), cv::IMREAD_GRAYSCALE);
        out_result.sss_row_count = png.empty() ? state.sss_row_count : png.rows;
        out_result.has_sss = true;
    }
    if (state.has_3d && out_result.intensity_mp4_path.isEmpty()) {
        out_result.intensity_mp4_path = state.intensity_mp4_path;
        out_result.range_mp4_path = state.range_mp4_path;
        out_result.esl3d_source_path = state.esl3d_source;
        out_result.has_3d = true;
    }
}

} // namespace

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

bool loadOrConvert(const QString& folder, ReplayConversionResult& out_result, QString& err) {
    out_result = {};
    const QString summary_path = QDir(folder).filePath(QStringLiteral("recording_summary.json"));
    if (!QFileInfo::exists(summary_path)) {
        err = QStringLiteral("请选择包含 recording_summary.json 的录制根目录。");
        return false;
    }
    out_result.session_folder = QFileInfo(folder).absoluteFilePath();

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

    QHash<QString, DirReplayState> dir_states;
    bool any_used_cache = false;

    for (const QString& file_path : esl2d_paths) {
        const QString cache_dir = replayCacheDirForSonarFile(file_path);
        QDir().mkpath(cache_dir);
        DirReplayState& state = dir_states[cache_dir];
        state.cache_dir = cache_dir;
        state.esl2d_source = file_path;

        const QString fls_cached = flsCachePath(cache_dir, file_path);
        const QString sss_cached = sssCachePath(cache_dir, file_path);
        const bool fls_ready = cacheFileReady(fls_cached);
        const bool sss_ready = cacheFileReady(sss_cached);
        if (fls_ready) {
            state.fls_mp4_path = fls_cached;
            state.has_fls = true;
            any_used_cache = true;
        }
        if (sss_ready) {
            state.sss_png_path = sss_cached;
            state.has_sss = true;
            any_used_cache = true;
        }
        if (fls_ready && sss_ready) {
            continue;
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
            out_result.timeline_us.push_back(static_cast<qint64>(ts_us));

            if (sonar_type == 0 && !fls_ready) {
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
                        20.0,
                        frame.size(),
                        true);
                }
                state.fls_writer.write(frame);
                state.has_fls = true;
            } else if (sonar_type == 1 && !sss_ready &&
                       static_cast<int>(bins.size()) >= static_cast<int>(2 * bin_count)) {
                cv::Mat row(1, static_cast<int>(bin_count * 2), CV_8UC1);
                const auto* star = bins.data();
                const auto* port = bins.data() + bin_count;
                float min_v = std::numeric_limits<float>::max();
                float max_v = std::numeric_limits<float>::lowest();
                for (int i = 0; i < static_cast<int>(bin_count); ++i) {
                    min_v = std::min(min_v, std::min(star[i], port[i]));
                    max_v = std::max(max_v, std::max(star[i], port[i]));
                }
                const float denom = std::max(1e-6f, max_v - min_v);
                for (int i = 0; i < static_cast<int>(bin_count); ++i) {
                    row.at<uchar>(0, i) =
                        static_cast<uchar>(std::clamp((port[i] - min_v) / denom, 0.0f, 1.0f) * 255.0f);
                    row.at<uchar>(0, static_cast<int>(bin_count) + i) =
                        static_cast<uchar>(std::clamp((star[i] - min_v) / denom, 0.0f, 1.0f) * 255.0f);
                }
                if (state.sss_waterfall.empty()) {
                    state.sss_waterfall = row.clone();
                } else {
                    cv::vconcat(state.sss_waterfall, row, state.sss_waterfall);
                }
                state.sss_row_count = state.sss_waterfall.rows;
                state.has_sss = true;
            }
        }
    }

    for (const QString& file_path : esl3d_paths) {
        const QString cache_dir = replayCacheDirForSonarFile(file_path);
        QDir().mkpath(cache_dir);
        DirReplayState& state = dir_states[cache_dir];
        state.cache_dir = cache_dir;
        state.esl3d_source = file_path;

        const QString range_cached = rangeCachePath(cache_dir, file_path);
        const QString intensity_cached = intensityCachePath(cache_dir, file_path);
        const bool range_ready = cacheFileReady(range_cached);
        const bool intensity_ready = cacheFileReady(intensity_cached);
        if (range_ready && intensity_ready) {
            state.range_mp4_path = range_cached;
            state.intensity_mp4_path = intensity_cached;
            state.has_3d = true;
            any_used_cache = true;
            continue;
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
            if (!state.range_writer.isOpened()) {
                state.range_mp4_path = range_cached;
                state.intensity_mp4_path = intensity_cached;
                state.range_writer.open(
                    state.range_mp4_path.toStdString(),
                    cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                    20.0,
                    range_img.size(),
                    true);
                state.intensity_writer.open(
                    state.intensity_mp4_path.toStdString(),
                    cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
                    20.0,
                    inten_img.size(),
                    true);
            }
            state.range_writer.write(range_img);
            state.intensity_writer.write(inten_img);
            state.has_3d = true;
            out_result.timeline_us.push_back(static_cast<qint64>(ts_us));
        }
    }

    for (auto it = dir_states.begin(); it != dir_states.end(); ++it) {
        DirReplayState& state = it.value();
        state.fls_writer.release();
        state.range_writer.release();
        state.intensity_writer.release();
        if (state.has_sss && !state.sss_waterfall.empty() && !cacheFileReady(state.sss_png_path)) {
            cv::Mat colored;
            cv::applyColorMap(state.sss_waterfall, colored, cv::COLORMAP_BONE);
            state.sss_png_path = sssCachePath(state.cache_dir, state.esl2d_source);
            cv::imwrite(state.sss_png_path.toStdString(), colored);
        }
        mergeDirState(state, out_result);
    }

    if (out_result.has_3d && !out_result.esl3d_source_path.isEmpty()) {
        out_result.esl3d_frame_count =
            indexEsl3dFrameOffsets(out_result.esl3d_source_path, out_result.esl3d_frame_offsets);
        if (out_result.esl3d_frame_count <= 0 && !out_result.intensity_mp4_path.isEmpty()) {
            out_result.esl3d_frame_count = videoFrameCount(out_result.intensity_mp4_path);
        }
    }

    std::sort(out_result.timeline_us.begin(), out_result.timeline_us.end());
    out_result.timeline_us.erase(
        std::unique(out_result.timeline_us.begin(), out_result.timeline_us.end()),
        out_result.timeline_us.end());
    out_result.used_cache = any_used_cache;

    const QString timeline_dir =
        out_result.output_dir.isEmpty() ? QFileInfo(summary_path).absolutePath() : out_result.output_dir;
    if (!any_used_cache) {
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

    return out_result.has_fls || out_result.has_sss || out_result.has_3d;
}

} // namespace replay_mode
