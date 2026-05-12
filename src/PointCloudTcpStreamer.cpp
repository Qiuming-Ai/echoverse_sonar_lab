#include "PointCloudTcpStreamer.hpp"

#include <QByteArray>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QJsonDocument>
#include <QJsonObject>
#include <QtEndian>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>

namespace standalone_mvp {
namespace {

constexpr std::uint32_t kMagic = 0x5033534Eu; // "NS3P"
constexpr std::uint16_t kVersion = 1u;
constexpr std::uint16_t kHeaderBytes = 56u;
constexpr double kRad2Deg = 57.2957795130823208768;
constexpr int kWaitBytesWrittenTimeoutMs = 80;
constexpr int kMaxConsecutiveWriteFailuresBeforeDisconnect = 5;

template <typename T>
void appendLe(QByteArray& out, T value) {
    const T little = qToLittleEndian(value);
    out.append(reinterpret_cast<const char*>(&little), static_cast<int>(sizeof(T)));
}

QByteArray floatsToLeBytes(const std::vector<float>& values) {
    QByteArray out;
    out.reserve(static_cast<int>(values.size() * sizeof(float)));
    for (float v : values) {
        std::uint32_t raw = 0;
        static_assert(sizeof(float) == sizeof(std::uint32_t), "float size mismatch");
        std::memcpy(&raw, &v, sizeof(raw));
        appendLe(out, raw);
    }
    return out;
}

double computeYawDeg(const Eigen::Vector3d& forward) {
    return std::atan2(forward.y(), forward.x()) * kRad2Deg;
}

double computePitchDeg(const Eigen::Vector3d& forward) {
    return std::asin(std::clamp(-forward.z(), -1.0, 1.0)) * kRad2Deg;
}

} // namespace

PointCloudTcpStreamer::PointCloudTcpStreamer()
    : server_(std::make_unique<QTcpServer>()) {
}

PointCloudTcpStreamer::~PointCloudTcpStreamer() {
    stop();
}

void PointCloudTcpStreamer::dropClient() {
    if (!client_) {
        return;
    }
    QTcpSocket* sock = client_.data();
    client_.clear();
    if (!sock) {
        status_.client_connected = false;
        consecutive_write_failures_ = 0;
        return;
    }
    sock->disconnectFromHost();
    sock->abort();
    sock->setParent(nullptr);
    if (QCoreApplication::instance()) {
        sock->deleteLater();
    } else {
        delete sock;
    }
    status_.client_connected = false;
    consecutive_write_failures_ = 0;
}

void PointCloudTcpStreamer::applyConfig(
    bool enabled,
    const std::string& host,
    std::uint16_t port,
    bool file_output_enabled,
    const std::string& file_output_path) {
    const std::uint16_t clamped_port = static_cast<std::uint16_t>(std::clamp<int>(port, 1, 65535));
    const std::string next_host = host.empty() ? std::string("0.0.0.0") : host;
    if (enabled_ == enabled &&
        host_ == next_host &&
        port_ == clamped_port &&
        file_output_enabled_ == file_output_enabled &&
        file_output_path_ == file_output_path) {
        return;
    }
    enabled_ = enabled;
    file_output_enabled_ = file_output_enabled;
    file_output_path_ = file_output_path;
    host_ = next_host;
    port_ = clamped_port;
    if (!enabled_ && !file_output_enabled_) {
        std::cout << "[tcp_dbg] applyConfig disable -> stop" << std::endl;
        stop();
        return;
    }
    if (!enabled_) {
        dropClient();
        if (server_) {
            server_->close();
        }
        status_.running = false;
        status_.client_connected = false;
        refreshFileOutput();
        return;
    }

    stop();
    bind_address_ = QHostAddress(QString::fromStdString(host_));
    if (bind_address_.isNull()) {
        bind_address_ = QHostAddress::AnyIPv4;
    }
    if (!server_) {
        server_ = std::make_unique<QTcpServer>();
    }
    const bool ok = server_->listen(bind_address_, port_);
    status_.running = ok;
    std::cout << "[tcp_dbg] listen host=" << host_ << " port=" << port_
              << " ok=" << (ok ? 1 : 0) << std::endl;
    refreshFileOutput();
}

void PointCloudTcpStreamer::stop() {
    dropClient();
    if (file_output_) {
        file_output_->close();
        file_output_.reset();
    }
    if (server_) {
        server_->close();
    }
    status_.running = false;
    status_.client_connected = false;
    consecutive_write_failures_ = 0;
}

void PointCloudTcpStreamer::refreshFileOutput() {
    if (!file_output_enabled_ || file_output_path_.empty()) {
        if (file_output_) {
            file_output_->close();
            file_output_.reset();
        }
        return;
    }
    const QString path = QString::fromStdString(file_output_path_);
    const QFileInfo fi(path);
    if (!fi.dir().exists()) {
        fi.dir().mkpath(".");
    }
    if (!file_output_ || file_output_->fileName() != path) {
        if (file_output_) {
            file_output_->close();
        }
        file_output_ = std::make_unique<QFile>(path);
    }
    if (!file_output_->isOpen()) {
        if (!file_output_->open(QIODevice::WriteOnly | QIODevice::Append)) {
            std::cout << "[tcp_dbg] file output open failed path=" << path.toStdString() << std::endl;
            file_output_.reset();
        }
    }
}

void PointCloudTcpStreamer::refreshConnection() {
    if (!server_ || !server_->isListening()) {
        status_.running = false;
        status_.client_connected = false;
        return;
    }
    status_.running = true;

    while (server_->hasPendingConnections()) {
        QTcpSocket* next = server_->nextPendingConnection();
        if (!next) {
            continue;
        }
        if (client_) {
            dropClient();
        }
        client_ = next;
        client_->setSocketOption(QAbstractSocket::LowDelayOption, 1);
        consecutive_write_failures_ = 0;
        std::cout << "[tcp_dbg] client connected peer="
                  << client_->peerAddress().toString().toStdString()
                  << ":" << client_->peerPort() << std::endl;
    }

    if (client_ && client_->state() != QAbstractSocket::ConnectedState) {
        std::cout << "[tcp_dbg] client dropped state=" << static_cast<int>(client_->state()) << std::endl;
        dropClient();
    }
    status_.client_connected = static_cast<bool>(client_);
}

bool PointCloudTcpStreamer::writeAll(const QByteArray& data) {
    if (!client_) {
        return false;
    }
    const qint64 written = client_->write(data);
    if (written < 0) {
        std::cout << "[tcp_dbg] write failed error=" << client_->errorString().toStdString() << std::endl;
        ++consecutive_write_failures_;
        return false;
    }
    if (written != data.size()) {
        std::cout << "[tcp_dbg] partial queued=" << written
                  << " expected=" << data.size()
                  << " pending=" << client_->bytesToWrite() << std::endl;
    }
    if (!client_->waitForBytesWritten(kWaitBytesWrittenTimeoutMs)) {
        std::cout << "[tcp_dbg] waitForBytesWritten timeout/error: "
                  << client_->errorString().toStdString()
                  << " pending=" << client_->bytesToWrite() << std::endl;
        ++consecutive_write_failures_;
        return false;
    }
    consecutive_write_failures_ = 0;
    return true;
}

QByteArray PointCloudTcpStreamer::buildPacket(const PointCloudFrame& frame, std::uint64_t seq) const {
    const std::uint32_t width = static_cast<std::uint32_t>(frame.polar_frame.width);
    const std::uint32_t height = static_cast<std::uint32_t>(frame.polar_frame.height);
    const std::uint32_t point_count = static_cast<std::uint32_t>(frame.polar_frame.point_count);
    const Eigen::Vector3d forward = frame.pose_forward_world.norm() > 1e-9
                                        ? frame.pose_forward_world.normalized()
                                        : Eigen::Vector3d::UnitX();

    QJsonObject metadata;
    metadata["byte_order"] = "little_endian";
    metadata["layout"] = "row_major";
    metadata["data_order"] = "range_then_intensity";
    metadata["range_invalid_value"] = frame.polar_frame.invalid_value;
    metadata["intensity_invalid_value"] = frame.polar_frame.invalid_value;

    QJsonObject pose;
    pose["x"] = frame.pose_position_world.x();
    pose["y"] = frame.pose_position_world.y();
    pose["z"] = frame.pose_position_world.z();
    pose["yaw_deg"] = computeYawDeg(forward);
    pose["pitch_deg"] = computePitchDeg(forward);
    pose["quat_w"] = Eigen::Quaterniond(frame.pose_rotation_world).w();
    pose["quat_x"] = Eigen::Quaterniond(frame.pose_rotation_world).x();
    pose["quat_y"] = Eigen::Quaterniond(frame.pose_rotation_world).y();
    pose["quat_z"] = Eigen::Quaterniond(frame.pose_rotation_world).z();
    metadata["pose"] = pose;

    const PointCloudSonarConfig& c = frame.config;
    QJsonObject sonar_cfg;
    sonar_cfg["enabled"] = c.enabled;
    sonar_cfg["range_m"] = c.range_m;
    sonar_cfg["frequency_khz"] = c.frequency_khz;
    sonar_cfg["bandwidth_khz"] = c.bandwidth_khz;
    sonar_cfg["horizontal_angle_resolution_deg"] = c.horizontal_angle_resolution_deg;
    sonar_cfg["vertical_angle_resolution_deg"] = c.vertical_angle_resolution_deg;
    sonar_cfg["horizontal_fov_deg"] = c.horizontal_fov_deg;
    sonar_cfg["vertical_fov_deg"] = c.vertical_fov_deg;
    sonar_cfg["point_count_formula"] =
        "point_count = floor(horizontal_fov_deg/horizontal_angle_resolution_deg) * floor(vertical_fov_deg/vertical_angle_resolution_deg)";
    metadata["sonar_config"] = sonar_cfg;

    QJsonObject env;
    env["enable_attenuation"] = c.enable_attenuation;
    env["attenuation_frequency_khz"] = c.attenuation_frequency_khz;
    env["temperature_c"] = c.temperature_c;
    env["salinity_ppt"] = c.salinity_ppt;
    env["acidity_ph"] = c.acidity_ph;
    env["depth_m"] = c.depth_m;
    env["enable_reverb"] = c.enable_reverb;
    env["enable_speckle"] = c.enable_speckle;
    env["sound_speed_mps"] = c.sound_speed_mps;
    metadata["environment"] = env;

    QJsonObject frame_info;
    frame_info["seq"] = static_cast<qint64>(seq);
    frame_info["timestamp_us"] = static_cast<qint64>(frame.timestamp_us);
    frame_info["width"] = static_cast<int>(width);
    frame_info["height"] = static_cast<int>(height);
    frame_info["point_count"] = static_cast<int>(point_count);
    frame_info["rounding_policy"] = "floor";
    metadata["frame"] = frame_info;

    const QByteArray metadata_bytes = QJsonDocument(metadata).toJson(QJsonDocument::Compact);
    const QByteArray range_bytes = floatsToLeBytes(frame.polar_frame.range_image_m);
    const QByteArray intensity_bytes = floatsToLeBytes(frame.polar_frame.intensity_image);

    const std::uint32_t metadata_size = static_cast<std::uint32_t>(metadata_bytes.size());
    const std::uint32_t range_size = static_cast<std::uint32_t>(range_bytes.size());
    const std::uint32_t intensity_size = static_cast<std::uint32_t>(intensity_bytes.size());
    const std::uint32_t payload_size = metadata_size + range_size + intensity_size;

    QByteArray packet;
    packet.reserve(static_cast<int>(kHeaderBytes + payload_size));
    appendLe(packet, kMagic);
    appendLe(packet, kVersion);
    appendLe(packet, kHeaderBytes);
    appendLe(packet, seq);
    appendLe(packet, frame.timestamp_us);
    appendLe(packet, width);
    appendLe(packet, height);
    appendLe(packet, point_count);
    appendLe(packet, metadata_size);
    appendLe(packet, range_size);
    appendLe(packet, intensity_size);
    appendLe(packet, payload_size);
    appendLe(packet, static_cast<std::uint32_t>(0u)); // reserved
    packet.append(metadata_bytes);
    packet.append(range_bytes);
    packet.append(intensity_bytes);
    return packet;
}

void PointCloudTcpStreamer::sendFrame(const PointCloudFrame& frame) {
    refreshConnection();
    const std::uint64_t seq = ++seq_;
    const QByteArray packet = buildPacket(frame, seq);
    if (file_output_) {
        const qint64 n = file_output_->write(packet);
        if (n != packet.size()) {
            std::cout << "[tcp_dbg] file output write failed path=" << file_output_->fileName().toStdString() << std::endl;
            file_output_->close();
            file_output_.reset();
        } else {
            file_output_->flush();
        }
    }
    if (!enabled_ || !status_.running || !status_.client_connected) {
        return;
    }
    if (!writeAll(packet)) {
        std::cout << "[tcp_dbg] sendFrame failed seq=" << seq
                  << " consecutive_failures=" << consecutive_write_failures_ << std::endl;
        if (consecutive_write_failures_ >= kMaxConsecutiveWriteFailuresBeforeDisconnect) {
            std::cout << "[tcp_dbg] drop client after consecutive write failures" << std::endl;
            dropClient();
        }
        return;
    }
    status_.last_sent_seq = seq;
    status_.last_payload_bytes = static_cast<std::size_t>(packet.size());
    if ((seq % 20u) == 0u) {
        std::cout << "[tcp_dbg] sendFrame ok seq=" << seq
                  << " bytes=" << packet.size()
                  << " w=" << frame.polar_frame.width
                  << " h=" << frame.polar_frame.height << std::endl;
    }
}

PointCloudTcpRuntimeStatus PointCloudTcpStreamer::status() const {
    return status_;
}

} // namespace standalone_mvp
