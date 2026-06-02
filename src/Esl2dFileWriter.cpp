#include "Esl2dFileWriter.hpp"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QRegularExpression>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>

namespace standalone_mvp {
namespace {

constexpr std::uint32_t kMagic = 0x5032534Eu;
constexpr std::uint16_t kVersion = 1u;
constexpr std::uint16_t kHeaderBytes = 64u;
constexpr int kWaitBytesWrittenTimeoutMs = 80;
constexpr int kMaxConsecutiveTcpWriteFailures = 5;

template <typename T>
void appendLe(QByteArray& out, T value) {
    const char* p = reinterpret_cast<const char*>(&value);
    out.append(p, static_cast<int>(sizeof(T)));
}

QByteArray floatsToLeBytes(const std::vector<float>& values) {
    QByteArray out;
    out.resize(static_cast<int>(values.size() * sizeof(float)));
    if (!values.empty()) {
        std::memcpy(out.data(), values.data(), values.size() * sizeof(float));
    }
    return out;
}

QJsonObject makeFrameInfo(std::uint64_t seq,
                          std::uint64_t timestamp_us,
                          Esl2dSonarKind sonar_kind,
                          std::uint32_t beam_count,
                          std::uint32_t bin_count,
                          float max_range_m) {
    QJsonObject frame;
    frame["seq"] = static_cast<qint64>(seq);
    frame["timestamp_us"] = static_cast<qint64>(timestamp_us);
    frame["sonar_type"] = static_cast<int>(sonar_kind);
    frame["beam_count"] = static_cast<int>(beam_count);
    frame["bin_count"] = static_cast<int>(bin_count);
    frame["max_range_m"] = static_cast<double>(max_range_m);
    return frame;
}

} // namespace

QString buildEsl2dOutputPath(const QString& project_dir, const QString& sonar_name, const QString& kind_suffix) {
    const QString safe_name = QString(sonar_name).replace(QRegularExpression(R"([\\/:*?"<>|])"), "_");
    const QString date = QDateTime::currentDateTime().toString("yyyyMMdd_HHmm");
    const QString out_dir = QDir(project_dir).filePath("Sonar 2D");
    QDir().mkpath(out_dir);
    return QDir(out_dir).filePath(QString("%1_%2_%3.esl2d").arg(safe_name, kind_suffix, date));
}

Esl2dFileWriter::Esl2dFileWriter() : tcp_server_(std::make_unique<QTcpServer>()) {}

Esl2dFileWriter::~Esl2dFileWriter() {
    close();
}

void Esl2dFileWriter::applyConfig(const bool tcp_output_enabled,
                                  const std::string& tcp_host,
                                  const std::uint16_t tcp_port,
                                  const bool file_output_enabled,
                                  const std::string& file_output_path) {
    tcp_output_enabled_ = tcp_output_enabled;
    tcp_host_ = tcp_host;
    tcp_port_ = tcp_port;
    file_output_enabled_ = file_output_enabled;
    file_output_path_ = file_output_path;
    refreshFileOutput();
    refreshTcpServer();
}

void Esl2dFileWriter::close() {
    dropTcpClient();
    if (tcp_server_) {
        tcp_server_->close();
    }
    if (file_output_) {
        file_output_->close();
        file_output_.reset();
    }
    seq_ = 0;
    consecutive_tcp_write_failures_ = 0;
}

void Esl2dFileWriter::dropTcpClient() {
    if (!tcp_client_) {
        return;
    }
    QTcpSocket* sock = tcp_client_.data();
    tcp_client_.clear();
    if (!sock) {
        consecutive_tcp_write_failures_ = 0;
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
    consecutive_tcp_write_failures_ = 0;
}

void Esl2dFileWriter::refreshTcpServer() {
    if (!tcp_output_enabled_) {
        dropTcpClient();
        if (tcp_server_) {
            tcp_server_->close();
        }
        return;
    }
    if (!tcp_server_) {
        tcp_server_ = std::make_unique<QTcpServer>();
    }
    QHostAddress bind_address(QString::fromStdString(tcp_host_));
    if (bind_address.isNull()) {
        bind_address = QHostAddress::AnyIPv4;
    }
    if (tcp_server_->isListening()) {
        if (tcp_server_->serverAddress() == bind_address && tcp_server_->serverPort() == tcp_port_) {
            return;
        }
        dropTcpClient();
        tcp_server_->close();
    }
    if (!tcp_server_->listen(bind_address, tcp_port_)) {
        std::cout << "[esl2d] tcp listen failed host=" << tcp_host_ << " port=" << tcp_port_ << std::endl;
    }
}

bool Esl2dFileWriter::writeTcpAll(const QByteArray& data) {
    if (!tcp_output_enabled_ || !tcp_server_ || !tcp_server_->isListening()) {
        return false;
    }
    while (tcp_server_->hasPendingConnections()) {
        QTcpSocket* next = tcp_server_->nextPendingConnection();
        if (!next) {
            continue;
        }
        dropTcpClient();
        tcp_client_ = next;
        tcp_client_->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    }
    if (!tcp_client_ || tcp_client_->state() != QAbstractSocket::ConnectedState) {
        return false;
    }
    const qint64 written = tcp_client_->write(data);
    if (written != data.size()) {
        ++consecutive_tcp_write_failures_;
        if (consecutive_tcp_write_failures_ >= kMaxConsecutiveTcpWriteFailures) {
            dropTcpClient();
        }
        return false;
    }
    if (!tcp_client_->waitForBytesWritten(kWaitBytesWrittenTimeoutMs)) {
        ++consecutive_tcp_write_failures_;
        if (consecutive_tcp_write_failures_ >= kMaxConsecutiveTcpWriteFailures) {
            dropTcpClient();
        }
        return false;
    }
    consecutive_tcp_write_failures_ = 0;
    return true;
}

void Esl2dFileWriter::refreshFileOutput() {
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
            std::cout << "[esl2d] file output open failed path=" << path.toStdString() << std::endl;
            file_output_.reset();
        }
    }
}

bool Esl2dFileWriter::extractBeamBins(const sonar_types_v2::samples::Sonar& sample,
                                      const std::uint32_t beam_index,
                                      std::vector<float>& out_bins) {
    if (!sample.beam_count || !sample.bin_count || beam_index >= sample.beam_count) {
        return false;
    }
    const std::size_t need = static_cast<std::size_t>(sample.beam_count) * static_cast<std::size_t>(sample.bin_count);
    if (sample.bins.size() < need) {
        return false;
    }
    out_bins.resize(sample.bin_count);
    const float* src = sample.bins.data() + static_cast<std::size_t>(beam_index) * sample.bin_count;
    std::memcpy(out_bins.data(), src, static_cast<std::size_t>(sample.bin_count) * sizeof(float));
    return true;
}

std::vector<Esl2dFileWriter::BeamSlice> Esl2dFileWriter::beamsFromFlsSample(const sonar_types_v2::samples::Sonar& sample) {
    std::vector<BeamSlice> beams;
    if (!sample.beam_count || !sample.bin_count) {
        return beams;
    }
    beams.resize(sample.beam_count);
    for (std::uint32_t beam = 0; beam < sample.beam_count; ++beam) {
        BeamSlice& slice = beams[beam];
        if (beam < sample.bearings.size()) {
            slice.bearing_deg = static_cast<float>(sample.bearings[beam].getDeg());
        } else if (sample.beam_count > 0) {
            const double half = sample.beam_width.getDeg() * 0.5;
            const double t = static_cast<double>(beam) / static_cast<double>(std::max(1u, sample.beam_count - 1));
            slice.bearing_deg = static_cast<float>(-half + t * (2.0 * half));
        }
        if (!extractBeamBins(sample, beam, slice.bins)) {
            beams.clear();
            break;
        }
    }
    return beams;
}

std::vector<Esl2dFileWriter::BeamSlice> Esl2dFileWriter::beamsFromSssSamples(
    const sonar_types_v2::samples::Sonar& starboard,
    const sonar_types_v2::samples::Sonar& port) {
    std::vector<BeamSlice> beams(2);
    beams[0].bearing_deg = 0.0f;
    beams[0].side_label = QStringLiteral("starboard");
    beams[1].bearing_deg = 180.0f;
    beams[1].side_label = QStringLiteral("port");

    if (!extractBeamBins(starboard, 0, beams[0].bins)) {
        return {};
    }
    if (!extractBeamBins(port, 0, beams[1].bins)) {
        return {};
    }
    if (beams[0].bins.size() != beams[1].bins.size()) {
        return {};
    }
    return beams;
}

bool Esl2dFileWriter::writeFlsFrame(const sonar_types_v2::samples::Sonar& sample, const Esl2dWriteParams& params) {
    const std::vector<BeamSlice> beams = beamsFromFlsSample(sample);
    if (beams.empty()) {
        return false;
    }
    return writePacket(Esl2dSonarKind::FLS, sample.beam_count, sample.bin_count, params.max_range_m, beams, params);
}

bool Esl2dFileWriter::writeSssFrame(const sonar_types_v2::samples::Sonar& starboard,
                                    const sonar_types_v2::samples::Sonar& port,
                                    const Esl2dWriteParams& params) {
    const std::vector<BeamSlice> beams = beamsFromSssSamples(starboard, port);
    if (beams.size() != 2 || beams[0].bins.empty()) {
        return false;
    }
    return writePacket(Esl2dSonarKind::SSS,
                       2u,
                       static_cast<std::uint32_t>(beams[0].bins.size()),
                       params.max_range_m,
                       beams,
                       params);
}

bool Esl2dFileWriter::writePacket(const Esl2dSonarKind sonar_kind,
                                  const std::uint32_t beam_count,
                                  const std::uint32_t bin_count,
                                  const float max_range_m,
                                  const std::vector<BeamSlice>& beams,
                                  const Esl2dWriteParams& params) {
    if (!outputEnabled()) {
        return false;
    }
    if (beam_count == 0 || bin_count == 0 || beams.size() != beam_count || max_range_m <= 0.0f) {
        return false;
    }

    std::vector<float> beam_angles;
    beam_angles.reserve(beam_count);
    std::vector<float> intensity;
    intensity.reserve(static_cast<std::size_t>(beam_count) * bin_count);

    QJsonArray beams_meta;
    for (std::uint32_t i = 0; i < beam_count; ++i) {
        if (beams[i].bins.size() != bin_count) {
            return false;
        }
        beam_angles.push_back(beams[i].bearing_deg);
        intensity.insert(intensity.end(), beams[i].bins.begin(), beams[i].bins.end());

        QJsonObject beam_obj;
        beam_obj["index"] = static_cast<int>(i);
        beam_obj["bearing_deg"] = static_cast<double>(beams[i].bearing_deg);
        beam_obj["bin_count"] = static_cast<int>(bin_count);
        beam_obj["range_start_m"] = 0.0;
        beam_obj["range_end_m"] = static_cast<double>(max_range_m);
        if (!beams[i].side_label.isEmpty()) {
            beam_obj["side"] = beams[i].side_label;
        }
        beams_meta.append(beam_obj);
    }

    const std::uint64_t seq = ++seq_;
    const std::uint64_t timestamp_us =
        params.timestamp_us != 0 ? params.timestamp_us
                                 : static_cast<std::uint64_t>(QDateTime::currentMSecsSinceEpoch()) * 1000u;

    QJsonObject metadata;
    metadata["byte_order"] = "little_endian";
    metadata["layout"] = "beam_major";
    metadata["data_order"] = "metadata_then_beam_angles_then_intensity";
    metadata["sonar_kind"] = (sonar_kind == Esl2dSonarKind::FLS) ? "fls" : "sss";
    metadata["sonar_module_name"] = params.sonar_module_name;
    metadata["frame"] = makeFrameInfo(seq, timestamp_us, sonar_kind, beam_count, bin_count, max_range_m);
    metadata["beams"] = beams_meta;

    QJsonObject pose;
    pose["x"] = params.pose.x;
    pose["y"] = params.pose.y;
    pose["z"] = params.pose.z;
    pose["yaw_deg"] = params.pose.yaw_deg;
    pose["pitch_deg"] = params.pose.pitch_deg;
    pose["quat_w"] = params.pose.quat_w;
    pose["quat_x"] = params.pose.quat_x;
    pose["quat_y"] = params.pose.quat_y;
    pose["quat_z"] = params.pose.quat_z;
    metadata["pose"] = pose;
    if (!params.sonar_config.isEmpty()) {
        metadata["sonar_config"] = params.sonar_config;
    }
    if (!params.environment.isEmpty()) {
        metadata["environment"] = params.environment;
    }

    const QByteArray metadata_bytes = QJsonDocument(metadata).toJson(QJsonDocument::Compact);
    const QByteArray beam_angles_bytes = floatsToLeBytes(beam_angles);
    const QByteArray intensity_bytes = floatsToLeBytes(intensity);

    const std::uint32_t metadata_size = static_cast<std::uint32_t>(metadata_bytes.size());
    const std::uint32_t beam_angles_size = static_cast<std::uint32_t>(beam_angles_bytes.size());
    const std::uint32_t intensity_size = static_cast<std::uint32_t>(intensity_bytes.size());
    const std::uint32_t payload_size = metadata_size + beam_angles_size + intensity_size;

    QByteArray packet;
    packet.reserve(static_cast<int>(kHeaderBytes + payload_size));
    appendLe(packet, kMagic);
    appendLe(packet, kVersion);
    appendLe(packet, kHeaderBytes);
    appendLe(packet, seq);
    appendLe(packet, timestamp_us);
    appendLe(packet, static_cast<std::uint16_t>(sonar_kind));
    appendLe(packet, static_cast<std::uint16_t>(0u));
    appendLe(packet, beam_count);
    appendLe(packet, bin_count);
    appendLe(packet, max_range_m);
    appendLe(packet, metadata_size);
    appendLe(packet, beam_angles_size);
    appendLe(packet, intensity_size);
    appendLe(packet, payload_size);
    appendLe(packet, static_cast<std::uint32_t>(0u));
    appendLe(packet, static_cast<std::uint32_t>(0u));
    packet.append(metadata_bytes);
    packet.append(beam_angles_bytes);
    packet.append(intensity_bytes);

    bool ok = false;
    if (file_output_enabled_ && file_output_ && file_output_->isOpen()) {
        const qint64 n = file_output_->write(packet);
        if (n != packet.size()) {
            std::cout << "[esl2d] write failed path=" << file_output_->fileName().toStdString() << std::endl;
            file_output_->close();
            file_output_.reset();
        } else {
            file_output_->flush();
            ok = true;
        }
    }
    if (tcp_output_enabled_) {
        ok = writeTcpAll(packet) || ok;
    }
    return ok;
}

} // namespace standalone_mvp
