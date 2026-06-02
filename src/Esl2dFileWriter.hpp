#pragma once

#include <sonar_types_v2/echoverse_sonar_types.hpp>

#include <QFile>
#include <QHostAddress>
#include <QJsonObject>
#include <QPointer>
#include <QTcpServer>
#include <QTcpSocket>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace standalone_mvp {

enum class Esl2dSonarKind : std::uint16_t {
    FLS = 0,
    SSS = 1,
};

struct Esl2dPoseSnapshot {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw_deg = 0.0;
    double pitch_deg = 0.0;
    double quat_w = 1.0;
    double quat_x = 0.0;
    double quat_y = 0.0;
    double quat_z = 0.0;
};

struct Esl2dWriteParams {
    Esl2dSonarKind sonar_kind = Esl2dSonarKind::FLS;
    std::uint64_t timestamp_us = 0;
    float max_range_m = 0.0f;
    Esl2dPoseSnapshot pose;
    QJsonObject sonar_config;
    QJsonObject environment;
    QString sonar_module_name;
};

class SonarTcpHub;

class Esl2dFileWriter {
public:
    Esl2dFileWriter();
    ~Esl2dFileWriter();

    void setTcpHub(SonarTcpHub* hub);
    void applyConfig(bool tcp_output_enabled,
                     const std::string& tcp_host,
                     std::uint16_t tcp_port,
                     bool file_output_enabled,
                     const std::string& file_output_path);
    void close();
    bool outputEnabled() const { return session_active_ && (tcp_output_enabled_ || file_output_enabled_); }
    void setSessionActive(bool active) { session_active_ = active; }
    std::uint64_t framesWritten() const { return seq_; }
    void resetFrameCount() { seq_ = 0; }

    bool writeFlsFrame(const sonar_types_v2::samples::Sonar& sample, const Esl2dWriteParams& params);
    bool writeSssFrame(const sonar_types_v2::samples::Sonar& starboard,
                       const sonar_types_v2::samples::Sonar& port,
                       const Esl2dWriteParams& params);

private:
    struct BeamSlice {
        float bearing_deg = 0.0f;
        std::vector<float> bins;
        QString side_label;
    };

    bool writePacket(Esl2dSonarKind sonar_kind,
                     std::uint32_t beam_count,
                     std::uint32_t bin_count,
                     float max_range_m,
                     const std::vector<BeamSlice>& beams,
                     const Esl2dWriteParams& params);
    void refreshFileOutput();
    void refreshTcpServer();
    void dropTcpClient();
    bool writeTcpAll(const QByteArray& data);
    static bool extractBeamBins(const sonar_types_v2::samples::Sonar& sample,
                                std::uint32_t beam_index,
                                std::vector<float>& out_bins);
    static std::vector<BeamSlice> beamsFromFlsSample(const sonar_types_v2::samples::Sonar& sample);
    static std::vector<BeamSlice> beamsFromSssSamples(const sonar_types_v2::samples::Sonar& starboard,
                                                        const sonar_types_v2::samples::Sonar& port);

    bool tcp_output_enabled_ = false;
    bool file_output_enabled_ = false;
    bool session_active_ = false;
    SonarTcpHub* tcp_hub_ = nullptr;
    std::string tcp_host_ = "0.0.0.0";
    std::uint16_t tcp_port_ = 30001;
    std::string file_output_path_;
    std::uint64_t seq_ = 0;
    int consecutive_tcp_write_failures_ = 0;

    std::unique_ptr<QFile> file_output_;
    std::unique_ptr<QTcpServer> tcp_server_;
    QPointer<QTcpSocket> tcp_client_;
};

QString buildEsl2dOutputPath(const QString& project_dir, const QString& sonar_name, const QString& kind_suffix);

} // namespace standalone_mvp
