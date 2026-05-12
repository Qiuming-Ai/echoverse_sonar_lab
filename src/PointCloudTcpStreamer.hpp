#pragma once

#include "PointCloudSonarSimulation.hpp"

#include <QHostAddress>
#include <QPointer>
#include <QFile>
#include <QTcpServer>
#include <QTcpSocket>

#include <cstddef>
#include <cstdint>
#include <memory>

namespace standalone_mvp {

struct PointCloudTcpRuntimeStatus {
    bool running = false;
    bool client_connected = false;
    std::uint64_t last_sent_seq = 0;
    std::size_t last_payload_bytes = 0;
};

class PointCloudTcpStreamer {
public:
    PointCloudTcpStreamer();
    ~PointCloudTcpStreamer();

    void applyConfig(bool enabled, const std::string& host, std::uint16_t port, bool file_output_enabled, const std::string& file_output_path);
    void stop();
    void sendFrame(const PointCloudFrame& frame);

    PointCloudTcpRuntimeStatus status() const;

private:
    void refreshConnection();
    bool writeAll(const QByteArray& data);
    void refreshFileOutput();
    QByteArray buildPacket(const PointCloudFrame& frame, std::uint64_t seq) const;
    void dropClient();

    bool enabled_ = false;
    bool file_output_enabled_ = false;
    std::string host_ = "0.0.0.0";
    std::string file_output_path_;
    std::uint16_t port_ = 30001;
    std::uint64_t seq_ = 0;
    int consecutive_write_failures_ = 0;

    QHostAddress bind_address_;
    std::unique_ptr<QTcpServer> server_;
    std::unique_ptr<QFile> file_output_;
    QPointer<QTcpSocket> client_;
    PointCloudTcpRuntimeStatus status_;
};

} // namespace standalone_mvp
