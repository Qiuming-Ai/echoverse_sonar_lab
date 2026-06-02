#pragma once

#include <QByteArray>
#include <QHostAddress>
#include <QPointer>
#include <QTcpServer>
#include <QTcpSocket>

#include <cstdint>
#include <map>
#include <memory>
#include <string>

namespace standalone_mvp {

class SonarTcpHub {
public:
    void ensureListening(std::uint16_t port, const std::string& host);
    bool send(std::uint16_t port, const QByteArray& data);
    void shutdownAll();

private:
    struct PortEntry {
        std::unique_ptr<QTcpServer> server;
        QPointer<QTcpSocket> client;
        QHostAddress bind_address;
        int consecutive_write_failures = 0;
    };

    void dropClient(PortEntry& entry);
    bool writeAll(PortEntry& entry, const QByteArray& data);
    void refreshConnections(PortEntry& entry);

    std::map<std::uint16_t, PortEntry> ports_;
};

} // namespace standalone_mvp
