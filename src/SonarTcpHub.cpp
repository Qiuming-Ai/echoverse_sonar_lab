#include "SonarTcpHub.hpp"

#include <QCoreApplication>

#include <algorithm>
#include <iostream>

namespace standalone_mvp {
namespace {

constexpr int kWaitBytesWrittenTimeoutMs = 80;
constexpr int kMaxConsecutiveWriteFailures = 5;

} // namespace

void SonarTcpHub::dropClient(PortEntry& entry) {
    if (!entry.client) {
        return;
    }
    QTcpSocket* sock = entry.client.data();
    entry.client.clear();
    if (!sock) {
        entry.consecutive_write_failures = 0;
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
    entry.consecutive_write_failures = 0;
}

void SonarTcpHub::refreshConnections(PortEntry& entry) {
    if (!entry.server || !entry.server->isListening()) {
        return;
    }
    while (entry.server->hasPendingConnections()) {
        QTcpSocket* next = entry.server->nextPendingConnection();
        if (!next) {
            continue;
        }
        dropClient(entry);
        entry.client = next;
        entry.client->setSocketOption(QAbstractSocket::LowDelayOption, 1);
    }
    if (entry.client && entry.client->state() != QAbstractSocket::ConnectedState) {
        dropClient(entry);
    }
}

bool SonarTcpHub::writeAll(PortEntry& entry, const QByteArray& data) {
    if (!entry.client || entry.client->state() != QAbstractSocket::ConnectedState) {
        return false;
    }
    const qint64 written = entry.client->write(data);
    if (written < 0 || written != data.size()) {
        ++entry.consecutive_write_failures;
        return false;
    }
    if (!entry.client->waitForBytesWritten(kWaitBytesWrittenTimeoutMs)) {
        ++entry.consecutive_write_failures;
        return false;
    }
    entry.consecutive_write_failures = 0;
    return true;
}

void SonarTcpHub::ensureListening(const std::uint16_t port, const std::string& host) {
    const std::uint16_t clamped_port = static_cast<std::uint16_t>(std::clamp<int>(port, 1, 65535));
    const std::string bind_host = host.empty() ? std::string("0.0.0.0") : host;
    PortEntry& entry = ports_[clamped_port];
    QHostAddress bind_address(QString::fromStdString(bind_host));
    if (bind_address.isNull()) {
        bind_address = QHostAddress::AnyIPv4;
    }
    if (entry.server && entry.server->isListening() &&
        entry.server->serverPort() == static_cast<int>(clamped_port) &&
        entry.bind_address == bind_address) {
        return;
    }
    dropClient(entry);
    if (entry.server) {
        entry.server->close();
    } else {
        entry.server = std::make_unique<QTcpServer>();
    }
    entry.bind_address = bind_address;
    if (!entry.server->listen(bind_address, clamped_port)) {
        std::cout << "[tcp_hub] listen failed port=" << clamped_port << std::endl;
    }
}

bool SonarTcpHub::send(const std::uint16_t port, const QByteArray& data) {
    const auto it = ports_.find(port);
    if (it == ports_.end()) {
        return false;
    }
    PortEntry& entry = it->second;
    refreshConnections(entry);
    const bool ok = writeAll(entry, data);
    if (!ok && entry.consecutive_write_failures >= kMaxConsecutiveWriteFailures) {
        dropClient(entry);
    }
    return ok;
}

void SonarTcpHub::shutdownAll() {
    for (auto& [port, entry] : ports_) {
        (void)port;
        dropClient(entry);
        if (entry.server) {
            entry.server->close();
        }
    }
    ports_.clear();
}

} // namespace standalone_mvp
