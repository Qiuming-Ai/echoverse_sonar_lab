#pragma once

#include "AppConfig.hpp"
#include "SonarTcpHub.hpp"

#include <QString>
#include <vector>

namespace standalone_mvp {

class OutputController {
public:
    bool isRunning() const { return running_; }
    const QString& sessionRoot() const { return session_root_; }

    void startSession(const QString& project_dir, const std::vector<SonarModuleConfig>& modules);
    void stopSession();
    SonarTcpHub& tcpHub() { return tcp_hub_; }

private:
    bool running_ = false;
    QString session_root_;
    SonarTcpHub tcp_hub_;
};

} // namespace standalone_mvp
