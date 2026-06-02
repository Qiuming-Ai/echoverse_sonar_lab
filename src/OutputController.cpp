#include "OutputController.hpp"

#include "SonarOutputUtil.hpp"

#include <algorithm>
#include <set>

namespace standalone_mvp {

void OutputController::startSession(const QString& project_dir, const std::vector<SonarModuleConfig>& modules) {
    if (running_) {
        stopSession();
    }
    session_root_ = buildOutputSessionRoot(project_dir);
    std::set<std::uint16_t> ports;
    for (const auto& mod : modules) {
        if (!mod.enabled) {
            continue;
        }
        switch (mod.type) {
        case SonarModuleType::FLS:
            if (mod.fls_config.tcp_output_enabled) {
                ports.insert(static_cast<std::uint16_t>(std::clamp(mod.fls_config.tcp_port, 1, 65535)));
            }
            if (mod.point_cloud_config.enabled && mod.point_cloud_config.tcp_output_enabled) {
                ports.insert(static_cast<std::uint16_t>(std::clamp(mod.point_cloud_config.tcp_port, 1, 65535)));
            }
            break;
        case SonarModuleType::MBES:
            if (mod.mbes_config.tcp_output_enabled) {
                ports.insert(static_cast<std::uint16_t>(std::clamp(mod.mbes_config.tcp_port, 1, 65535)));
            }
            if (mod.point_cloud_config.enabled && mod.point_cloud_config.tcp_output_enabled) {
                ports.insert(static_cast<std::uint16_t>(std::clamp(mod.point_cloud_config.tcp_port, 1, 65535)));
            }
            break;
        case SonarModuleType::SSS:
            if (mod.sss_config.tcp_output_enabled) {
                ports.insert(static_cast<std::uint16_t>(std::clamp(mod.sss_config.tcp_port, 1, 65535)));
            }
            break;
        }
    }
    for (const std::uint16_t port : ports) {
        tcp_hub_.ensureListening(port, "0.0.0.0");
    }
    running_ = true;
}

void OutputController::stopSession() {
    tcp_hub_.shutdownAll();
    running_ = false;
    session_root_.clear();
}

} // namespace standalone_mvp
