#include "RockSonarPlotView.hpp"
#include "SharedScene.hpp"
#include "MultibeamEngine.hpp"
#include "AppConfig.hpp"
#include "InfoPanelWidget.hpp"
#include "SettingsDialog.hpp"
#include "ui/DockWorkspace.hpp"
#include "PointCloudSonarSimulation.hpp"
#include "PointCloudTcpStreamer.hpp"
#include "PointCloudViewerWindow.hpp"
#include "CameraModule.hpp"
#include "FlsModule.hpp"
#include "MbesModule.hpp"
#include "SssModule.hpp"
#include "SceneEditorPanel.hpp"

#include <QApplication>
#include <QEventLoop>
#include <QEvent>
#include <QDialog>
#include <QDir>
#include <QFileInfo>
#include <QFrame>
#include <QHBoxLayout>
#include <QImage>
#include <QKeyEvent>
#include <QLabel>
#include <QMenu>
#include <QPixmap>
#include <QProcess>
#include <QPushButton>
#include <QObject>
#include <QString>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QWindow>
#include "SonarControlPanel.hpp"
#include "SideScanControlPanel.hpp"

#include <sonar_types_v2/echoverse_math_types.hpp>
#include <sonar_types_v2/echoverse_frame_types.hpp>
#include <sonar_types_v2/echoverse_sonar_types.hpp>
#include <Eigen/Geometry>
#include <sonar_core/AcousticRaySimulator.hpp>
#include <sonar_core/SignalProcessingUtils.hpp>
#include <osg/CopyOp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <osg/Camera>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Image>
#include <osg/LineWidth>
#include <osg/Matrixd>
#include <osg/MatrixTransform>
#include <osg/ShapeDrawable>
#include <osgViewer/Viewer>
#include <osgViewer/GraphicsWindow>
#if defined(_WIN32)
#include <osgViewer/api/Win32/GraphicsWindowWin32>
#endif
#include <osgGA/TrackballManipulator>
#include <osgGA/GUIEventHandler>

#include <atomic>
#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <limits>
#include <mutex>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>
#include <iomanip>
#if defined(_WIN32)
#include <windows.h>
#include <psapi.h>
#include <DbgHelp.h>
#pragma comment(lib, "Dbghelp.lib")
#endif


cv::Mat frameToMat(const sonar_types_v2::samples::frame::Frame& frame) {
    const int width = static_cast<int>(frame.size.width);
    const int height = static_cast<int>(frame.size.height);
    if (width <= 0 || height <= 0 || frame.image.empty()) {
        return cv::Mat();
    }
    cv::Mat img(height, width, CV_8UC3, const_cast<uint8_t*>(frame.image.data()));
    return img.clone();
}

cv::Mat makeVisibleSonarImage(const cv::Mat& src) {
    if (src.empty()) {
        return src;
    }
    cv::Mat gray;
    cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);
    double minv = 0.0;
    double maxv = 0.0;
    cv::minMaxLoc(gray, &minv, &maxv);
    if (maxv <= minv + 1.0) {
        return src;
    }
    cv::Mat stretched;
    const double scale = 255.0 / (maxv - minv);
    const double shift = -minv * scale;
    gray.convertTo(stretched, CV_8U, scale, shift);
    cv::Mat colored;
    cv::cvtColor(stretched, colored, cv::COLOR_GRAY2BGR);
    return colored;
}

struct SonarStats {
    float min_v = 0.0f;
    float max_v = 0.0f;
    double mean_v = 0.0;
    std::size_t non_zero = 0;
};

struct PoseState {
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double pitch = 0.0;
};

namespace {

std::mutex& rawDebugMutex() {
    static std::mutex m;
    return m;
}

std::ofstream& rawDebugFile() {
    static std::ofstream f("echoverse_sonar_lab_debug.log", std::ios::out | std::ios::app);
    return f;
}

std::string nowForDebug() {
    const auto now = std::chrono::system_clock::now();
    const auto tt = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S")
        << "." << std::setfill('0') << std::setw(3) << ms.count();
    return oss.str();
}

void rawDebug(const std::string& msg) {
    std::lock_guard<std::mutex> lock(rawDebugMutex());
    const std::string line = "[" + nowForDebug() + "] " + msg;
    std::cerr << line << std::endl;
    std::ofstream& f = rawDebugFile();
    if (f.is_open()) {
        f << line << std::endl;
        f.flush();
    }
}

void onSignalCrash(int sig) {
    rawDebug(std::string("[fatal] signal received: ") + std::to_string(sig));
    std::_Exit(128 + sig);
}

#if defined(_WIN32)
std::string makeCrashDumpFileName() {
    SYSTEMTIME st{};
    GetLocalTime(&st);
    std::ostringstream oss;
    oss << "echoverse_sonar_lab_crash_"
        << st.wYear
        << std::setfill('0') << std::setw(2) << st.wMonth
        << std::setfill('0') << std::setw(2) << st.wDay
        << "_"
        << std::setfill('0') << std::setw(2) << st.wHour
        << std::setfill('0') << std::setw(2) << st.wMinute
        << std::setfill('0') << std::setw(2) << st.wSecond
        << ".dmp";
    return oss.str();
}

void writeMiniDump(EXCEPTION_POINTERS* ep) {
    if (!ep) {
        rawDebug("[fatal] skip minidump: null exception pointers");
        return;
    }
    const std::string dump_name = makeCrashDumpFileName();
    HANDLE h = CreateFileA(
        dump_name.c_str(), GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        rawDebug(std::string("[fatal] minidump create failed file=") + dump_name);
        return;
    }

    MINIDUMP_EXCEPTION_INFORMATION mei{};
    mei.ThreadId = GetCurrentThreadId();
    mei.ExceptionPointers = ep;
    mei.ClientPointers = FALSE;

    const BOOL ok = MiniDumpWriteDump(
        GetCurrentProcess(),
        GetCurrentProcessId(),
        h,
        static_cast<MINIDUMP_TYPE>(MiniDumpWithThreadInfo | MiniDumpWithIndirectlyReferencedMemory),
        &mei,
        nullptr,
        nullptr);
    CloseHandle(h);
    rawDebug(std::string("[fatal] minidump ") + (ok ? "written: " : "failed: ") + dump_name);
}

LONG WINAPI onUnhandledException(EXCEPTION_POINTERS* ep) {
    if (!ep || !ep->ExceptionRecord) {
        rawDebug("[fatal] unhandled exception: null EXCEPTION_POINTERS");
        return EXCEPTION_EXECUTE_HANDLER;
    }
    void* crash_addr = ep->ExceptionRecord->ExceptionAddress;
    MEMORY_BASIC_INFORMATION mbi{};
    std::string module_name = "unknown";
    std::uintptr_t module_base = 0;
    if (VirtualQuery(crash_addr, &mbi, sizeof(mbi)) == sizeof(mbi)) {
        HMODULE mod = static_cast<HMODULE>(mbi.AllocationBase);
        module_base = reinterpret_cast<std::uintptr_t>(mod);
        char path_buf[MAX_PATH] = {};
        if (GetModuleFileNameA(mod, path_buf, MAX_PATH) > 0) {
            module_name = path_buf;
        }
    }
    const std::uintptr_t addr = reinterpret_cast<std::uintptr_t>(crash_addr);
    const std::uintptr_t offset = (module_base != 0 && addr >= module_base) ? (addr - module_base) : 0;

    std::ostringstream oss;
    oss << "[fatal] unhandled exception code=0x" << std::hex
        << static_cast<unsigned long>(ep->ExceptionRecord->ExceptionCode)
        << " address=0x" << addr
        << " module=" << module_name
        << " module_base=0x" << module_base
        << " module_offset=0x" << offset;
    rawDebug(oss.str());
    writeMiniDump(ep);
    return EXCEPTION_EXECUTE_HANDLER;
}
#endif

void installCrashDebugHooks() {
    std::signal(SIGABRT, onSignalCrash);
    std::signal(SIGFPE, onSignalCrash);
    std::signal(SIGILL, onSignalCrash);
#if !defined(_WIN32)
    std::signal(SIGSEGV, onSignalCrash);
#endif

    std::set_terminate([]() {
        std::string message = "[fatal] std::terminate called";
        if (const std::exception_ptr ep = std::current_exception(); ep) {
            try {
                std::rethrow_exception(ep);
            } catch (const std::exception& e) {
                message += std::string(", what=") + e.what();
            } catch (...) {
                message += ", non-std exception";
            }
        }
        rawDebug(message);
        std::_Exit(1);
    });

#if defined(_WIN32)
    SetUnhandledExceptionFilter(onUnhandledException);
#endif
}

} // namespace

// Build sonar body pose directly from the current camera view.
// Unified convention: X forward, Y left, Z up.
static Eigen::Affine3d bodyAffineFromCameraViewMatrix(const osg::Matrixd& view_matrix) {
    osg::Vec3d eye_osg;
    osg::Vec3d center_osg;
    osg::Vec3d up_osg;
    view_matrix.getLookAt(eye_osg, center_osg, up_osg, 1.0);

    const Eigen::Vector3d t(eye_osg.x(), eye_osg.y(), eye_osg.z());
    Eigen::Vector3d forward(center_osg.x() - eye_osg.x(),
                            center_osg.y() - eye_osg.y(),
                            center_osg.z() - eye_osg.z());
    if (forward.norm() < 1e-10) {
        forward = Eigen::Vector3d::UnitX();
    } else {
        forward.normalize();
    }

    const Eigen::Vector3d up_raw(up_osg.x(), up_osg.y(), up_osg.z());
    Eigen::Vector3d z = up_raw - forward * forward.dot(up_raw);
    if (z.norm() < 1e-10) {
        z = Eigen::Vector3d::UnitZ();
        z = z - forward * forward.dot(z);
    }
    z.normalize();
    const Eigen::Vector3d y = z.cross(forward).normalized();
    z = forward.cross(y).normalized();
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    pose.translation() = t;
    pose.linear().col(0) = forward;
    pose.linear().col(1) = y;
    pose.linear().col(2) = z;
    return pose;
}

static void syncTrackballToPose(osgGA::TrackballManipulator* tb, const PoseState& pose) {
    if (!tb) {
        return;
    }
    const Eigen::Quaterniond q =
        Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pose.pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    const Eigen::Vector3d eye = pose.position;
    const Eigen::Vector3d center = pose.position + q * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d up = q * Eigen::Vector3d::UnitZ();
    tb->setTransformation(osg::Vec3d(eye.x(), eye.y(), eye.z()), osg::Vec3d(center.x(), center.y(), center.z()),
                        osg::Vec3d(up.x(), up.y(), up.z()));
}

static PoseState poseStateFromBodyAffine(const Eigen::Affine3d& pose) {
    PoseState state;
    state.position = pose.translation();
    const Eigen::Vector3d forward = pose.linear().col(0).normalized();
    state.yaw = std::atan2(forward.y(), forward.x());
    state.pitch = std::asin(std::clamp(-forward.z(), -1.0, 1.0));
    return state;
}

static void setCameraViewFromPose(osg::Camera* camera, const PoseState& pose) {
    if (!camera) {
        return;
    }
    const Eigen::Quaterniond q =
        Eigen::AngleAxisd(pose.yaw, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(pose.pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    const Eigen::Vector3d forward = q * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d up = q * Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d eye = pose.position;
    const Eigen::Vector3d look_at = eye + forward;
    camera->setViewMatrixAsLookAt(
        osg::Vec3d(eye.x(), eye.y(), eye.z()),
        osg::Vec3d(look_at.x(), look_at.y(), look_at.z()),
        osg::Vec3d(up.x(), up.y(), up.z()));
}

bool applyControlKey(int key, PoseState& pose, double step_xy, double step_yaw, double step_pitch, double step_z) {
    constexpr double kPitchLimitRad = 1.4; // ~80 deg, keeps look-at stable
    const double cy = std::cos(pose.yaw);
    const double sy = std::sin(pose.yaw);
    switch (key) {
    case 'w':
    case 'W':
        pose.position.x() += step_xy * cy;
        pose.position.y() += step_xy * sy;
        return true;
    case 's':
    case 'S':
        pose.position.x() -= step_xy * cy;
        pose.position.y() -= step_xy * sy;
        return true;
    case 'a':
    case 'A':
        pose.position.x() += step_xy * (-sy);
        pose.position.y() += step_xy * cy;
        return true;
    case 'd':
    case 'D':
        pose.position.x() -= step_xy * (-sy);
        pose.position.y() -= step_xy * cy;
        return true;
    case 'q':
    case 'Q':
        pose.yaw += step_yaw;
        return true;
    case 'e':
    case 'E':
        pose.yaw -= step_yaw;
        return true;
    case 'u':
    case 'U':
        pose.pitch -= step_pitch; // nose up (X forward, Y left, Z up)
        pose.pitch = std::clamp(pose.pitch, -kPitchLimitRad, kPitchLimitRad);
        return true;
    case 'j':
    case 'J':
        pose.pitch += step_pitch; // nose down
        pose.pitch = std::clamp(pose.pitch, -kPitchLimitRad, kPitchLimitRad);
        return true;
    case 'i':
    case 'I':
        pose.position.z() += step_z;
        return true;
    case 'k':
    case 'K':
        pose.position.z() -= step_z;
        return true;
    default:
        return false;
    }
}

class PoseControlHandler : public osgGA::GUIEventHandler {
public:
    PoseControlHandler(PoseState& pose, double& step_xy, double& step_yaw, double& step_pitch, double& step_z,
                       osgGA::TrackballManipulator* tb)
        : pose_(pose), step_xy_(step_xy), step_yaw_(step_yaw), step_pitch_(step_pitch), step_z_(step_z), tb_(tb) {}

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override {
        if (ea.getEventType() != osgGA::GUIEventAdapter::KEYDOWN) {
            return false;
        }
        const int key = ea.getKey();
        if (!applyControlKey(key, pose_, step_xy_, step_yaw_, step_pitch_, step_z_)) {
            return false;
        }
        if (tb_) {
            syncTrackballToPose(tb_, pose_);
        }
        std::cout << "[gui] pose_cmd key=" << static_cast<char>(key)
                  << " xyz=(" << pose_.position.x() << "," << pose_.position.y() << "," << pose_.position.z()
                  << ") yaw=" << pose_.yaw << " pitch=" << pose_.pitch << std::endl;
        return true;
    }

private:
    PoseState& pose_;
    double& step_xy_;
    double& step_yaw_;
    double& step_pitch_;
    double& step_z_;
    osgGA::TrackballManipulator* tb_;
};

class KeyForwardFilter : public QObject {
public:
    KeyForwardFilter(PoseState& pose, double& step_xy, double& step_yaw, double& step_pitch, double& step_z,
                     osgGA::TrackballManipulator* tb)
        : pose_(pose), step_xy_(step_xy), step_yaw_(step_yaw), step_pitch_(step_pitch), step_z_(step_z), tb_(tb) {}

protected:
    bool eventFilter(QObject*, QEvent* event) override {
        if (event->type() != QEvent::KeyPress) {
            return false;
        }
        auto* key_event = static_cast<QKeyEvent*>(event);
        const int key = key_event->key();
        if (!applyControlKey(key, pose_, step_xy_, step_yaw_, step_pitch_, step_z_)) {
            return false;
        }
        if (tb_) {
            syncTrackballToPose(tb_, pose_);
        }
        return true;
    }

private:
    PoseState& pose_;
    double& step_xy_;
    double& step_yaw_;
    double& step_pitch_;
    double& step_z_;
    osgGA::TrackballManipulator* tb_;
};

SonarStats computeSonarStats(const std::vector<float>& bins) {
    SonarStats stats;
    if (bins.empty()) {
        return stats;
    }
    float min_v = std::numeric_limits<float>::max();
    float max_v = std::numeric_limits<float>::lowest();
    double sum_v = 0.0;
    std::size_t non_zero = 0;
    for (float v : bins) {
        min_v = std::min(min_v, v);
        max_v = std::max(max_v, v);
        sum_v += static_cast<double>(v);
        if (std::abs(v) > 1e-8f) {
            ++non_zero;
        }
    }
    stats.min_v = min_v;
    stats.max_v = max_v;
    stats.mean_v = sum_v / static_cast<double>(bins.size());
    stats.non_zero = non_zero;
    return stats;
}

static double radToDeg(double rad) {
    return rad * 180.0 / 3.14159265358979323846;
}

static double degToRad(double deg) {
    return deg * 3.14159265358979323846 / 180.0;
}

static unsigned int computeDerivedBinCount(const standalone_mvp::SonarConfigUi& s) {
    const double bandwidth_hz = std::max(1.0, s.bandwidth_khz * 1000.0);
    const double range_m = std::max(0.1, s.range_m);
    const double c_mps = std::max(1.0, s.sound_speed_mps);
    const double value = (bandwidth_hz * 2.0 * range_m) / c_mps;
    return static_cast<unsigned int>(std::max(1.0, std::floor(value)));
}

static unsigned int computeDerivedBeamCount(const standalone_mvp::SonarConfigUi& s) {
    const double beam_w = std::max(standalone_mvp::kMinSonarBeamDeg, s.beam_width_deg);
    const double ang_res = std::max(0.01, s.angular_resolution_deg);
    const double value = beam_w / ang_res;
    return static_cast<unsigned int>(std::max(1.0, std::floor(value)));
}

/// Build SonarConfigUi for SSS A/B from SSS module config + shared sonar environment.
static standalone_mvp::SonarConfigUi sonarConfigForSss(const standalone_mvp::SideScanSonarConfigUi& ss,
                                                       const standalone_mvp::SonarConfigUi& env_cfg,
                                                       float range_m) {
    standalone_mvp::SonarConfigUi s = env_cfg;
    s.range_m = static_cast<double>(range_m);
    s.gain = ss.gain;
    s.center_frequency_khz = ss.center_frequency_khz;
    s.bandwidth_khz = std::min(ss.bandwidth_khz, ss.center_frequency_khz - 0.1);
    s.angular_resolution_deg = std::clamp(ss.angular_resolution_deg, 0.05, 30.0);
    s.beam_width_deg = std::clamp(ss.beam_width_deg, standalone_mvp::kMinSonarBeamDeg, standalone_mvp::kMaxSonarBeamDeg);
    s.beam_height_deg = std::clamp(ss.beam_height_deg, standalone_mvp::kMinSonarBeamDeg, standalone_mvp::kMaxSonarBeamDeg);
    s.attenuation_frequency_khz = ss.center_frequency_khz;
    return s;
}

/// Keep RTT texture dimensions within a safe range (see OffscreenCaptureRig width/height derivation).
static void clampSssBinCountAndResolution(unsigned int& bin_count, unsigned int& resolution, float k_rc,
                                         unsigned max_tex_dim) {
    resolution = static_cast<unsigned int>(static_cast<float>(bin_count) * k_rc);
    if (resolution > max_tex_dim) {
        resolution = max_tex_dim;
        bin_count = std::max(1u, static_cast<unsigned int>(std::floor(static_cast<float>(resolution) / k_rc)));
    }
    resolution = std::max(64u, resolution);
}

int main(int argc, char** argv) {
    installCrashDebugHooks();
    rawDebug("[startup] echoverse_sonar_lab main entered");
    std::cout << "[gui] startup" << std::endl;
    QString project_path_cli;
    int max_frames = -1;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--project") == 0 && i + 1 < argc) {
            project_path_cli = QString::fromLocal8Bit(argv[++i]);
        } else if (std::strcmp(argv[i], "--max-frames") == 0 && i + 1 < argc) {
            max_frames = std::atoi(argv[++i]);
        } else if (argv[i][0] != '-') {
            bool ok = false;
            const int v = QString::fromLocal8Bit(argv[i]).toInt(&ok);
            if (ok && max_frames < 0) {
                max_frames = v;
            }
        }
    }

    // Qt must not consume argv[1] (frame limit); use a dedicated argv for QApplication.
    static int qt_argc = 1;
    static char qt_app_name[] = "EchoVerseSonarLab";
    static char* qt_argv[] = {qt_app_name, nullptr};
    QApplication qt_app(qt_argc, qt_argv);

    QString config_path_resolved;
    if (!project_path_cli.isEmpty()) {
        const QString resolved = standalone_mvp::resolveProjectFileArgument(project_path_cli);
        if (resolved.isEmpty()) {
            std::cerr << "[gui] error: could not resolve project path: " << project_path_cli.toStdString() << std::endl;
            return 1;
        }
        config_path_resolved = resolved;
    }
    standalone_mvp::AppConfigStore config_store(config_path_resolved);
    standalone_mvp::AppConfigData app_cfg = config_store.load();
    const bool saved_sonar_docked_in_main = app_cfg.sonar_window_docked_in_main;
    const QString saved_sonar_split_layout = app_cfg.sonar_workspace_split_layout.trimmed().toLower();
    const QString project_dir =
        QFileInfo(config_store.path()).absolutePath().isEmpty()
            ? QDir::currentPath()
            : QFileInfo(config_store.path()).absolutePath();
    if (standalone_mvp::ensureSonarParamFilesForProject(app_cfg, project_dir)) {
        config_store.save(app_cfg);
    }
    const standalone_mvp::SonarModuleConfig* primary_fls_module = nullptr;
    const standalone_mvp::SonarModuleConfig* primary_mbes_module = nullptr;
    const standalone_mvp::SonarModuleConfig* primary_sss_module = nullptr;
    int primary_fls_module_idx = -1;
    int primary_mbes_module_idx = -1;
    int primary_sss_module_idx = -1;
    std::vector<standalone_mvp::SonarModuleConfig> extra_fls_modules;
    std::vector<standalone_mvp::SonarModuleConfig> extra_mbes_modules;
    std::vector<standalone_mvp::SonarModuleConfig> extra_sss_modules;
    for (const auto& sm : app_cfg.sonar_modules) {
        if (!sm.enabled) {
            continue;
        }
        switch (sm.type) {
        case standalone_mvp::SonarModuleType::FLS:
            if (!primary_fls_module) {
                primary_fls_module = &sm;
                primary_fls_module_idx = static_cast<int>(&sm - app_cfg.sonar_modules.data());
            }
            else extra_fls_modules.push_back(sm);
            break;
        case standalone_mvp::SonarModuleType::MBES:
            if (!primary_mbes_module) {
                primary_mbes_module = &sm;
                primary_mbes_module_idx = static_cast<int>(&sm - app_cfg.sonar_modules.data());
            }
            else extra_mbes_modules.push_back(sm);
            break;
        case standalone_mvp::SonarModuleType::SSS:
            if (!primary_sss_module) {
                primary_sss_module = &sm;
                primary_sss_module_idx = static_cast<int>(&sm - app_cfg.sonar_modules.data());
            }
            else extra_sss_modules.push_back(sm);
            break;
        }
    }
    const standalone_mvp::SonarModuleConfig default_fls_module = [] {
        standalone_mvp::SonarModuleConfig m;
        m.type = standalone_mvp::SonarModuleType::FLS;
        m.enabled = false;
        m.fls_config.enable_2d_fls = false;
        m.point_cloud_config.enabled = false;
        return m;
    }();
    const standalone_mvp::SonarModuleConfig default_mbes_module = [] {
        standalone_mvp::SonarModuleConfig m;
        m.type = standalone_mvp::SonarModuleType::MBES;
        m.enabled = false;
        m.mbes_config.enable_2d_fls = false;
        return m;
    }();
    const standalone_mvp::SonarModuleConfig default_sss_module = [] {
        standalone_mvp::SonarModuleConfig m;
        m.type = standalone_mvp::SonarModuleType::SSS;
        m.enabled = false;
        m.sss_config.enabled = false;
        m.sss_camera_slot1.clear();
        m.sss_camera_slot2.clear();
        return m;
    }();
    const standalone_mvp::SonarModuleConfig& fls_module_cfg =
        primary_fls_module ? *primary_fls_module : default_fls_module;
    const standalone_mvp::SonarModuleConfig& mbes_module_cfg =
        primary_mbes_module ? *primary_mbes_module : default_mbes_module;
    const standalone_mvp::SonarModuleConfig& sss_module_cfg =
        primary_sss_module ? *primary_sss_module : default_sss_module;
    rawDebug(std::string("[config] path=") + config_store.path().toStdString());
    std::cout << "[gui] config path: " << config_store.path().toStdString() << std::endl;

    standalone_mvp::SonarConfigUi primary_fls_cfg = fls_module_cfg.fls_config;
    standalone_mvp::SonarConfigUi primary_mbes_cfg = mbes_module_cfg.mbes_config;
    standalone_mvp::SideScanSonarConfigUi primary_sss_cfg = sss_module_cfg.sss_config;
    standalone_mvp::EnvironmentConfig global_env_cfg = app_cfg.environment;
    primary_fls_cfg.bandwidth_khz = std::min(primary_fls_cfg.bandwidth_khz, primary_fls_cfg.center_frequency_khz - 0.1);
    primary_mbes_cfg.bandwidth_khz = std::min(primary_mbes_cfg.bandwidth_khz, primary_mbes_cfg.center_frequency_khz - 0.1);
    primary_sss_cfg.bandwidth_khz = std::min(primary_sss_cfg.bandwidth_khz, primary_sss_cfg.center_frequency_khz - 0.1);
    unsigned int bin_count = computeDerivedBinCount(primary_fls_cfg);
    unsigned int beam_count = computeDerivedBeamCount(primary_fls_cfg);
    unsigned int mbes_bin_count = computeDerivedBinCount(primary_mbes_cfg);
    unsigned int mbes_beam_count = computeDerivedBeamCount(primary_mbes_cfg);
    primary_fls_cfg.bin_count = static_cast<int>(bin_count);
    primary_fls_cfg.beam_count = static_cast<int>(beam_count);
    primary_mbes_cfg.bin_count = static_cast<int>(mbes_bin_count);
    primary_mbes_cfg.beam_count = static_cast<int>(mbes_beam_count);
    float range_m = static_cast<float>(primary_fls_cfg.range_m);
    float gain = static_cast<float>(primary_fls_cfg.gain);
    float beam_width_deg = static_cast<float>(primary_fls_cfg.beam_width_deg);
    float beam_height_deg = static_cast<float>(primary_fls_cfg.beam_height_deg);
    float mbes_range_m = static_cast<float>(primary_mbes_cfg.range_m);
    float mbes_gain = static_cast<float>(primary_mbes_cfg.gain);
    float mbes_beam_width_deg = static_cast<float>(primary_mbes_cfg.beam_width_deg);
    float mbes_beam_height_deg = static_cast<float>(primary_mbes_cfg.beam_height_deg);
    double sonar_max_fps = primary_fls_cfg.max_fps;
    double viewer_max_fps = primary_fls_cfg.viewer_max_fps;
    double sonar_sound_speed_mps = global_env_cfg.sound_speed_mps;
    double camera_hfov_deg = app_cfg.camera_system.main_camera.horizontal_fov_deg;
    double camera_vfov_deg = app_cfg.camera_system.main_camera.vertical_fov_deg;
    const bool third_person_view_enabled = app_cfg.scene.third_person_view_enabled;
    CameraModule camera_module(app_cfg);
    const standalone_mvp::SubCameraConfig* sss_slot1_cfg = camera_module.findSubCameraByName(
        sss_module_cfg.sss_camera_slot1.toStdString());
    const standalone_mvp::SubCameraConfig* sss_slot2_cfg = camera_module.findSubCameraByName(
        sss_module_cfg.sss_camera_slot2.toStdString());
    const standalone_mvp::SubCameraConfig* mbes_cfg = camera_module.findSubCameraByName(
        mbes_module_cfg.camera_binding.toStdString());
    const std::string sss_slot1_name = sss_module_cfg.sss_camera_slot1.toStdString();
    const std::string sss_slot2_name = sss_module_cfg.sss_camera_slot2.toStdString();
    const std::string mbes_camera_name = mbes_module_cfg.camera_binding.toStdString();
    const bool primary_side_scan_enabled = sss_module_cfg.enabled && sss_module_cfg.sss_config.enabled;
    const int side_scan_window_width = primary_sss_cfg.window_width;
    const int side_scan_window_height = primary_sss_cfg.window_height;
    const int side_scan_update_stride = std::max(1, primary_sss_cfg.update_stride);
    const float side_scan_range_m =
        std::clamp(static_cast<float>(primary_sss_cfg.range_m), 1.0f, 1000.0f);
    SssModule sss_module(sss_module_cfg);
    std::vector<QLabel*>& subcamera_labels = camera_module.labels;
    constexpr float kResolutionConstant = 5.12f; // imaging_sonar_simulation::Task::resolution_constant
    const unsigned int resolution = static_cast<unsigned int>(static_cast<float>(bin_count) * kResolutionConstant);
    const unsigned int mbes_resolution = static_cast<unsigned int>(static_cast<float>(mbes_bin_count) * kResolutionConstant);
    // SonarCanvas uses widget size in resizeEvent; match BASE_* scaling reference (see SonarCanvas.hpp).
    const int sonar_plot_w = [] {
        if (const char* e = std::getenv("STANDALONE_SONAR_PLOT_WIDTH")) {
            try {
                return std::max(64, std::stoi(e));
            } catch (...) {
            }
        }
        return 1166; // 1300 - 134 from SonarCanvas.hpp
    }();
    const int sonar_plot_h = [] {
        if (const char* e = std::getenv("STANDALONE_SONAR_PLOT_HEIGHT")) {
            try {
                return std::max(64, std::stoi(e));
            } catch (...) {
            }
        }
        return 600;
    }();

    const bool enable_2d_fls = fls_module_cfg.enabled && primary_fls_cfg.enable_2d_fls;

    // Default: full sonar control panel (canvas + range/gain/palette/grid). OpenCV preview is opt-in.
    const bool use_rock_sonar_ui = (std::getenv("STANDALONE_USE_OPENCV_SONAR") == nullptr);
    const bool enable_opencv_window =
        enable_2d_fls && (std::getenv("STANDALONE_USE_OPENCV_SONAR") != nullptr) &&
        (std::getenv("STANDALONE_DISABLE_OPENCV_WINDOW") == nullptr);
    int image_update_stride = 1;
    if (const char* stride_env = std::getenv("STANDALONE_IMAGE_UPDATE_STRIDE")) {
        try {
            image_update_stride = std::max(1, std::stoi(stride_env));
        } catch (...) {
            image_update_stride = 1;
        }
    }
    const bool enable_sonar_tick = (std::getenv("STANDALONE_DISABLE_SONAR_TICK") == nullptr);
    std::atomic<bool> scene_edit_pauses_sonar{false};
    bool enable_auto_pose = app_cfg.pose.enable_auto_pose;
    const std::string scripted_keys = std::getenv("STANDALONE_SCRIPT_KEYS") ? std::getenv("STANDALONE_SCRIPT_KEYS") : "";
    std::size_t scripted_key_index = 0;

    standalone_mvp::configureOsgDataPath();
    const QString world_for_scene =
        standalone_mvp::resolveSceneWorldForLoad(app_cfg.scene.world, config_store.path());
    const std::string world_spec = world_for_scene.toStdString();
    osg::ref_ptr<osg::Group> root =
        standalone_mvp::createSharedSceneGraph(range_m, world_spec);
    std::cout << "[gui] scene created, world=" << world_spec
              << ", children=" << root->getNumChildren() << std::endl;
    constexpr unsigned int kSceneMask = 0x1u;
    constexpr unsigned int kDebugCameraMask = 0x2u;
    osg::ref_ptr<osg::Group> camera_debug_group = new osg::Group();
    camera_debug_group->setNodeMask(kDebugCameraMask);
    root->addChild(camera_debug_group.get());
    // Independent scene graphs for side-scan acoustic simulators (each owns its own depth composer state).
    osg::ref_ptr<osg::Group> side_scan_scene_a;
    osg::ref_ptr<osg::Group> side_scan_scene_b;
    const bool sss_ready = sss_slot1_cfg && sss_slot2_cfg && sss_slot1_cfg->enabled && sss_slot2_cfg->enabled;
    const bool mbes_ready = mbes_cfg && mbes_cfg->enabled;
    if (primary_side_scan_enabled && sss_ready) {
        osg::Object* cloned_a = root->clone(osg::CopyOp(osg::CopyOp::DEEP_COPY_ALL));
        osg::Object* cloned_b = root->clone(osg::CopyOp(osg::CopyOp::DEEP_COPY_ALL));
        side_scan_scene_a = dynamic_cast<osg::Group*>(cloned_a);
        side_scan_scene_b = dynamic_cast<osg::Group*>(cloned_b);
        if (side_scan_scene_a.valid() && side_scan_scene_b.valid()) {
            std::cout << "[side_scan] scene cloned for SSS acoustic simulator x2" << std::endl;
        } else {
            std::cout << "[side_scan] warning: scene clone failed" << std::endl;
        }
    }
    FlsModule fls_module(fls_module_cfg);
    fls_module.setEnvironmentConfig(global_env_cfg);
    std::unique_ptr<sonar_core::AcousticRaySimulator>& sonar = fls_module.sonar;
    MbesModule mbes_module(mbes_module_cfg);
    mbes_module.setEnvironmentConfig(global_env_cfg);
    std::unique_ptr<sonar_core::AcousticRaySimulator>& mbes_sonar = mbes_module.sonar;
    std::vector<std::unique_ptr<FlsModule>> extra_fls_modules_rt;
    std::vector<std::unique_ptr<MbesModule>> extra_mbes_modules_rt;
    std::vector<std::unique_ptr<SssModule>> extra_sss_modules_rt;
    if (!fls_module.initSimulation(root, kResolutionConstant)) {
        std::cout << "[gui] 2D FLS disabled: skipping acoustic simulator and sonar image rendering" << std::endl;
    }
    if (!(mbes_ready && mbes_module.initSimulation(root, kResolutionConstant))) {
        std::cout << "[gui] MBES sonar disabled: skipping MBES acoustic simulator and sonar rendering" << std::endl;
    }
    std::unique_ptr<sonar_core::AcousticRaySimulator>& side_scan_sonar_a = sss_module.sonar_a;
    std::unique_ptr<sonar_core::AcousticRaySimulator>& side_scan_sonar_b = sss_module.sonar_b;
    if (sss_module.sonarEnabledByBinding() && side_scan_scene_a.valid() && side_scan_scene_b.valid() && sss_ready) {
        try {
            const standalone_mvp::SonarConfigUi cfg_sss = sonarConfigForSss(primary_sss_cfg, primary_fls_cfg, side_scan_range_m);
            unsigned int ss_bin = computeDerivedBinCount(cfg_sss);
            unsigned int ss_beam = computeDerivedBeamCount(cfg_sss);
            unsigned int ss_res = static_cast<unsigned int>(static_cast<float>(ss_bin) * kResolutionConstant);
            constexpr unsigned int kMaxSssTexDim = 4096u;
            clampSssBinCountAndResolution(ss_bin, ss_res, kResolutionConstant, kMaxSssTexDim);
            const float sss_gain_for_sim = static_cast<float>(primary_sss_cfg.gain);
            side_scan_sonar_a = std::make_unique<sonar_core::AcousticRaySimulator>(
                side_scan_range_m,
                sss_gain_for_sim,
                ss_bin,
                sonar_types_v2::Angle::fromDeg(static_cast<float>(cfg_sss.beam_width_deg)),
                sonar_types_v2::Angle::fromDeg(static_cast<float>(cfg_sss.beam_height_deg)),
                ss_res,
                false,
                side_scan_scene_a);
            side_scan_sonar_a->setSonarBeamCount(ss_beam);
            side_scan_sonar_a->enableReverb(global_env_cfg.enable_reverb);
            side_scan_sonar_a->enableSpeckleNoise(global_env_cfg.enable_speckle);
            side_scan_sonar_b = std::make_unique<sonar_core::AcousticRaySimulator>(
                side_scan_range_m,
                sss_gain_for_sim,
                ss_bin,
                sonar_types_v2::Angle::fromDeg(static_cast<float>(cfg_sss.beam_width_deg)),
                sonar_types_v2::Angle::fromDeg(static_cast<float>(cfg_sss.beam_height_deg)),
                ss_res,
                false,
                side_scan_scene_b);
            side_scan_sonar_b->setSonarBeamCount(ss_beam);
            side_scan_sonar_b->enableReverb(global_env_cfg.enable_reverb);
            side_scan_sonar_b->enableSpeckleNoise(global_env_cfg.enable_speckle);
            std::cout << "[side_scan] acoustic simulator ready (SSS A/B)" << std::endl;
        } catch (const std::exception& e) {
            side_scan_sonar_a.reset();
            side_scan_sonar_b.reset();
            std::cout << "[side_scan] acoustic simulator init failed: " << e.what() << std::endl;
        }
    }
    for (const auto& mod : extra_fls_modules) {
        try {
            auto rt = std::make_unique<FlsModule>(mod);
            rt->setEnvironmentConfig(global_env_cfg);
            const bool has_2d_runtime = rt->initSimulation(root, kResolutionConstant);
            if (has_2d_runtime || rt->pointCloudEnabledByBinding()) {
                extra_fls_modules_rt.push_back(std::move(rt));
            }
        } catch (...) {
        }
    }
    for (const auto& mod : extra_mbes_modules) {
        try {
            auto rt = std::make_unique<MbesModule>(mod);
            rt->setEnvironmentConfig(global_env_cfg);
            if (rt->initSimulation(root, kResolutionConstant)) {
                extra_mbes_modules_rt.push_back(std::move(rt));
            }
        } catch (...) {
        }
    }
    for (const auto& mod : extra_sss_modules) {
        if (mod.sss_camera_slot1.trimmed().isEmpty() || mod.sss_camera_slot2.trimmed().isEmpty()) {
            continue;
        }
        try {
            auto rt = std::make_unique<SssModule>(mod);
            osg::Object* cloned_a = root->clone(osg::CopyOp(osg::CopyOp::DEEP_COPY_ALL));
            osg::Object* cloned_b = root->clone(osg::CopyOp(osg::CopyOp::DEEP_COPY_ALL));
            rt->scene_a = dynamic_cast<osg::Group*>(cloned_a);
            rt->scene_b = dynamic_cast<osg::Group*>(cloned_b);
            if (!rt->scene_a.valid() || !rt->scene_b.valid()) {
                continue;
            }
            const standalone_mvp::SonarConfigUi cfg_sss = sonarConfigForSss(mod.sss_config, primary_fls_cfg, static_cast<float>(mod.sss_config.range_m));
            unsigned int ss_bin = computeDerivedBinCount(cfg_sss);
            unsigned int ss_beam = computeDerivedBeamCount(cfg_sss);
            unsigned int ss_res = static_cast<unsigned int>(static_cast<float>(ss_bin) * kResolutionConstant);
            constexpr unsigned int kMaxSssTexDim = 4096u;
            clampSssBinCountAndResolution(ss_bin, ss_res, kResolutionConstant, kMaxSssTexDim);
            rt->sonar_a = std::make_unique<sonar_core::AcousticRaySimulator>(
                static_cast<float>(mod.sss_config.range_m), static_cast<float>(mod.sss_config.gain), ss_bin,
                sonar_types_v2::Angle::fromDeg(static_cast<float>(cfg_sss.beam_width_deg)),
                sonar_types_v2::Angle::fromDeg(static_cast<float>(cfg_sss.beam_height_deg)), ss_res, false, rt->scene_a);
            rt->sonar_b = std::make_unique<sonar_core::AcousticRaySimulator>(
                static_cast<float>(mod.sss_config.range_m), static_cast<float>(mod.sss_config.gain), ss_bin,
                sonar_types_v2::Angle::fromDeg(static_cast<float>(cfg_sss.beam_width_deg)),
                sonar_types_v2::Angle::fromDeg(static_cast<float>(cfg_sss.beam_height_deg)), ss_res, false, rt->scene_b);
            rt->sonar_a->setSonarBeamCount(ss_beam);
            rt->sonar_b->setSonarBeamCount(ss_beam);
            rt->sonar_a->enableReverb(global_env_cfg.enable_reverb);
            rt->sonar_b->enableReverb(global_env_cfg.enable_reverb);
            rt->sonar_a->enableSpeckleNoise(global_env_cfg.enable_speckle);
            rt->sonar_b->enableSpeckleNoise(global_env_cfg.enable_speckle);
            extra_sss_modules_rt.push_back(std::move(rt));
        } catch (...) {
        }
    }
    const bool is_tank = (world_spec.find("tank") != std::string::npos);
    osg::Vec3d suggested_pos;
    double suggested_yaw = 0.0;
    double suggested_pitch = 0.0;
    const bool has_suggested_pose =
        standalone_mvp::computeInitialPoseNearCollision(world_spec, range_m, suggested_pos, suggested_yaw, suggested_pitch);
    const Eigen::Vector3d base_pos = has_suggested_pose
        ? Eigen::Vector3d(suggested_pos.x(), suggested_pos.y(), suggested_pos.z())
        : (is_tank ? Eigen::Vector3d(10.5, 9.63, 0.0) : Eigen::Vector3d(49.34, 106.01, -5.87));
    const double base_yaw = has_suggested_pose ? suggested_yaw : (is_tank ? -0.00 : 0.00);
    const Eigen::Vector3d configured_pos(app_cfg.pose.x, app_cfg.pose.y, app_cfg.pose.z);
    const double configured_yaw = degToRad(app_cfg.camera_system.main_camera.yaw_deg);
    const double configured_pitch = degToRad(app_cfg.camera_system.main_camera.pitch_deg);
    PoseState configured_pose;
    configured_pose.position = configured_pos;
    configured_pose.yaw = configured_yaw;
    configured_pose.pitch = configured_pitch;
    if (sonar) {
        sonar->setAttenuationCoefficient(primary_fls_cfg.center_frequency_khz, global_env_cfg.temperature_c,
                                         std::max(0.0, -configured_pos.z()), global_env_cfg.salinity_ppt,
                                         global_env_cfg.acidity_ph, global_env_cfg.enable_attenuation);
        std::cout << "[gui] sonar backend created (cfg: range=" << range_m << " gain=" << gain << " bins=" << bin_count
                  << " beams=" << beam_count << " res=" << resolution << " image_mode=paper_sonar_pipeline"
                  << ")" << std::endl;
    }
    if (mbes_sonar) {
        mbes_sonar->setAttenuationCoefficient(primary_mbes_cfg.center_frequency_khz, primary_mbes_cfg.temperature_c,
                                              std::max(0.0, -configured_pos.z()), primary_mbes_cfg.salinity_ppt,
                                              primary_mbes_cfg.acidity_ph, primary_mbes_cfg.enable_attenuation);
    }
    if (side_scan_sonar_a) {
        side_scan_sonar_a->setAttenuationCoefficient(primary_sss_cfg.center_frequency_khz, global_env_cfg.temperature_c,
                                                     std::max(0.0, -configured_pos.z()), global_env_cfg.salinity_ppt,
                                                     global_env_cfg.acidity_ph, global_env_cfg.enable_attenuation);
    }
    if (side_scan_sonar_b) {
        side_scan_sonar_b->setAttenuationCoefficient(primary_sss_cfg.center_frequency_khz, global_env_cfg.temperature_c,
                                                     std::max(0.0, -configured_pos.z()), global_env_cfg.salinity_ppt,
                                                     global_env_cfg.acidity_ph, global_env_cfg.enable_attenuation);
    }
    std::cout << "[gui] initial_pose cfg xyz=(" << configured_pose.position.x() << "," << configured_pose.position.y()
              << "," << configured_pose.position.z() << ") yaw=" << configured_pose.yaw
              << " pitch=" << configured_pose.pitch
              << " suggested_xyz=(" << base_pos.x() << "," << base_pos.y() << "," << base_pos.z()
              << ")" << std::endl;

    QWidget sonar_tab_window;
    DockWorkspace* sonar_workspace = nullptr;
    if (use_rock_sonar_ui) {
        sonar_tab_window.setWindowTitle(QStringLiteral("EchoVerse Sonar Lab - Sonar Images"));
        sonar_tab_window.resize(1100, 720);
        sonar_tab_window.setStyleSheet(QStringLiteral("QWidget{background:#505050;color:#eaf4ff;}"));
        auto* sonar_tab_layout = new QVBoxLayout(&sonar_tab_window);
        sonar_tab_layout->setContentsMargins(4, 4, 4, 4);
        sonar_workspace = new DockWorkspace(&sonar_tab_window);
        sonar_tab_layout->addWidget(sonar_workspace);
    }
    if (use_rock_sonar_ui && sonar) {
        fls_module.setupWidget(sonar_workspace, QStringLiteral("%1 (FLS)").arg(fls_module_cfg.name));
    }
    if (use_rock_sonar_ui && mbes_sonar) {
        mbes_module.setupWidget(sonar_workspace, QStringLiteral("%1 (MBES)").arg(mbes_module_cfg.name));
    }
    if (use_rock_sonar_ui) {
        for (auto& extra : extra_fls_modules_rt) {
            if (!extra->sonar) {
                continue;
            }
            extra->setupWidget(sonar_workspace, QStringLiteral("%1 (FLS)").arg(extra->module_cfg.name));
        }
        for (auto& extra : extra_mbes_modules_rt) {
            if (!extra->sonar) {
                continue;
            }
            extra->setupWidget(sonar_workspace, QStringLiteral("%1 (MBES)").arg(extra->module_cfg.name));
        }
        const int tab_count = (sonar_workspace && sonar_workspace->primaryTabWidget())
            ? sonar_workspace->primaryTabWidget()->count()
            : 0;
        std::cout << "[gui] sonar control tabs host ready (tabs=" << tab_count << ")"
                  << std::endl;
    }
    {
        Eigen::Affine3d initial_pose = Eigen::Affine3d::Identity();
        initial_pose.translation() = configured_pose.position;
        initial_pose.linear() =
            (Eigen::AngleAxisd(configured_pose.yaw, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(configured_pose.pitch, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()))
                .toRotationMatrix();
        fls_module.initPointCloudRuntime(root, initial_pose, 80, 700, project_dir, sonar_workspace);
        int extra_idx = 0;
        for (auto& extra : extra_fls_modules_rt) {
            extra->initPointCloudRuntime(
                root, initial_pose, 120 + 40 * (extra_idx % 3), 740 + 40 * extra_idx, project_dir, sonar_workspace);
            ++extra_idx;
        }
    }
    {
        Eigen::Affine3d initial_pose = Eigen::Affine3d::Identity();
        initial_pose.translation() = configured_pose.position;
        initial_pose.linear() =
            (Eigen::AngleAxisd(configured_pose.yaw, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(configured_pose.pitch, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()))
                .toRotationMatrix();
        mbes_module.initPointCloudRuntime(root, initial_pose, 80, 700, project_dir, sonar_workspace);
        int extra_idx = 0;
        for (auto& extra : extra_mbes_modules_rt) {
            extra->initPointCloudRuntime(
                root, initial_pose, 80 + 40 * (extra_idx % 3), 700 + 40 * extra_idx, project_dir, sonar_workspace);
            ++extra_idx;
        }
    }

    const int viewer_w = std::max(320, app_cfg.scene.viewer_width);
    const int viewer_h = standalone_mvp::viewerHeightForSonarAspect(viewer_w, beam_width_deg, beam_height_deg);
    std::cout << "[gui] viewer window " << viewer_w << "x" << viewer_h
              << " (pixel aspect matched to sonar " << beam_width_deg << "x" << beam_height_deg << " deg FOV)"
              << std::endl;

    osgViewer::Viewer viewer;
    viewer.setSceneData(root);
    viewer.setUpViewInWindow(0, 0, viewer_w, viewer_h);
    // Keep OSG rendering in one thread. With embedded Qt windows and a second OSG viewer
    // (point cloud window), default threaded models can intermittently crash in osg.dll.
    viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
    viewer.getCamera()->setClearColor(osg::Vec4(0.03f, 0.05f, 0.08f, 1.0f));
    viewer.getCamera()->setCullMask(kSceneMask);
    osg::ref_ptr<osgGA::TrackballManipulator> trackball = new osgGA::TrackballManipulator();
    viewer.setCameraManipulator(trackball.get());
    std::unique_ptr<osgViewer::Viewer> third_viewer;
    if (third_person_view_enabled) {
        third_viewer = std::make_unique<osgViewer::Viewer>();
        third_viewer->setSceneData(root);
        third_viewer->setUpViewInWindow(80, 80, 960, 640);
        third_viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);
        third_viewer->getCamera()->setCullMask(kSceneMask | kDebugCameraMask);
        third_viewer->getCamera()->setClearColor(osg::Vec4(0.02f, 0.02f, 0.03f, 1.0f));
        third_viewer->setCameraManipulator(new osgGA::TrackballManipulator());
        std::cout << "[third_person] enabled: free camera debug window opened" << std::endl;
    }
    // Home pose uses the same body convention as sonar sim: +X forward, +Y left, +Z up.
    {
        const Eigen::Quaterniond q =
            Eigen::AngleAxisd(configured_pose.yaw, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(configured_pose.pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
        const Eigen::Vector3d eye = configured_pose.position;
        const Eigen::Vector3d look_at = configured_pose.position + q * Eigen::Vector3d::UnitX();
        const Eigen::Vector3d up = q * Eigen::Vector3d::UnitZ();
        trackball->setHomePosition(osg::Vec3d(eye.x(), eye.y(), eye.z()), osg::Vec3d(look_at.x(), look_at.y(), look_at.z()),
                                   osg::Vec3d(up.x(), up.y(), up.z()));
    }
    viewer.home();
    camera_module.buildSubCameras(viewer, camera_debug_group.get(), kSceneMask, sss_slot1_name, sss_slot2_name);
    std::vector<SubCameraRuntime>& sub_cameras = camera_module.sub_cameras;
    auto poseFromCameraBinding = [&](const std::string& binding, Eigen::Affine3d& out_pose) -> bool {
        if (binding.empty() || binding == "Main Camera") {
            out_pose = bodyAffineFromCameraViewMatrix(viewer.getCamera()->getViewMatrix());
            return true;
        }
        const SubCameraRuntime* rt = camera_module.findRuntimeByName(binding);
        if (!rt || !rt->camera) {
            return false;
        }
        out_pose = bodyAffineFromCameraViewMatrix(rt->camera->getViewMatrix());
        return true;
    };
    // Match pose_control.rb defaults: 1.0 m and 5 deg increments; pitch same as yaw; vertical step same as XY.
    double pose_step_xy = std::max(0.01, app_cfg.pose.step_xy);
    double pose_step_yaw = degToRad(app_cfg.pose.step_yaw_deg);
    double pose_step_pitch = pose_step_yaw;
    double pose_step_z = std::max(0.01, app_cfg.pose.step_z);
    std::cout << "[gui] viewer created" << std::endl;

    if (enable_opencv_window) {
        cv::namedWindow("sonar_image", cv::WINDOW_AUTOSIZE);
        cv::resizeWindow("sonar_image", sonar_plot_w, sonar_plot_h);
        cv::moveWindow("sonar_image", 1440, 80);
        std::cout << "[gui] OpenCV sonar preview (SonarCanvas capture), update_stride=" << image_update_stride << std::endl;
    } else if (fls_module.rock_sonar_ui) {
        std::cout << "[gui] sonar display=sonar control panel (use STANDALONE_USE_OPENCV_SONAR=1 for OpenCV window instead)"
                  << std::endl;
    } else {
        std::cout << "[gui] no sonar image window" << std::endl;
    }
    std::vector<float> prev_bins;
    PoseState commanded_pose;
    commanded_pose = configured_pose;
    const PoseState auto_pose_origin = configured_pose;
    viewer.addEventHandler(
        new PoseControlHandler(commanded_pose, pose_step_xy, pose_step_yaw, pose_step_pitch, pose_step_z, trackball.get()));
    std::cout << "[gui] control mode=" << (enable_auto_pose ? "auto(debug)" : "manual(default)")
              << " keys=W/S/A/D plane Q/E yaw U/J pitch I/K up/down" << std::endl;
    std::cout << "[gui] pose_mode=camera-driven (camera pose feeds sonar directly)" << std::endl;
    if (!scripted_keys.empty()) {
        std::cout << "[gui] scripted keys enabled: " << scripted_keys << std::endl;
    }

    QWidget dashboard_window;
    dashboard_window.setWindowTitle("EchoVerse Sonar Lab");
    dashboard_window.resize(1660, 600);
    dashboard_window.setMinimumSize(1100, 560);
    dashboard_window.setStyleSheet("QWidget{background:#000000;color:#eaf4ff;}");
    auto* root_layout = new QVBoxLayout(&dashboard_window);
    root_layout->setContentsMargins(10, 10, 10, 10);
    root_layout->setSpacing(8);
    auto* top_bar = new QHBoxLayout();
    top_bar->addStretch();
    QPushButton sonar_dock_button("Show Sonar", &dashboard_window);
    QPushButton settings_button("Settings", &dashboard_window);
    QPushButton scene_editor_button("Scene Editor", &dashboard_window);
    sonar_dock_button.setStyleSheet(
        "QPushButton{background:#3b4f66;color:#ffffff;border:1px solid #a7c6e8;border-radius:6px;padding:6px 12px;font-weight:600;}"
        "QPushButton:hover{background:#4b6888;}");
    scene_editor_button.setStyleSheet(
        "QPushButton{background:#295f3b;color:#ffffff;border:1px solid #8ac0a0;border-radius:6px;padding:6px 12px;font-weight:600;}"
        "QPushButton:hover{background:#3a7a4e;}");
    settings_button.setStyleSheet(
        "QPushButton{background:#1f5c97;color:#ffffff;border:1px solid #97c0e6;border-radius:6px;padding:6px 12px;font-weight:600;}"
        "QPushButton:hover{background:#2f74b5;}");
    top_bar->addWidget(&sonar_dock_button, 0, Qt::AlignRight);
    top_bar->addWidget(&scene_editor_button, 0, Qt::AlignRight);
    top_bar->addWidget(&settings_button, 0, Qt::AlignRight);
    root_layout->addLayout(top_bar);

    QWidget side_scan_window;
    auto add_sss_instance_ui = [&](SssModule& mod, const QString& tab_label, bool ready) {
        QWidget* sss_host = nullptr;
        if (sonar_workspace) {
            sss_host = new QWidget();
            sss_host->setMinimumSize(120, 80);
            sss_host->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
            sonar_workspace->addTab(sss_host, tab_label);
        } else {
            side_scan_window.setWindowTitle(QString::fromLatin1("Side Scan — waterfall (SSS A port | SSS B starboard)"));
            side_scan_window.resize(side_scan_window_width, side_scan_window_height);
            side_scan_window.setStyleSheet("QWidget{background:#000000;color:#dcefff;}");
            sss_host = &side_scan_window;
        }
        auto* side_scan_layout = new QVBoxLayout(sss_host);
        side_scan_layout->setContentsMargins(6, 6, 6, 6);
        side_scan_layout->setSpacing(4);
        auto* side_scan_title = new QLabel(
            QStringLiteral("Side-scan waterfall: horizontal axis is distance (-range to +range), vertical axis is time; newest echoes appear at the bottom."),
            sss_host);
        side_scan_title->setWordWrap(true);
        side_scan_title->setAlignment(Qt::AlignCenter);
        side_scan_title->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Minimum);
        side_scan_title->setStyleSheet("QLabel{color:#dcefff;background:#000000;font-size:12px;font-weight:600;}");
        side_scan_layout->addWidget(side_scan_title);
        if (ready) {
            auto* row = new QWidget(sss_host);
            auto* row_layout = new QHBoxLayout(row);
            row_layout->setContentsMargins(0, 0, 0, 0);
            row_layout->setSpacing(6);
            mod.setupStripWidget(row);
            row_layout->addWidget(mod.strip_widget, 1);
            side_scan_layout->addWidget(row, 1);
        } else {
            auto* err = new QLabel(
                QStringLiteral("This side-scan instance is not ready. Check SSS A/B camera bindings and sonar simulation initialization."),
                sss_host);
            err->setAlignment(Qt::AlignCenter);
            err->setWordWrap(true);
            err->setStyleSheet("QLabel{color:#97a8ba;background:#000000;border:1px solid #35506b;padding:8px;}");
            side_scan_layout->addWidget(err, 1);
        }
    };
    if (primary_side_scan_enabled || !extra_sss_modules_rt.empty()) {
        add_sss_instance_ui(sss_module, QStringLiteral("SSS %1").arg(sss_module_cfg.name), side_scan_sonar_a && side_scan_sonar_b);
        for (auto& extra : extra_sss_modules_rt) {
            const bool ready = extra->sonar_a && extra->sonar_b;
            add_sss_instance_ui(*extra, QStringLiteral("SSS %1").arg(extra->module_cfg.name), ready);
        }
        if (!sonar_workspace) {
            side_scan_window.show();
        }
    }
    const bool has_sonar_tabs =
        use_rock_sonar_ui && sonar_workspace && sonar_workspace->primaryTabWidget() &&
        sonar_workspace->primaryTabWidget()->count() > 0;
    auto preset_from_saved_layout = [&](const QString& text) {
        if (text == "horizontal") return DockWorkspace::LayoutPreset::Horizontal;
        if (text == "vertical") return DockWorkspace::LayoutPreset::Vertical;
        if (text == "quad") return DockWorkspace::LayoutPreset::Quad;
        return DockWorkspace::LayoutPreset::Single;
    };
    if (has_sonar_tabs && sonar_workspace) {
        sonar_workspace->applyLayoutPreset(preset_from_saved_layout(saved_sonar_split_layout));
    }

    auto* content_container = new QWidget(&dashboard_window);
    content_container->setMinimumHeight(0);
    content_container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto* content_layout = new QHBoxLayout(content_container);
    content_layout->setSpacing(10);
    constexpr int kInfoDrawerWidth = 360;
    constexpr int kInfoDrawerMargin = 10;
    constexpr int kInfoDrawerTop = 54;
    constexpr int kInfoDrawerBottom = 10;
    bool info_drawer_visible = false;
    auto* info_panel = new standalone_mvp::InfoPanelWidget(&dashboard_window);
    info_panel->setFixedWidth(kInfoDrawerWidth);
    info_panel->setWindowFlags(Qt::Tool | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
    info_panel->setAttribute(Qt::WA_ShowWithoutActivating, true);
    info_panel->setVisible(info_drawer_visible);
    info_panel->setStyleSheet(
        "QWidget{background:#000000;border:1px solid #6ea2d4;}");
    QPushButton info_drawer_toggle_button("Show Info", &dashboard_window);
    info_drawer_toggle_button.setAutoDefault(false);
    info_drawer_toggle_button.setDefault(false);
    info_drawer_toggle_button.setStyleSheet(
        "QPushButton{background:#1f5c97;color:#ffffff;border:1px solid #97c0e6;border-radius:6px;padding:6px 12px;font-weight:600;}"
        "QPushButton:hover{background:#2f74b5;}");
    QObject::connect(&info_drawer_toggle_button, &QPushButton::clicked, [&]() {
        info_drawer_visible = !info_drawer_visible;
        info_panel->setVisible(info_drawer_visible);
        info_drawer_toggle_button.setText(info_drawer_visible ? "Hide Info" : "Show Info");
        if (info_drawer_visible) {
            info_panel->raise();
        }
        info_drawer_toggle_button.raise();
    });

    auto* viewer_frame = new QFrame(&dashboard_window);
    viewer_frame->setStyleSheet("QFrame{background:#000000;border:1px solid #6ea2d4;}");
    content_layout->addWidget(viewer_frame, 1);
    auto* scene_editor_dialog = new QDialog(&dashboard_window);
    scene_editor_dialog->setWindowTitle("Scene Editor");
    scene_editor_dialog->setWindowFlag(Qt::Window, true);
    scene_editor_dialog->resize(900, 560);
    auto* scene_editor_dialog_layout = new QVBoxLayout(scene_editor_dialog);
    scene_editor_dialog_layout->setContentsMargins(8, 8, 8, 8);
    auto* scene_editor = new standalone_mvp::SceneEditorPanel(scene_editor_dialog);
    {
        const QString wf =
            QFileInfo(world_for_scene).isFile() ? QFileInfo(world_for_scene).absoluteFilePath() : QString();
        scene_editor->setWorldFile(wf, world_for_scene);
        scene_editor->setWorldModelsGroup(standalone_mvp::findWorldModelsGroup(root.get()), range_m);
        scene_editor->setPauseSonarCallback([&](bool on) { scene_edit_pauses_sonar.store(on); });
    }
    scene_editor_dialog_layout->addWidget(scene_editor);
    auto* sonar_dock_panel = new QWidget(&dashboard_window);
    sonar_dock_panel->setStyleSheet("QWidget{background:#505050;color:#eaf4ff;border:1px solid #6ea2d4;}");
    sonar_dock_panel->setMinimumHeight(0);
    sonar_dock_panel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto* sonar_dock_layout = new QVBoxLayout(sonar_dock_panel);
    sonar_dock_layout->setContentsMargins(4, 4, 4, 4);
    sonar_dock_layout->setSpacing(0);
    if (has_sonar_tabs && sonar_workspace) {
        sonar_workspace->setParent(sonar_dock_panel);
        sonar_dock_layout->addWidget(sonar_workspace, 1);
    }
    QWidget sonar_floating_window;
    sonar_floating_window.setWindowTitle(QStringLiteral("EchoVerse Sonar Lab - Sonar Images"));
    sonar_floating_window.resize(1100, 720);
    sonar_floating_window.setStyleSheet(QStringLiteral("QWidget{background:#505050;color:#eaf4ff;}"));
    auto* sonar_floating_layout = new QVBoxLayout(&sonar_floating_window);
    sonar_floating_layout->setContentsMargins(4, 4, 4, 4);
    sonar_floating_layout->setSpacing(0);
    sonar_floating_window.setAttribute(Qt::WA_QuitOnClose, false);
    auto* main_vertical_splitter = new QSplitter(Qt::Vertical, &dashboard_window);
    main_vertical_splitter->setChildrenCollapsible(false);
    main_vertical_splitter->addWidget(content_container);
    main_vertical_splitter->addWidget(sonar_dock_panel);
    main_vertical_splitter->setStretchFactor(0, 1);
    main_vertical_splitter->setStretchFactor(1, 1);
    root_layout->addWidget(main_vertical_splitter, 1);

    auto* viewer_host = new QWidget(viewer_frame);
    viewer_host->setStyleSheet("QWidget{background:#000000;border:1px solid #3f6c95;}");
    auto* subcamera_panel = new QWidget(viewer_frame);
    subcamera_panel->setStyleSheet("QWidget{background:#0d121a;border:1px solid #35506b;}");
    auto* subcamera_layout = new QVBoxLayout(subcamera_panel);
    subcamera_layout->setContentsMargins(6, 6, 6, 6);
    subcamera_layout->setSpacing(6);
    auto makeSubcameraLabel = [&](const QString& title) {
        auto* box = new QFrame(subcamera_panel);
        box->setStyleSheet("QFrame{background:#060a10;border:1px solid #4a6f93;}");
        auto* v = new QVBoxLayout(box);
        v->setContentsMargins(4, 4, 4, 4);
        v->setSpacing(4);
        auto* t = new QLabel(title, box);
        t->setStyleSheet("QLabel{color:#cfe8ff;font-size:11px;font-weight:600;background:transparent;}");
        auto* l = new QLabel("disabled", box);
        l->setAlignment(Qt::AlignCenter);
        l->setMinimumSize(120, 70);
        l->setStyleSheet("QLabel{color:#97a8ba;background:#000000;border:1px solid #22364a;}");
        v->addWidget(t);
        v->addWidget(l, 1);
        subcamera_layout->addWidget(box, 1);
        return l;
    };
    subcamera_labels.clear();
    for (const auto& cam_name : camera_module.sub_names) {
        subcamera_labels.push_back(makeSubcameraLabel(QString::fromStdString(cam_name)));
    }
    for (std::size_t i = 0; i < sub_cameras.size() && i < subcamera_labels.size(); ++i) {
        sub_cameras[i].label = subcamera_labels[i];
    }
    subcamera_panel->show();
    auto* capture_label = new QLabel("Optical capture image", viewer_frame);
    capture_label->setStyleSheet("QLabel{color:#ffffff;background:rgba(0,0,0,120);padding:4px 8px;border:1px solid #808080;}");
    capture_label->show();
#if defined(_WIN32)
    auto* gw_win32 = dynamic_cast<osgViewer::GraphicsWindowWin32*>(viewer.getCamera()->getGraphicsContext());
    QWindow* osg_foreign_window = gw_win32 ? QWindow::fromWinId(reinterpret_cast<WId>(gw_win32->getHWND())) : nullptr;
#else
    QWindow* osg_foreign_window = nullptr;
#endif
    QWidget* osg_container = nullptr;
    if (osg_foreign_window) {
        osg_container = QWidget::createWindowContainer(osg_foreign_window, viewer_host);
        osg_container->setFocusPolicy(Qt::StrongFocus);
        osg_container->show();
    } else {
        std::cout << "[gui] warning: failed to embed OSG native window into Qt container" << std::endl;
    }

    auto update_viewport_layout = [&]() {
        info_drawer_toggle_button.move(kInfoDrawerMargin, kInfoDrawerMargin);
        if (info_panel) {
            const int drawer_h = std::max(
                120, dashboard_window.height() - kInfoDrawerTop - kInfoDrawerBottom);
            const QPoint global_top_left = dashboard_window.mapToGlobal(QPoint(kInfoDrawerMargin, kInfoDrawerTop));
            info_panel->setGeometry(global_top_left.x(), global_top_left.y(), kInfoDrawerWidth, drawer_h);
            info_panel->setVisible(info_drawer_visible);
            if (info_drawer_visible) {
                info_panel->raise();
            }
        }
        info_drawer_toggle_button.raise();
        const QRect area = viewer_frame->contentsRect().adjusted(8, 8, -8, -8);
        if (area.width() <= 0 || area.height() <= 0) {
            return;
        }
        const int side_w = std::clamp(area.width() / 4, 180, 340);
        const QRect main_area(area.x(), area.y(), std::max(80, area.width() - side_w - 6), area.height());
        subcamera_panel->setGeometry(main_area.right() + 6, area.y(), side_w, area.height());

        const double target_aspect =
            std::tan(degToRad(camera_hfov_deg) * 0.5) / std::tan(degToRad(camera_vfov_deg) * 0.5);
        int w = main_area.width();
        int h = static_cast<int>(std::lround(static_cast<double>(w) / target_aspect));
        if (h > main_area.height()) {
            h = main_area.height();
            w = static_cast<int>(std::lround(static_cast<double>(h) * target_aspect));
        }
        const int x = main_area.x() + (main_area.width() - w) / 2;
        const int y = main_area.y() + (main_area.height() - h) / 2;
        viewer_host->setGeometry(x, y, w, h);
        if (osg_container) {
            osg_container->setGeometry(viewer_host->rect());
        }
        capture_label->move(main_area.x() + 8, main_area.y() + 8);
        viewer.getCamera()->setViewport(new osg::Viewport(0, 0, std::max(1, w), std::max(1, h)));
        viewer.getCamera()->setProjectionMatrixAsPerspective(camera_vfov_deg, target_aspect, 0.1, 10000.0);
        camera_module.updateViewports();
    };

    standalone_mvp::SettingsDialog settings_dialog(&dashboard_window);
    settings_dialog.setWindowFlag(Qt::Window, true);
    settings_dialog.setWindowFlag(Qt::WindowStaysOnTopHint, true);
    settings_dialog.setFromConfig(app_cfg);
    settings_dialog.setRestartHintVisible(true);
    settings_button.setAutoDefault(false);
    settings_button.setDefault(false);
    scene_editor_button.setAutoDefault(false);
    scene_editor_button.setDefault(false);
    QObject::connect(&settings_button, &QPushButton::pressed, [&settings_dialog]() {
        QMetaObject::invokeMethod(&settings_dialog, [&settings_dialog]() {
            settings_dialog.setWindowState((settings_dialog.windowState() & ~Qt::WindowMinimized) | Qt::WindowActive);
            settings_dialog.show();
            settings_dialog.raise();
            settings_dialog.activateWindow();
        }, Qt::QueuedConnection);
    });
    QObject::connect(&scene_editor_button, &QPushButton::pressed, [scene_editor_dialog]() {
        QMetaObject::invokeMethod(scene_editor_dialog, [scene_editor_dialog]() {
            scene_editor_dialog->setWindowState((scene_editor_dialog->windowState() & ~Qt::WindowMinimized) | Qt::WindowActive);
            scene_editor_dialog->show();
            scene_editor_dialog->raise();
            scene_editor_dialog->activateWindow();
        }, Qt::QueuedConnection);
    });

    enum class SonarWindowMode {
        Docked = 0,
        Floating,
    };
    SonarWindowMode sonar_window_mode = saved_sonar_docked_in_main ? SonarWindowMode::Docked : SonarWindowMode::Floating;
    auto move_sonar_workspace_to_dock = [&]() {
        if (!has_sonar_tabs || !sonar_workspace) {
            return;
        }
        sonar_workspace->setParent(sonar_dock_panel);
        sonar_dock_layout->addWidget(sonar_workspace, 1);
        sonar_window_mode = SonarWindowMode::Docked;
    };
    auto move_sonar_workspace_to_floating = [&]() {
        if (!has_sonar_tabs || !sonar_workspace) {
            return;
        }
        sonar_workspace->setParent(&sonar_floating_window);
        sonar_floating_layout->addWidget(sonar_workspace, 1);
        sonar_window_mode = SonarWindowMode::Floating;
    };
    bool sonar_dock_visible = has_sonar_tabs;
    auto enforce_fullscreen_for_docked_quad = [&]() {
        if (!has_sonar_tabs || !sonar_workspace) {
            return;
        }
        if (sonar_window_mode == SonarWindowMode::Docked &&
            sonar_workspace->layoutPreset() == DockWorkspace::LayoutPreset::Quad) {
            dashboard_window.showMaximized();
            dashboard_window.raise();
            dashboard_window.activateWindow();
        }
    };
    auto apply_main_split_ratio = [&]() {
        if (sonar_dock_visible) {
            // Default main/sonar ratio = 1.5 : 2.
            main_vertical_splitter->setSizes({1500, 2000});
        }
    };
    if (has_sonar_tabs && sonar_workspace && sonar_window_mode == SonarWindowMode::Floating) {
        move_sonar_workspace_to_floating();
        sonar_dock_panel->setVisible(false);
        sonar_floating_window.show();
        sonar_floating_window.raise();
    } else {
        sonar_dock_panel->setVisible(sonar_dock_visible);
    }
    sonar_dock_button.setText(sonar_dock_visible ? "Hide Sonar" : "Show Sonar");
    if (sonar_dock_visible) {
        apply_main_split_ratio();
        enforce_fullscreen_for_docked_quad();
    }
    sonar_dock_button.setEnabled(has_sonar_tabs);
    sonar_dock_panel->setContextMenuPolicy(Qt::CustomContextMenu);
    sonar_floating_window.setContextMenuPolicy(Qt::CustomContextMenu);
    auto show_sonar_mode_menu = [&](const QPoint& global_pos) {
        if (!has_sonar_tabs) {
            return;
        }
        QMenu menu;
        QAction* to_docked = menu.addAction(QStringLiteral("Dock Sonar In Main Window"));
        QAction* to_floating = menu.addAction(QStringLiteral("Pop Out Sonar Window"));
        to_docked->setEnabled(sonar_window_mode == SonarWindowMode::Floating);
        to_floating->setEnabled(sonar_window_mode == SonarWindowMode::Docked);
        QAction* selected = menu.exec(global_pos);
        if (!selected) {
            return;
        }
        if (selected == to_docked) {
            sonar_floating_window.hide();
            move_sonar_workspace_to_dock();
            sonar_dock_visible = true;
            sonar_dock_panel->setVisible(true);
            sonar_dock_button.setText("Hide Sonar");
            apply_main_split_ratio();
            enforce_fullscreen_for_docked_quad();
            return;
        }
        if (selected == to_floating) {
            move_sonar_workspace_to_floating();
            sonar_dock_visible = true;
            sonar_dock_panel->setVisible(false);
            main_vertical_splitter->setSizes({1, 0});
            sonar_floating_window.show();
            sonar_floating_window.raise();
            sonar_dock_button.setText("Hide Sonar");
            return;
        }
    };
    if (has_sonar_tabs && sonar_workspace) {
        sonar_workspace->setExtraContextMenuBuilder([&](QMenu& menu) {
            QAction* to_docked = menu.addAction(QStringLiteral("Dock Sonar In Main Window"));
            QAction* to_floating = menu.addAction(QStringLiteral("Pop Out Sonar Window"));
            to_docked->setEnabled(sonar_window_mode == SonarWindowMode::Floating);
            to_floating->setEnabled(sonar_window_mode == SonarWindowMode::Docked);
            QObject::connect(to_docked, &QAction::triggered, &dashboard_window, [&]() {
                sonar_floating_window.hide();
                move_sonar_workspace_to_dock();
                sonar_dock_visible = true;
                sonar_dock_panel->setVisible(true);
                sonar_dock_button.setText("Hide Sonar");
                apply_main_split_ratio();
                enforce_fullscreen_for_docked_quad();
            });
            QObject::connect(to_floating, &QAction::triggered, &dashboard_window, [&]() {
                move_sonar_workspace_to_floating();
                sonar_dock_visible = true;
                sonar_dock_panel->setVisible(false);
                main_vertical_splitter->setSizes({1, 0});
                sonar_floating_window.show();
                sonar_floating_window.raise();
                sonar_dock_button.setText("Hide Sonar");
            });
        });
    }
    QObject::connect(sonar_dock_panel, &QWidget::customContextMenuRequested, [&show_sonar_mode_menu, sonar_dock_panel](const QPoint& p) {
        show_sonar_mode_menu(sonar_dock_panel->mapToGlobal(p));
    });
    QObject::connect(&sonar_floating_window, &QWidget::customContextMenuRequested, [&show_sonar_mode_menu, &sonar_floating_window](const QPoint& p) {
        show_sonar_mode_menu(sonar_floating_window.mapToGlobal(p));
    });
    if (has_sonar_tabs && sonar_workspace) {
        QObject::connect(sonar_workspace, &DockWorkspace::layoutPresetChanged, &dashboard_window, [&]() {
            enforce_fullscreen_for_docked_quad();
        });
    }
    class HideOnCloseFilter final : public QObject {
    public:
        HideOnCloseFilter(bool* visible_flag, QPushButton* toggle_btn, QObject* parent = nullptr)
            : QObject(parent), visible_flag_(visible_flag), toggle_btn_(toggle_btn) {}
    protected:
        bool eventFilter(QObject* watched, QEvent* event) override {
            if (event->type() == QEvent::Close) {
                if (auto* w = qobject_cast<QWidget*>(watched)) {
                    w->hide();
                }
                if (visible_flag_) {
                    *visible_flag_ = false;
                }
                if (toggle_btn_) {
                    toggle_btn_->setText("Show Sonar");
                }
                event->ignore();
                return true;
            }
            return QObject::eventFilter(watched, event);
        }
    private:
        bool* visible_flag_ = nullptr;
        QPushButton* toggle_btn_ = nullptr;
    };
    auto* hide_on_close_filter = new HideOnCloseFilter(&sonar_dock_visible, &sonar_dock_button, &dashboard_window);
    sonar_floating_window.installEventFilter(hide_on_close_filter);
    QObject::connect(&sonar_dock_button, &QPushButton::clicked, [&]() {
        if (!has_sonar_tabs) {
            return;
        }
        sonar_dock_visible = !sonar_dock_visible;
        if (sonar_window_mode == SonarWindowMode::Docked) {
            sonar_dock_panel->setVisible(sonar_dock_visible);
            if (sonar_dock_visible) {
                apply_main_split_ratio();
            } else {
                main_vertical_splitter->setSizes({1, 0});
            }
        } else {
            if (sonar_dock_visible) {
                sonar_floating_window.show();
                sonar_floating_window.raise();
            } else {
                sonar_floating_window.hide();
            }
        }
        sonar_dock_button.setText(sonar_dock_visible ? "Hide Sonar" : "Show Sonar");
    });

    bool restart_requested = false;
    KeyForwardFilter key_filter(commanded_pose, pose_step_xy, pose_step_yaw, pose_step_pitch, pose_step_z, trackball.get());
    dashboard_window.installEventFilter(&key_filter);
    viewer_frame->installEventFilter(&key_filter);
    viewer_host->installEventFilter(&key_filter);
    if (osg_container) {
        osg_container->installEventFilter(&key_filter);
    }
    dashboard_window.setFocusPolicy(Qt::StrongFocus);
    fls_module.connectWidgetSignals();
    mbes_module.connectWidgetSignals();
    for (auto& extra : extra_fls_modules_rt) extra->connectWidgetSignals();
    for (auto& extra : extra_mbes_modules_rt) extra->connectWidgetSignals();
    sss_module.connectStripSignals();
    for (auto& extra : extra_sss_modules_rt) extra->connectStripSignals();

    viewer_frame->setFocusPolicy(Qt::StrongFocus);
    viewer_host->setFocusPolicy(Qt::StrongFocus);

    auto persist_runtime_to_config = [&]() {
        if (!restart_requested) {
            app_cfg.pose.x = commanded_pose.position.x();
            app_cfg.pose.y = commanded_pose.position.y();
            app_cfg.pose.z = commanded_pose.position.z();
            app_cfg.pose.step_xy = pose_step_xy;
            app_cfg.pose.step_z = pose_step_z;
            app_cfg.pose.step_yaw_deg = radToDeg(pose_step_yaw);
            app_cfg.pose.enable_auto_pose = enable_auto_pose;
            app_cfg.camera_system.main_camera.yaw_deg = radToDeg(commanded_pose.yaw);
            app_cfg.camera_system.main_camera.pitch_deg = radToDeg(commanded_pose.pitch);
            primary_fls_cfg.range_m = range_m;
            primary_fls_cfg.gain = gain;
            primary_fls_cfg.bin_count = static_cast<int>(bin_count);
            primary_fls_cfg.beam_count = static_cast<int>(beam_count);
            primary_fls_cfg.beam_width_deg = beam_width_deg;
            primary_fls_cfg.beam_height_deg = beam_height_deg;
            primary_fls_cfg.max_fps = sonar_max_fps;
            primary_fls_cfg.viewer_max_fps = viewer_max_fps;
            primary_fls_cfg.sound_speed_mps = sonar_sound_speed_mps;
            app_cfg.environment = global_env_cfg;
            primary_mbes_cfg.range_m = mbes_range_m;
            primary_mbes_cfg.gain = mbes_gain;
            primary_mbes_cfg.beam_count = static_cast<int>(mbes_beam_count);
            primary_mbes_cfg.bin_count = static_cast<int>(mbes_bin_count);
            primary_mbes_cfg.beam_width_deg = mbes_beam_width_deg;
            primary_mbes_cfg.beam_height_deg = mbes_beam_height_deg;
            app_cfg.camera_system.main_camera.horizontal_fov_deg = camera_hfov_deg;
            app_cfg.camera_system.main_camera.vertical_fov_deg = camera_vfov_deg;
            app_cfg.camera = app_cfg.camera_system.main_camera;
            primary_sss_cfg.range_m = static_cast<double>(sss_module.runtime_range_m);
            primary_sss_cfg.gain = static_cast<double>(sss_module.runtime_gain);
            if (primary_fls_module_idx >= 0 && primary_fls_module_idx < static_cast<int>(app_cfg.sonar_modules.size())) {
                auto& mod = app_cfg.sonar_modules[primary_fls_module_idx];
                mod.fls_config = primary_fls_cfg;
                const auto& fls_pc = fls_module.point_cloud_cfg_runtime;
                mod.point_cloud_config.enabled = fls_pc.enabled;
                mod.point_cloud_config.range_m = fls_pc.range_m;
                mod.point_cloud_config.frequency_khz = fls_pc.frequency_khz;
                mod.point_cloud_config.bandwidth_khz = fls_pc.bandwidth_khz;
                mod.point_cloud_config.horizontal_angle_resolution_deg = fls_pc.horizontal_angle_resolution_deg;
                mod.point_cloud_config.vertical_angle_resolution_deg = fls_pc.vertical_angle_resolution_deg;
                mod.point_cloud_config.horizontal_fov_deg = fls_pc.horizontal_fov_deg;
                mod.point_cloud_config.vertical_fov_deg = fls_pc.vertical_fov_deg;
                mod.point_cloud_config.max_point_count = static_cast<int>(fls_pc.max_point_count);
                mod.point_cloud_config.palette_index = fls_pc.palette_index;
                mod.point_cloud_config.show_coordinate_overlay = fls_pc.show_coordinate_overlay;
                mod.point_cloud_config.tcp_output_enabled = fls_pc.tcp_output_enabled;
                mod.point_cloud_config.file_output_enabled = fls_pc.file_output_enabled;
                mod.point_cloud_config.tcp_host = QString::fromStdString(fls_pc.tcp_host);
                mod.point_cloud_config.tcp_port = static_cast<int>(fls_pc.tcp_port);
            }
            if (primary_mbes_module_idx >= 0 && primary_mbes_module_idx < static_cast<int>(app_cfg.sonar_modules.size())) {
                auto& mod = app_cfg.sonar_modules[primary_mbes_module_idx];
                mod.mbes_config = primary_mbes_cfg;
                const auto& mbes_pc = mbes_module.bottom_cfg_runtime;
                mod.point_cloud_config.enabled = mbes_pc.enabled;
                mod.point_cloud_config.range_m = mbes_pc.range_m;
                mod.point_cloud_config.frequency_khz = mbes_pc.frequency_khz;
                mod.point_cloud_config.bandwidth_khz = mbes_pc.bandwidth_khz;
                mod.point_cloud_config.horizontal_angle_resolution_deg = mbes_pc.horizontal_angle_resolution_deg;
                mod.point_cloud_config.vertical_angle_resolution_deg = mbes_pc.vertical_angle_resolution_deg;
                mod.point_cloud_config.horizontal_fov_deg = mbes_pc.horizontal_fov_deg;
                mod.point_cloud_config.vertical_fov_deg = mbes_pc.vertical_fov_deg;
                mod.point_cloud_config.max_point_count = static_cast<int>(mbes_pc.max_point_count);
                mod.point_cloud_config.palette_index = mbes_pc.palette_index;
                mod.point_cloud_config.show_coordinate_overlay = mbes_pc.show_coordinate_overlay;
                mod.point_cloud_config.tcp_output_enabled = mbes_pc.tcp_output_enabled;
                mod.point_cloud_config.file_output_enabled = mbes_pc.file_output_enabled;
                mod.point_cloud_config.tcp_host = QString::fromStdString(mbes_pc.tcp_host);
                mod.point_cloud_config.tcp_port = static_cast<int>(mbes_pc.tcp_port);
            }
            if (primary_sss_module_idx >= 0 && primary_sss_module_idx < static_cast<int>(app_cfg.sonar_modules.size())) {
                app_cfg.sonar_modules[primary_sss_module_idx].sss_config = primary_sss_cfg;
            }
            app_cfg.sonar_window_docked_in_main = (sonar_window_mode == SonarWindowMode::Docked);
            if (sonar_workspace) {
                switch (sonar_workspace->layoutPreset()) {
                case DockWorkspace::LayoutPreset::Horizontal:
                    app_cfg.sonar_workspace_split_layout = "horizontal";
                    break;
                case DockWorkspace::LayoutPreset::Vertical:
                    app_cfg.sonar_workspace_split_layout = "vertical";
                    break;
                case DockWorkspace::LayoutPreset::Quad:
                    app_cfg.sonar_workspace_split_layout = "quad";
                    break;
                case DockWorkspace::LayoutPreset::Single:
                default:
                    app_cfg.sonar_workspace_split_layout = "single";
                    break;
                }
            }
            for (const auto& extra_sss : extra_sss_modules_rt) {
                if (!extra_sss) {
                    continue;
                }
                const QString target_name = extra_sss->module_cfg.name;
                for (std::size_t i = 0; i < app_cfg.sonar_modules.size(); ++i) {
                    if (static_cast<int>(i) == primary_sss_module_idx) {
                        continue;
                    }
                    auto& mod = app_cfg.sonar_modules[i];
                    if (mod.type != standalone_mvp::SonarModuleType::SSS) {
                        continue;
                    }
                    if (mod.name == target_name) {
                        mod.sss_config = extra_sss->module_cfg.sss_config;
                        mod.sss_camera_slot1 = extra_sss->module_cfg.sss_camera_slot1;
                        mod.sss_camera_slot2 = extra_sss->module_cfg.sss_camera_slot2;
                        break;
                    }
                }
            }
        }
        config_store.save(app_cfg);
    };

    QObject::connect(settings_dialog.applyButton(), &QPushButton::clicked, [&]() {
        app_cfg = settings_dialog.configFromUi();
        global_env_cfg = app_cfg.environment;
        sonar_sound_speed_mps = global_env_cfg.sound_speed_mps;
        fls_module.setEnvironmentConfig(global_env_cfg);
        mbes_module.setEnvironmentConfig(global_env_cfg);
        for (auto& extra : extra_fls_modules_rt) extra->setEnvironmentConfig(global_env_cfg);
        for (auto& extra : extra_mbes_modules_rt) extra->setEnvironmentConfig(global_env_cfg);
        restart_requested = true;
        settings_dialog.setRestartHintVisible(true);
        persist_runtime_to_config();
        viewer.setDone(true);
    });
    QObject::connect(settings_dialog.saveButton(), &QPushButton::clicked, [&]() {
        app_cfg = settings_dialog.configFromUi();
        global_env_cfg = app_cfg.environment;
        sonar_sound_speed_mps = global_env_cfg.sound_speed_mps;
        fls_module.setEnvironmentConfig(global_env_cfg);
        mbes_module.setEnvironmentConfig(global_env_cfg);
        for (auto& extra : extra_fls_modules_rt) extra->setEnvironmentConfig(global_env_cfg);
        for (auto& extra : extra_mbes_modules_rt) extra->setEnvironmentConfig(global_env_cfg);
        restart_requested = true;
        settings_dialog.setRestartHintVisible(true);
        persist_runtime_to_config();
        settings_dialog.accept();
        viewer.setDone(true);
    });
    dashboard_window.showMaximized();
    apply_main_split_ratio();
    QTimer::singleShot(0, &dashboard_window, [apply_main_split_ratio]() { apply_main_split_ratio(); });
    update_viewport_layout();
    auto compute_sonar_target_fps = [&]() {
        if (!sonar) {
            return 0.0;
        }
        const double range_now = std::max(0.1, static_cast<double>(sonar->getRange()));
        const double physics_fps = std::max(0.1, sonar_sound_speed_mps / (2.0 * range_now));
        return std::min(sonar_max_fps, physics_fps);
    };
    auto compute_mbes_target_fps = [&]() {
        if (!mbes_sonar) {
            return 0.0;
        }
        const double range_now = std::max(0.1, static_cast<double>(mbes_sonar->getRange()));
        const double physics_fps = std::max(0.1, sonar_sound_speed_mps / (2.0 * range_now));
        return std::min(sonar_max_fps, physics_fps);
    };
    auto last_sonar_tick = std::chrono::steady_clock::now();
    auto prev_sonar_tick_for_fps = std::chrono::steady_clock::time_point();
    double sonar_actual_fps = 0.0;
    double sonar_target_fps = compute_sonar_target_fps();
    auto last_mbes_tick = std::chrono::steady_clock::now();
    double mbes_target_fps = compute_mbes_target_fps();
    auto compute_point_cloud_target_fps = [&]() {
        return std::clamp(sonar_max_fps, 1.0, 30.0);
    };
    auto last_point_cloud_tick = std::chrono::steady_clock::now();
    double point_cloud_target_fps = compute_point_cloud_target_fps();
    auto last_viewer_frame_tick = std::chrono::steady_clock::now();
    const QString app_path = QCoreApplication::applicationFilePath();
    auto restart_if_requested = [&]() {
        if (!restart_requested) {
            return;
        }
        QStringList args;
        const QString project_path = config_store.path().trimmed();
        if (!project_path.isEmpty()) {
            args << QStringLiteral("--project") << QDir::toNativeSeparators(project_path);
        }
        std::cout << "[gui] restarting application"
                  << " project=" << (project_path.isEmpty() ? "<default>" : project_path.toStdString())
                  << std::endl;
        QProcess::startDetached(app_path, args);
    };
    if (max_frames > 0) {
        int frames = 0;
        while (!viewer.done() && frames < max_frames) {
            if (!dashboard_window.isVisible()) {
                viewer.setDone(true);
                break;
            }
            if (third_viewer && third_viewer->done()) {
                third_viewer.reset();
            }
            QApplication::processEvents(QEventLoop::AllEvents, 4);
            std::cout << "[gui] frame " << frames << " begin" << std::endl;
            range_m = fls_module.runtime_range_m;
            gain = fls_module.runtime_gain;
            mbes_range_m = mbes_module.runtime_range_m;
            mbes_gain = mbes_module.runtime_gain;
            fls_module.consumePointCloudUiConfig();
            for (auto& extra : extra_fls_modules_rt) {
                extra->consumePointCloudUiConfig();
            }
            mbes_module.consumePointCloudUiConfig();
            for (auto& extra : extra_mbes_modules_rt) {
                extra->consumePointCloudUiConfig();
            }
            bool pose_cmd_changed = false;
            if (!scripted_keys.empty() && (frames % 20) == 0) {
                const int scripted = scripted_keys[scripted_key_index % scripted_keys.size()];
                ++scripted_key_index;
                if (applyControlKey(scripted, commanded_pose, pose_step_xy, pose_step_yaw, pose_step_pitch, pose_step_z)) {
                    pose_cmd_changed = true;
                    syncTrackballToPose(trackball.get(), commanded_pose);
                    std::cout << "[gui] pose_cmd scripted=" << static_cast<char>(scripted)
                              << " xyz=(" << commanded_pose.position.x() << "," << commanded_pose.position.y() << "," << commanded_pose.position.z()
                              << ") yaw=" << commanded_pose.yaw << " pitch=" << commanded_pose.pitch << std::endl;
                }
            }
            const double t = static_cast<double>(frames) * 0.01;
            PoseState active_pose = commanded_pose;
            if (enable_auto_pose) {
                active_pose.position = auto_pose_origin.position +
                    Eigen::Vector3d(2.0 * std::sin(t), 1.2 * std::cos(t * 0.7), 0.6 * std::sin(t * 0.5));
                active_pose.yaw = auto_pose_origin.yaw + 0.35 * std::sin(0.5 * t);
                active_pose.pitch = auto_pose_origin.pitch + 0.18 * std::cos(0.4 * t);
            }
            syncTrackballToPose(trackball.get(), active_pose);
            setCameraViewFromPose(viewer.getCamera(), active_pose);
            camera_module.updateViews(active_pose.position, active_pose.yaw, active_pose.pitch);
            fls_module.setPointCloudRenderBlocked(true);
            for (auto& extra : extra_fls_modules_rt) extra->setPointCloudRenderBlocked(true);
            mbes_module.setPointCloudRenderBlocked(true);
            for (auto& extra : extra_mbes_modules_rt) extra->setPointCloudRenderBlocked(true);
            if (viewer_max_fps > 0.0) {
                const auto now = std::chrono::steady_clock::now();
                const double frame_period_s = 1.0 / std::max(1.0, viewer_max_fps);
                const double elapsed_s = std::chrono::duration<double>(now - last_viewer_frame_tick).count();
                if (elapsed_s < frame_period_s) {
                    std::this_thread::sleep_for(std::chrono::duration<double>(frame_period_s - elapsed_s));
                }
                last_viewer_frame_tick = std::chrono::steady_clock::now();
            }
            viewer.frame();
            if (third_viewer) {
                third_viewer->frame();
            }
            if ((frames % 2) == 0) {
                camera_module.updateWidgets();
            }
            if (!scene_edit_pauses_sonar.load() && (frames % side_scan_update_stride) == 0) {
                sss_module.tickFromCameraRuntimes(sub_cameras, global_env_cfg, active_pose.position, frames, image_update_stride);
            }
            fls_module.setPointCloudRenderBlocked(false);
            for (auto& extra : extra_fls_modules_rt) extra->setPointCloudRenderBlocked(false);
            mbes_module.setPointCloudRenderBlocked(false);
            for (auto& extra : extra_mbes_modules_rt) extra->setPointCloudRenderBlocked(false);
            bool do_sonar_tick = false;
            if (enable_sonar_tick && !scene_edit_pauses_sonar.load()) {
                sonar_target_fps = compute_sonar_target_fps();
                const double period_s = 1.0 / std::max(0.1, sonar_target_fps);
                const auto now = std::chrono::steady_clock::now();
                const double elapsed_s = std::chrono::duration<double>(now - last_sonar_tick).count();
                if (elapsed_s >= period_s) {
                    do_sonar_tick = true;
                    last_sonar_tick = now;
                    if (prev_sonar_tick_for_fps.time_since_epoch().count() != 0) {
                        const double dt = std::chrono::duration<double>(now - prev_sonar_tick_for_fps).count();
                        if (dt > 1e-6) {
                            sonar_actual_fps = 1.0 / dt;
                        }
                    }
                    prev_sonar_tick_for_fps = now;
                }
            }
            bool do_mbes_tick = false;
            if (enable_sonar_tick && !scene_edit_pauses_sonar.load()) {
                mbes_target_fps = compute_mbes_target_fps();
                const double period_s = 1.0 / std::max(0.1, mbes_target_fps);
                const auto now = std::chrono::steady_clock::now();
                const double elapsed_s = std::chrono::duration<double>(now - last_mbes_tick).count();
                if (elapsed_s >= period_s) {
                    do_mbes_tick = true;
                    last_mbes_tick = now;
                }
            }
            bool do_point_cloud_tick = false;
            if (!scene_edit_pauses_sonar.load() && fls_module.point_cloud_runtime_enabled &&
                fls_module.point_cloud_cfg_runtime.enabled) {
                point_cloud_target_fps = compute_point_cloud_target_fps();
                const double period_s = 1.0 / std::max(0.1, point_cloud_target_fps);
                const auto now = std::chrono::steady_clock::now();
                const double elapsed_s = std::chrono::duration<double>(now - last_point_cloud_tick).count();
                if (elapsed_s >= period_s) {
                    do_point_cloud_tick = true;
                    last_point_cloud_tick = now;
                }
            }
            if (do_sonar_tick && enable_2d_fls && sonar) {
                const Eigen::Affine3d pose = bodyAffineFromCameraViewMatrix(viewer.getCamera()->getViewMatrix());
                sonar_types_v2::samples::Sonar sonar_sample;
                if (fls_module.tick(pose, frames, image_update_stride, &sonar_sample)) {
                    prev_bins = sonar_sample.bins;
                    if (enable_opencv_window && (frames % image_update_stride) == 0) {
                        const int overlay_range_m =
                            static_cast<int>(std::lround(static_cast<double>(sonar->getRange())));
                        cv::Mat sonar_vis = standalone_mvp::renderSonarLikeSonarWidget(
                            sonar_sample, sonar_plot_w, sonar_plot_h, overlay_range_m);
                        if (!sonar_vis.empty()) {
                            cv::imshow("sonar_image", sonar_vis);
                        }
                    }
                }
            }
            if (do_sonar_tick) {
                for (auto& extra : extra_fls_modules_rt) {
                    if (!extra->sonar || !extra->module_cfg.fls_config.enable_2d_fls) {
                        continue;
                    }
                    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
                    if (!poseFromCameraBinding(extra->module_cfg.camera_binding.toStdString(), pose)) {
                        continue;
                    }
                    extra->tick(pose, frames, image_update_stride, nullptr);
                }
            }
            if (do_mbes_tick && mbes_sonar) {
                const auto mbes_it = std::find_if(sub_cameras.begin(), sub_cameras.end(), [&](const SubCameraRuntime& sc) {
                    return sc.name == mbes_camera_name;
                });
                if (mbes_it != sub_cameras.end()) {
                    const Eigen::Affine3d mbes_pose = bodyAffineFromCameraViewMatrix(mbes_it->camera->getViewMatrix());
                    sonar_types_v2::samples::Sonar mbes_sample;
                    if (mbes_module.tick(mbes_pose, frames, image_update_stride, &mbes_sample)) {
                        mbes_module.tickPointCloud(mbes_pose, (frames % image_update_stride) == 0);
                    }
                }
            }
            if (do_mbes_tick) {
                for (auto& extra : extra_mbes_modules_rt) {
                    if (!extra->sonar || !extra->module_cfg.mbes_config.enable_2d_fls) {
                        continue;
                    }
                    Eigen::Affine3d mbes_pose = Eigen::Affine3d::Identity();
                    if (!poseFromCameraBinding(extra->module_cfg.camera_binding.toStdString(), mbes_pose)) {
                        continue;
                    }
                    extra->tick(mbes_pose, frames, image_update_stride, nullptr);
                    extra->tickPointCloud(mbes_pose, (frames % image_update_stride) == 0);
                }
            }
            if ((frames % side_scan_update_stride) == 0) {
                for (auto& extra : extra_sss_modules_rt) {
                    extra->tickFromCameraRuntimes(sub_cameras, global_env_cfg, active_pose.position, frames, image_update_stride);
                }
            }
            if (do_point_cloud_tick) {
                const Eigen::Affine3d pose = bodyAffineFromCameraViewMatrix(viewer.getCamera()->getViewMatrix());
                fls_module.tickPointCloud(pose);
                for (auto& extra : extra_fls_modules_rt) {
                    Eigen::Affine3d extra_pose = Eigen::Affine3d::Identity();
                    if (!poseFromCameraBinding(extra->module_cfg.camera_binding.toStdString(), extra_pose)) {
                        continue;
                    }
                    extra->tickPointCloud(extra_pose);
                }
            }
            standalone_mvp::RuntimeInfoSnapshot snapshot;
            snapshot.world = app_cfg.scene.world;
            snapshot.x = active_pose.position.x();
            snapshot.y = active_pose.position.y();
            snapshot.z = active_pose.position.z();
            snapshot.yaw_deg = radToDeg(active_pose.yaw);
            snapshot.pitch_deg = radToDeg(active_pose.pitch);
            if (sonar) {
                snapshot.range_m = sonar->getRange();
                snapshot.gain = sonar->getGain();
                snapshot.beam_count = static_cast<int>(sonar->getSonarBeamCount());
                snapshot.bin_count = static_cast<int>(sonar->getSonarBinCount());
                snapshot.beam_width_deg = radToDeg(sonar->getSonarBeamWidth().getRad());
                snapshot.beam_height_deg = radToDeg(sonar->getSonarBeamHeight().getRad());
            } else {
                snapshot.range_m = range_m;
                snapshot.gain = gain;
                snapshot.beam_count = static_cast<int>(beam_count);
                snapshot.bin_count = static_cast<int>(bin_count);
                snapshot.beam_width_deg = beam_width_deg;
                snapshot.beam_height_deg = beam_height_deg;
            }
            snapshot.center_frequency_khz = primary_fls_cfg.center_frequency_khz;
            snapshot.step_xy = pose_step_xy;
            snapshot.step_z = pose_step_z;
            snapshot.auto_pose = enable_auto_pose;
            snapshot.resolution = static_cast<int>(static_cast<float>(snapshot.bin_count) * kResolutionConstant);
            snapshot.enable_reverb = global_env_cfg.enable_reverb;
            snapshot.enable_speckle = global_env_cfg.enable_speckle;
            snapshot.enable_attenuation = global_env_cfg.enable_attenuation;
            snapshot.attenuation_frequency_khz = global_env_cfg.attenuation_frequency_khz;
            snapshot.temperature_c = global_env_cfg.temperature_c;
            snapshot.salinity_ppt = global_env_cfg.salinity_ppt;
            snapshot.acidity_ph = global_env_cfg.acidity_ph;
            snapshot.sonar_actual_fps = sonar_actual_fps;
            snapshot.sonar_target_fps = sonar_target_fps;
            snapshot.sonar_max_fps = sonar_max_fps;
            if (info_panel) {
                info_panel->updateSnapshot(snapshot);
            }
            update_viewport_layout();
            QApplication::processEvents(QEventLoop::AllEvents, 2);
            if (enable_opencv_window) {
                std::cout << "[gui] frame " << frames << " image update begin" << std::endl;
                const int key = cv::waitKey(1);
                if (key >= 0 && applyControlKey(key, commanded_pose, pose_step_xy, pose_step_yaw, pose_step_pitch, pose_step_z)) {
                    pose_cmd_changed = true;
                    syncTrackballToPose(trackball.get(), commanded_pose);
                    std::cout << "[gui] pose_cmd key=" << static_cast<char>(key)
                              << " xyz=(" << commanded_pose.position.x() << "," << commanded_pose.position.y() << "," << commanded_pose.position.z()
                              << ") yaw=" << commanded_pose.yaw << " pitch=" << commanded_pose.pitch << std::endl;
                }
                std::cout << "[gui] frame " << frames << " image update end" << std::endl;
            }
            std::cout << "[gui] frame " << frames << " end" << std::endl;
            ++frames;
        }
        std::cout << "GUI FRAMES " << frames << std::endl;
        if (enable_opencv_window) {
            cv::destroyWindow("sonar_image");
        }
        persist_runtime_to_config();
        restart_if_requested();
        std::cout << "[gui] exit(max_frames)" << std::endl;
        return 0;
    }

    int frames = 0;
    while (!viewer.done()) {
        if (!dashboard_window.isVisible()) {
            viewer.setDone(true);
            break;
        }
        if (third_viewer && third_viewer->done()) {
            third_viewer.reset();
        }
        QApplication::processEvents(QEventLoop::AllEvents, 4);
        std::cout << "[gui] frame " << frames << " begin" << std::endl;
        range_m = fls_module.runtime_range_m;
        gain = fls_module.runtime_gain;
        mbes_range_m = mbes_module.runtime_range_m;
        mbes_gain = mbes_module.runtime_gain;
        fls_module.consumePointCloudUiConfig();
        for (auto& extra : extra_fls_modules_rt) {
            extra->consumePointCloudUiConfig();
        }
        mbes_module.consumePointCloudUiConfig();
        for (auto& extra : extra_mbes_modules_rt) {
            extra->consumePointCloudUiConfig();
        }
        bool pose_cmd_changed = false;
        if (!scripted_keys.empty() && (frames % 20) == 0) {
            const int scripted = scripted_keys[scripted_key_index % scripted_keys.size()];
            ++scripted_key_index;
            if (applyControlKey(scripted, commanded_pose, pose_step_xy, pose_step_yaw, pose_step_pitch, pose_step_z)) {
                pose_cmd_changed = true;
                syncTrackballToPose(trackball.get(), commanded_pose);
                std::cout << "[gui] pose_cmd scripted=" << static_cast<char>(scripted)
                          << " xyz=(" << commanded_pose.position.x() << "," << commanded_pose.position.y() << "," << commanded_pose.position.z()
                          << ") yaw=" << commanded_pose.yaw << " pitch=" << commanded_pose.pitch << std::endl;
            }
        }
        const double t = static_cast<double>(frames) * 0.01;
        PoseState active_pose = commanded_pose;
        if (enable_auto_pose) {
            active_pose.position = auto_pose_origin.position +
                Eigen::Vector3d(2.0 * std::sin(t), 1.2 * std::cos(t * 0.7), 0.6 * std::sin(t * 0.5));
            active_pose.yaw = auto_pose_origin.yaw + 0.35 * std::sin(0.5 * t);
            active_pose.pitch = auto_pose_origin.pitch + 0.18 * std::cos(0.4 * t);
        }
        syncTrackballToPose(trackball.get(), active_pose);
        setCameraViewFromPose(viewer.getCamera(), active_pose);
        camera_module.updateViews(active_pose.position, active_pose.yaw, active_pose.pitch);
        fls_module.setPointCloudRenderBlocked(true);
        for (auto& extra : extra_fls_modules_rt) extra->setPointCloudRenderBlocked(true);
        mbes_module.setPointCloudRenderBlocked(true);
        for (auto& extra : extra_mbes_modules_rt) extra->setPointCloudRenderBlocked(true);
        if (viewer_max_fps > 0.0) {
            const auto now = std::chrono::steady_clock::now();
            const double frame_period_s = 1.0 / std::max(1.0, viewer_max_fps);
            const double elapsed_s = std::chrono::duration<double>(now - last_viewer_frame_tick).count();
            if (elapsed_s < frame_period_s) {
                std::this_thread::sleep_for(std::chrono::duration<double>(frame_period_s - elapsed_s));
            }
            last_viewer_frame_tick = std::chrono::steady_clock::now();
        }
        viewer.frame();
        if (third_viewer) {
            third_viewer->frame();
        }
        if ((frames % 2) == 0) {
            camera_module.updateWidgets();
        }
        if (!scene_edit_pauses_sonar.load() && (frames % side_scan_update_stride) == 0) {
            sss_module.tickFromCameraRuntimes(sub_cameras, global_env_cfg, active_pose.position, frames, image_update_stride);
        }
        fls_module.setPointCloudRenderBlocked(false);
        for (auto& extra : extra_fls_modules_rt) extra->setPointCloudRenderBlocked(false);
        mbes_module.setPointCloudRenderBlocked(false);
        for (auto& extra : extra_mbes_modules_rt) extra->setPointCloudRenderBlocked(false);
        bool do_sonar_tick = false;
        if (enable_sonar_tick && !scene_edit_pauses_sonar.load()) {
            sonar_target_fps = compute_sonar_target_fps();
            const double period_s = 1.0 / std::max(0.1, sonar_target_fps);
            const auto now = std::chrono::steady_clock::now();
            const double elapsed_s = std::chrono::duration<double>(now - last_sonar_tick).count();
            if (elapsed_s >= period_s) {
                do_sonar_tick = true;
                last_sonar_tick = now;
                if (prev_sonar_tick_for_fps.time_since_epoch().count() != 0) {
                    const double dt = std::chrono::duration<double>(now - prev_sonar_tick_for_fps).count();
                    if (dt > 1e-6) {
                        sonar_actual_fps = 1.0 / dt;
                    }
                }
                prev_sonar_tick_for_fps = now;
            }
        }
        bool do_mbes_tick = false;
        if (enable_sonar_tick && !scene_edit_pauses_sonar.load()) {
            mbes_target_fps = compute_mbes_target_fps();
            const double period_s = 1.0 / std::max(0.1, mbes_target_fps);
            const auto now = std::chrono::steady_clock::now();
            const double elapsed_s = std::chrono::duration<double>(now - last_mbes_tick).count();
            if (elapsed_s >= period_s) {
                do_mbes_tick = true;
                last_mbes_tick = now;
            }
        }
        bool do_point_cloud_tick = false;
        if (!scene_edit_pauses_sonar.load() && fls_module.point_cloud_runtime_enabled &&
            fls_module.point_cloud_cfg_runtime.enabled) {
            point_cloud_target_fps = compute_point_cloud_target_fps();
            const double period_s = 1.0 / std::max(0.1, point_cloud_target_fps);
            const auto now = std::chrono::steady_clock::now();
            const double elapsed_s = std::chrono::duration<double>(now - last_point_cloud_tick).count();
            if (elapsed_s >= period_s) {
                do_point_cloud_tick = true;
                last_point_cloud_tick = now;
            }
        }
        if (do_sonar_tick && enable_2d_fls && sonar) {
            const Eigen::Affine3d pose = bodyAffineFromCameraViewMatrix(viewer.getCamera()->getViewMatrix());
            sonar_types_v2::samples::Sonar sonar_sample;
            if (fls_module.tick(pose, frames, image_update_stride, &sonar_sample)) {
                prev_bins = sonar_sample.bins;
                if (enable_opencv_window && (frames % image_update_stride) == 0) {
                    const int overlay_range_m =
                        static_cast<int>(std::lround(static_cast<double>(sonar->getRange())));
                    cv::Mat sonar_vis = standalone_mvp::renderSonarLikeSonarWidget(
                        sonar_sample, sonar_plot_w, sonar_plot_h, overlay_range_m);
                    if (!sonar_vis.empty()) {
                        cv::imshow("sonar_image", sonar_vis);
                    }
                }
            }
        }
        if (do_sonar_tick) {
            for (auto& extra : extra_fls_modules_rt) {
                if (!extra->sonar || !extra->module_cfg.fls_config.enable_2d_fls) {
                    continue;
                }
                Eigen::Affine3d pose = Eigen::Affine3d::Identity();
                if (!poseFromCameraBinding(extra->module_cfg.camera_binding.toStdString(), pose)) {
                    continue;
                }
                extra->tick(pose, frames, image_update_stride, nullptr);
            }
        }
        if (do_mbes_tick && mbes_sonar) {
            const auto mbes_it = std::find_if(sub_cameras.begin(), sub_cameras.end(), [&](const SubCameraRuntime& sc) {
                return sc.name == mbes_camera_name;
            });
            if (mbes_it != sub_cameras.end()) {
                const Eigen::Affine3d mbes_pose = bodyAffineFromCameraViewMatrix(mbes_it->camera->getViewMatrix());
                sonar_types_v2::samples::Sonar mbes_sample;
                if (mbes_module.tick(mbes_pose, frames, image_update_stride, &mbes_sample)) {
                    mbes_module.tickPointCloud(mbes_pose, (frames % image_update_stride) == 0);
                }
            }
        }
        if (do_mbes_tick) {
            for (auto& extra : extra_mbes_modules_rt) {
                if (!extra->sonar || !extra->module_cfg.mbes_config.enable_2d_fls) {
                    continue;
                }
                Eigen::Affine3d mbes_pose = Eigen::Affine3d::Identity();
                if (!poseFromCameraBinding(extra->module_cfg.camera_binding.toStdString(), mbes_pose)) {
                    continue;
                }
                extra->tick(mbes_pose, frames, image_update_stride, nullptr);
                extra->tickPointCloud(mbes_pose, (frames % image_update_stride) == 0);
            }
        }
        if ((frames % side_scan_update_stride) == 0) {
            for (auto& extra : extra_sss_modules_rt) {
                extra->tickFromCameraRuntimes(sub_cameras, global_env_cfg, active_pose.position, frames, image_update_stride);
            }
        }
        if (do_point_cloud_tick) {
            const Eigen::Affine3d pose = bodyAffineFromCameraViewMatrix(viewer.getCamera()->getViewMatrix());
            fls_module.tickPointCloud(pose);
            for (auto& extra : extra_fls_modules_rt) {
                Eigen::Affine3d extra_pose = Eigen::Affine3d::Identity();
                if (!poseFromCameraBinding(extra->module_cfg.camera_binding.toStdString(), extra_pose)) {
                    continue;
                }
                extra->tickPointCloud(extra_pose);
            }
        }
        standalone_mvp::RuntimeInfoSnapshot snapshot;
        snapshot.world = app_cfg.scene.world;
        snapshot.x = active_pose.position.x();
        snapshot.y = active_pose.position.y();
        snapshot.z = active_pose.position.z();
        snapshot.yaw_deg = radToDeg(active_pose.yaw);
        snapshot.pitch_deg = radToDeg(active_pose.pitch);
        if (sonar) {
            snapshot.range_m = sonar->getRange();
            snapshot.gain = sonar->getGain();
            snapshot.beam_count = static_cast<int>(sonar->getSonarBeamCount());
            snapshot.bin_count = static_cast<int>(sonar->getSonarBinCount());
            snapshot.beam_width_deg = radToDeg(sonar->getSonarBeamWidth().getRad());
            snapshot.beam_height_deg = radToDeg(sonar->getSonarBeamHeight().getRad());
        } else {
            snapshot.range_m = range_m;
            snapshot.gain = gain;
            snapshot.beam_count = static_cast<int>(beam_count);
            snapshot.bin_count = static_cast<int>(bin_count);
            snapshot.beam_width_deg = beam_width_deg;
            snapshot.beam_height_deg = beam_height_deg;
        }
        snapshot.center_frequency_khz = primary_fls_cfg.center_frequency_khz;
        snapshot.step_xy = pose_step_xy;
        snapshot.step_z = pose_step_z;
        snapshot.auto_pose = enable_auto_pose;
        snapshot.resolution = static_cast<int>(static_cast<float>(snapshot.bin_count) * kResolutionConstant);
        snapshot.enable_reverb = global_env_cfg.enable_reverb;
        snapshot.enable_speckle = global_env_cfg.enable_speckle;
        snapshot.enable_attenuation = global_env_cfg.enable_attenuation;
        snapshot.attenuation_frequency_khz = global_env_cfg.attenuation_frequency_khz;
        snapshot.temperature_c = global_env_cfg.temperature_c;
        snapshot.salinity_ppt = global_env_cfg.salinity_ppt;
        snapshot.acidity_ph = global_env_cfg.acidity_ph;
        snapshot.sonar_actual_fps = sonar_actual_fps;
        snapshot.sonar_target_fps = sonar_target_fps;
        snapshot.sonar_max_fps = sonar_max_fps;
        if (info_panel) {
            info_panel->updateSnapshot(snapshot);
        }
        update_viewport_layout();
        QApplication::processEvents(QEventLoop::AllEvents, 2);
        if (enable_opencv_window) {
            std::cout << "[gui] frame " << frames << " image update begin" << std::endl;
            const int key = cv::waitKey(1);
            if (key >= 0 && applyControlKey(key, commanded_pose, pose_step_xy, pose_step_yaw, pose_step_pitch, pose_step_z)) {
                pose_cmd_changed = true;
                syncTrackballToPose(trackball.get(), commanded_pose);
                std::cout << "[gui] pose_cmd key=" << static_cast<char>(key)
                          << " xyz=(" << commanded_pose.position.x() << "," << commanded_pose.position.y() << "," << commanded_pose.position.z()
                          << ") yaw=" << commanded_pose.yaw << " pitch=" << commanded_pose.pitch << std::endl;
            }
            std::cout << "[gui] frame " << frames << " image update end" << std::endl;
        }
        std::cout << "[gui] frame " << frames << " end" << std::endl;
        ++frames;
    }
    if (enable_opencv_window) {
        cv::destroyWindow("sonar_image");
    }
    persist_runtime_to_config();
    restart_if_requested();
    std::cout << "[gui] exit(normal)" << std::endl;
    return 0;
}



