#include "AppConfig.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>

#include <algorithm>
#include <filesystem>

namespace standalone_mvp {

namespace {
namespace fs = std::filesystem;

QString sanitizedSonarParamBaseName(QString name) {
    name = name.trimmed();
    name.replace(QRegularExpression(QStringLiteral("[\\\\/:*?\"<>|]+")), QStringLiteral("_"));
    if (name.isEmpty()) {
        name = QStringLiteral("Sonar");
    }
    return name;
}

bool patchSonarParamJsonByType(const QString& json_path, const SonarModuleConfig& sm) {
    QFile file(json_path);
    if (!file.open(QIODevice::ReadOnly)) {
        return false;
    }
    QJsonParseError parse_error;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parse_error);
    file.close();
    if (parse_error.error != QJsonParseError::NoError || !doc.isObject()) {
        return false;
    }

    const bool is_mbes = sm.type == SonarModuleType::MBES;
    const double freq_hz = (is_mbes ? sm.mbes_config.center_frequency_khz : sm.fls_config.center_frequency_khz) * 1000.0;
    const double bw_hz = (is_mbes ? sm.mbes_config.bandwidth_khz : sm.fls_config.bandwidth_khz) * 1000.0;

    QJsonObject root = doc.object();
    QJsonObject array_params = root.value("array_params").toObject();
    array_params["fc"] = freq_hz;
    array_params["BW"] = bw_hz;
    root["array_params"] = array_params;

    QJsonObject tx_signal_params = root.value("tx_signal_params").toObject();
    tx_signal_params["Subfc"] = freq_hz;
    tx_signal_params["SubBW"] = bw_hz;
    root["tx_signal_params"] = tx_signal_params;

    QFile out(json_path);
    if (!out.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return false;
    }
    out.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    out.close();
    return true;
}

QString sonarParamTemplatePath() {
    const QString rel = QStringLiteral("src/matlab_point2file2image/SonarParameter/Sonar.json");
    const QString from_cwd = QDir::cleanPath(QDir::currentPath() + QStringLiteral("/") + rel);
    if (QFile::exists(from_cwd)) {
        return from_cwd;
    }
    const QString from_app = QDir::cleanPath(
        QCoreApplication::applicationDirPath() + QStringLiteral("/../") + rel);
    return from_app;
}

QString worldKeyFromSelection(const QString& world) {
    const QString w = world.trimmed();
    if (w.isEmpty()) {
        return {};
    }
    QFileInfo fi(w);
    if (fi.suffix().compare(QStringLiteral("world"), Qt::CaseInsensitive) == 0) {
        return fi.completeBaseName();
    }
    return fi.fileName();
}

QStringList builtInSceneRoots() {
    QStringList roots;
#if defined(STANDALONE_SIMULATION_DIR)
    roots << (QStringLiteral(STANDALONE_SIMULATION_DIR) + QStringLiteral("/uwmodels/scenes"));
#endif
    roots << (QDir::currentPath() + QStringLiteral("/uwmodels/scenes"));
    roots << (QCoreApplication::applicationDirPath() + QStringLiteral("/uwmodels/scenes"));
    return roots;
}

bool copySceneDirRecursive(const QString& src_dir, const QString& dst_dir, QString* error) {
    std::error_code ec;
    const fs::path src_path = fs::path(src_dir.toStdWString());
    const fs::path dst_path = fs::path(dst_dir.toStdWString());
    if (!fs::exists(src_path, ec) || !fs::is_directory(src_path, ec)) {
        if (error) {
            *error = QStringLiteral("Source scene directory does not exist: %1").arg(src_dir);
        }
        return false;
    }
    if (fs::exists(dst_path, ec)) {
        return true;
    }
    if (!QDir().mkpath(QFileInfo(dst_dir).absolutePath())) {
        if (error) {
            *error = QStringLiteral("Failed to create destination parent directory: %1")
                         .arg(QFileInfo(dst_dir).absolutePath());
        }
        return false;
    }
    fs::copy(src_path, dst_path, fs::copy_options::recursive | fs::copy_options::overwrite_existing, ec);
    if (ec) {
        if (error) {
            *error = QStringLiteral("Failed to copy scene directory:\n%1").arg(QString::fromStdString(ec.message()));
        }
        return false;
    }
    return true;
}

} // namespace

void syncLegacyFieldsFromSonarModules(AppConfigData& cfg) {
    for (const auto& sm : cfg.sonar_modules) {
        if (sm.type == SonarModuleType::FLS) {
            cfg.sonar = sm.fls_config;
            cfg.point_cloud_sonar = sm.point_cloud_config;
            cfg.sonar_camera_binding.fls_camera = sm.camera_binding;
            cfg.sonar_camera_binding.point_cloud_camera = sm.camera_binding;
            break;
        }
    }
    for (const auto& sm : cfg.sonar_modules) {
        if (sm.type == SonarModuleType::MBES) {
            cfg.mbes_sonar = sm.mbes_config;
            cfg.sonar_camera_binding.mbes_camera = sm.camera_binding;
            break;
        }
    }
    for (const auto& sm : cfg.sonar_modules) {
        if (sm.type == SonarModuleType::SSS) {
            cfg.side_scan_sonar = sm.sss_config;
            cfg.sonar_camera_binding.sss_camera_slot1 = sm.sss_camera_slot1;
            cfg.sonar_camera_binding.sss_camera_slot2 = sm.sss_camera_slot2;
            break;
        }
    }
}

namespace {

double readDouble(const QJsonObject& obj, const char* key, double fallback) {
    if (!obj.contains(key)) {
        return fallback;
    }
    return obj.value(key).toDouble(fallback);
}

int readInt(const QJsonObject& obj, const char* key, int fallback) {
    if (!obj.contains(key)) {
        return fallback;
    }
    return obj.value(key).toInt(fallback);
}

bool readBool(const QJsonObject& obj, const char* key, bool fallback) {
    if (!obj.contains(key)) {
        return fallback;
    }
    return obj.value(key).toBool(fallback);
}

QString readString(const QJsonObject& obj, const char* key, const QString& fallback) {
    if (!obj.contains(key)) {
        return fallback;
    }
    const QString s = obj.value(key).toString();
    return s.isEmpty() ? fallback : s;
}

QJsonArray stringListToJson(const QStringList& list) {
    QJsonArray arr;
    for (const QString& value : list) {
        arr.append(value);
    }
    return arr;
}

QStringList stringListFromJson(const QJsonObject& obj, const char* key) {
    QStringList out;
    const QJsonArray arr = obj.value(key).toArray();
    for (const QJsonValue& value : arr) {
        out.push_back(value.toString());
    }
    return out;
}

QJsonObject subCameraToJson(const SubCameraConfig& c) {
    QJsonObject o;
    o["name"] = c.name;
    o["enabled"] = c.enabled;
    o["roll_offset_deg"] = c.roll_offset_deg;
    o["pitch_offset_deg"] = c.pitch_offset_deg;
    o["yaw_offset_deg"] = c.yaw_offset_deg;
    o["horizontal_fov_deg"] = c.horizontal_fov_deg;
    o["vertical_fov_deg"] = c.vertical_fov_deg;
    return o;
}

QJsonObject pathModeToJson(const PathModeConfig& cfg) {
    QJsonObject out;
    out["enabled"] = cfg.enabled;
    out["loop"] = cfg.loop;
    out["auto_start"] = cfg.auto_start;
    QJsonArray waypoints;
    for (const auto& wp : cfg.waypoints) {
        QJsonObject w;
        w["x"] = wp.x;
        w["y"] = wp.y;
        w["z"] = wp.depth_m;
        w["depth_m"] = wp.depth_m;
        w["speed_mps"] = wp.speed_mps;
        waypoints.append(w);
    }
    out["waypoints"] = waypoints;
    return out;
}

PathModeConfig pathModeFromJson(const QJsonObject& obj, PathModeConfig cfg) {
    cfg.enabled = readBool(obj, "enabled", cfg.enabled);
    cfg.loop = readBool(obj, "loop", cfg.loop);
    cfg.auto_start = readBool(obj, "auto_start", cfg.auto_start);
    cfg.waypoints.clear();
    const QJsonArray waypoints = obj.value("waypoints").toArray();
    for (const QJsonValue& v : waypoints) {
        const QJsonObject w = v.toObject();
        PathWaypointConfig wp;
        wp.x = readDouble(w, "x", 0.0);
        wp.y = readDouble(w, "y", 0.0);
        wp.depth_m = readDouble(w, "z", readDouble(w, "depth_m", -5.0));
        wp.speed_mps = std::max(0.001, readDouble(w, "speed_mps", 1.0));
        cfg.waypoints.push_back(wp);
    }
    return cfg;
}

QString sonarTypeToString(SonarModuleType type) {
    switch (type) {
    case SonarModuleType::FLS: return "FLS";
    case SonarModuleType::MBES: return "MBES";
    case SonarModuleType::SSS: return "SSS";
    }
    return "FLS";
}

SonarModuleType sonarTypeFromString(const QString& text) {
    if (text.compare("MBES", Qt::CaseInsensitive) == 0) return SonarModuleType::MBES;
    if (text.compare("SSS", Qt::CaseInsensitive) == 0) return SonarModuleType::SSS;
    return SonarModuleType::FLS;
}

QJsonObject sonarConfigToJson(const SonarConfigUi& cfg) {
    QJsonObject sonar;
    sonar["range_m"] = cfg.range_m;
    sonar["gain"] = cfg.gain;
    sonar["center_frequency_khz"] = cfg.center_frequency_khz;
    sonar["bandwidth_khz"] = cfg.bandwidth_khz;
    sonar["angular_resolution_deg"] = cfg.angular_resolution_deg;
    sonar["bin_count"] = cfg.bin_count;
    sonar["beam_count"] = cfg.beam_count;
    sonar["beam_width_deg"] = cfg.beam_width_deg;
    sonar["beam_height_deg"] = cfg.beam_height_deg;
    sonar["attenuation_frequency_khz"] = cfg.attenuation_frequency_khz;
    sonar["temperature_c"] = cfg.temperature_c;
    sonar["salinity_ppt"] = cfg.salinity_ppt;
    sonar["acidity_ph"] = cfg.acidity_ph;
    sonar["enable_reverb"] = cfg.enable_reverb;
    sonar["enable_speckle"] = cfg.enable_speckle;
    sonar["enable_attenuation"] = cfg.enable_attenuation;
    sonar["enable_2d_fls"] = cfg.enable_2d_fls;
    sonar["max_fps"] = cfg.max_fps;
    sonar["viewer_max_fps"] = cfg.viewer_max_fps;
    sonar["sound_speed_mps"] = cfg.sound_speed_mps;
    return sonar;
}

QJsonObject environmentConfigToJson(const EnvironmentConfig& cfg) {
    QJsonObject o;
    o["attenuation_frequency_khz"] = cfg.attenuation_frequency_khz;
    o["temperature_c"] = cfg.temperature_c;
    o["salinity_ppt"] = cfg.salinity_ppt;
    o["acidity_ph"] = cfg.acidity_ph;
    o["enable_reverb"] = cfg.enable_reverb;
    o["enable_speckle"] = cfg.enable_speckle;
    o["enable_attenuation"] = cfg.enable_attenuation;
    o["sound_speed_mps"] = cfg.sound_speed_mps;
    return o;
}

EnvironmentConfig environmentConfigFromJson(const QJsonObject& o, EnvironmentConfig cfg) {
    cfg.attenuation_frequency_khz =
        std::max(1.0, readDouble(o, "attenuation_frequency_khz", cfg.attenuation_frequency_khz));
    cfg.temperature_c = std::clamp(readDouble(o, "temperature_c", cfg.temperature_c), -2.0, 40.0);
    cfg.salinity_ppt = std::clamp(readDouble(o, "salinity_ppt", cfg.salinity_ppt), 0.0, 40.0);
    cfg.acidity_ph = std::clamp(readDouble(o, "acidity_ph", cfg.acidity_ph), 6.5, 8.5);
    cfg.enable_reverb = readBool(o, "enable_reverb", cfg.enable_reverb);
    cfg.enable_speckle = readBool(o, "enable_speckle", cfg.enable_speckle);
    cfg.enable_attenuation = readBool(o, "enable_attenuation", cfg.enable_attenuation);
    cfg.sound_speed_mps = std::clamp(readDouble(o, "sound_speed_mps", cfg.sound_speed_mps), 1000.0, 1800.0);
    return cfg;
}

SonarConfigUi sonarConfigFromJson(const QJsonObject& sonar, SonarConfigUi cfg) {
    cfg.range_m = std::max(1.0, readDouble(sonar, "range_m", cfg.range_m));
    cfg.gain = std::clamp(readDouble(sonar, "gain", cfg.gain), 0.0, 1.0);
    cfg.center_frequency_khz = std::clamp(readDouble(sonar, "center_frequency_khz", cfg.center_frequency_khz), 1.0, 2000.0);
    cfg.bandwidth_khz = std::clamp(readDouble(sonar, "bandwidth_khz", cfg.bandwidth_khz), 0.1, cfg.center_frequency_khz - 0.1);
    cfg.angular_resolution_deg = std::clamp(readDouble(sonar, "angular_resolution_deg", cfg.angular_resolution_deg), 0.05, 30.0);
    cfg.bin_count = std::max(1, readInt(sonar, "bin_count", cfg.bin_count));
    cfg.beam_count = std::max(1, readInt(sonar, "beam_count", cfg.beam_count));
    cfg.beam_width_deg = std::clamp(readDouble(sonar, "beam_width_deg", cfg.beam_width_deg), kMinSonarBeamDeg, kMaxSonarBeamDeg);
    cfg.beam_height_deg = std::clamp(readDouble(sonar, "beam_height_deg", cfg.beam_height_deg), kMinSonarBeamDeg, kMaxSonarBeamDeg);
    cfg.attenuation_frequency_khz = std::max(1.0, readDouble(sonar, "attenuation_frequency_khz", cfg.attenuation_frequency_khz));
    cfg.temperature_c = std::clamp(readDouble(sonar, "temperature_c", cfg.temperature_c), -2.0, 40.0);
    cfg.salinity_ppt = std::clamp(readDouble(sonar, "salinity_ppt", cfg.salinity_ppt), 0.0, 40.0);
    cfg.acidity_ph = std::clamp(readDouble(sonar, "acidity_ph", cfg.acidity_ph), 6.5, 8.5);
    cfg.enable_reverb = readBool(sonar, "enable_reverb", cfg.enable_reverb);
    cfg.enable_speckle = readBool(sonar, "enable_speckle", cfg.enable_speckle);
    cfg.enable_attenuation = readBool(sonar, "enable_attenuation", cfg.enable_attenuation);
    cfg.enable_2d_fls = readBool(sonar, "enable_2d_fls", cfg.enable_2d_fls);
    cfg.max_fps = std::clamp(readDouble(sonar, "max_fps", cfg.max_fps), 1.0, 240.0);
    cfg.viewer_max_fps = std::clamp(readDouble(sonar, "viewer_max_fps", cfg.viewer_max_fps), 1.0, 240.0);
    cfg.sound_speed_mps = std::clamp(readDouble(sonar, "sound_speed_mps", cfg.sound_speed_mps), 1000.0, 1800.0);
    return cfg;
}

QJsonObject sideScanConfigToJson(const SideScanSonarConfigUi& cfg) {
    QJsonObject o;
    o["enabled"] = cfg.enabled;
    o["range_m"] = cfg.range_m;
    o["gain"] = cfg.gain;
    o["center_frequency_khz"] = cfg.center_frequency_khz;
    o["bandwidth_khz"] = cfg.bandwidth_khz;
    o["beam_width_deg"] = cfg.beam_width_deg;
    o["beam_height_deg"] = cfg.beam_height_deg;
    o["angular_resolution_deg"] = cfg.angular_resolution_deg;
    o["window_width"] = cfg.window_width;
    o["window_height"] = cfg.window_height;
    o["update_stride"] = cfg.update_stride;
    return o;
}

SideScanSonarConfigUi sideScanConfigFromJson(const QJsonObject& o, SideScanSonarConfigUi cfg) {
    cfg.enabled = readBool(o, "enabled", cfg.enabled);
    cfg.range_m = std::clamp(readDouble(o, "range_m", cfg.range_m), 1.0, 1000.0);
    cfg.gain = std::clamp(readDouble(o, "gain", cfg.gain), 0.0, 1.0);
    cfg.center_frequency_khz = std::clamp(readDouble(o, "center_frequency_khz", cfg.center_frequency_khz), 1.0, 2000.0);
    cfg.bandwidth_khz = std::clamp(readDouble(o, "bandwidth_khz", cfg.bandwidth_khz), 0.1, std::max(0.1, cfg.center_frequency_khz - 0.1));
    cfg.beam_width_deg = std::clamp(readDouble(o, "beam_width_deg", cfg.beam_width_deg), kMinSonarBeamDeg, kMaxSonarBeamDeg);
    cfg.beam_height_deg = std::clamp(readDouble(o, "beam_height_deg", cfg.beam_height_deg), kMinSonarBeamDeg, kMaxSonarBeamDeg);
    cfg.angular_resolution_deg = std::clamp(readDouble(o, "angular_resolution_deg", cfg.angular_resolution_deg), 0.05, 30.0);
    cfg.window_width = std::clamp(readInt(o, "window_width", cfg.window_width), 320, 4096);
    cfg.window_height = std::clamp(readInt(o, "window_height", cfg.window_height), 120, 2160);
    cfg.update_stride = std::clamp(readInt(o, "update_stride", cfg.update_stride), 1, 30);
    return cfg;
}

QJsonObject pointCloudConfigToJson(const PointCloudSonarConfigUi& cfg) {
    QJsonObject o;
    o["enabled"] = cfg.enabled;
    o["range_m"] = cfg.range_m;
    o["frequency_khz"] = cfg.frequency_khz;
    o["bandwidth_khz"] = cfg.bandwidth_khz;
    o["horizontal_angle_resolution_deg"] = cfg.horizontal_angle_resolution_deg;
    o["vertical_angle_resolution_deg"] = cfg.vertical_angle_resolution_deg;
    o["horizontal_fov_deg"] = cfg.horizontal_fov_deg;
    o["vertical_fov_deg"] = cfg.vertical_fov_deg;
    o["max_point_count"] = cfg.max_point_count;
    o["palette_index"] = cfg.palette_index;
    o["show_coordinate_overlay"] = cfg.show_coordinate_overlay;
    o["tcp_output_enabled"] = cfg.tcp_output_enabled;
    o["file_output_enabled"] = cfg.file_output_enabled;
    o["tcp_host"] = cfg.tcp_host;
    o["tcp_port"] = cfg.tcp_port;
    return o;
}

PointCloudSonarConfigUi pointCloudConfigFromJson(const QJsonObject& o, PointCloudSonarConfigUi cfg) {
    cfg.enabled = readBool(o, "enabled", cfg.enabled);
    cfg.range_m = std::clamp(readDouble(o, "range_m", cfg.range_m), 0.0, 100.0);
    cfg.frequency_khz = std::clamp(readDouble(o, "frequency_khz", cfg.frequency_khz), 1.0, 2000.0);
    cfg.bandwidth_khz = std::clamp(readDouble(o, "bandwidth_khz", cfg.bandwidth_khz), 0.1, 2000.0);
    cfg.horizontal_angle_resolution_deg = std::clamp(readDouble(o, "horizontal_angle_resolution_deg", cfg.horizontal_angle_resolution_deg), 0.01, 30.0);
    cfg.vertical_angle_resolution_deg = std::clamp(readDouble(o, "vertical_angle_resolution_deg", cfg.vertical_angle_resolution_deg), 0.01, 30.0);
    cfg.horizontal_fov_deg = std::clamp(readDouble(o, "horizontal_fov_deg", cfg.horizontal_fov_deg), 1.0, 179.0);
    cfg.vertical_fov_deg = std::clamp(readDouble(o, "vertical_fov_deg", cfg.vertical_fov_deg), 1.0, 179.0);
    cfg.max_point_count = std::clamp(readInt(o, "max_point_count", cfg.max_point_count), 1000, 500000);
    cfg.palette_index = std::clamp(readInt(o, "palette_index", cfg.palette_index), 0, 3);
    cfg.show_coordinate_overlay = readBool(o, "show_coordinate_overlay", cfg.show_coordinate_overlay);
    cfg.tcp_output_enabled = readBool(o, "tcp_output_enabled", cfg.tcp_output_enabled);
    cfg.file_output_enabled = readBool(o, "file_output_enabled", cfg.file_output_enabled);
    cfg.tcp_host = readString(o, "tcp_host", cfg.tcp_host);
    cfg.tcp_port = std::clamp(readInt(o, "tcp_port", cfg.tcp_port), 1, 65535);
    return cfg;
}

SubCameraConfig readSubCameraFromJson(const QJsonObject& obj, const SubCameraConfig& fallback, const QString& default_name) {
    SubCameraConfig c = fallback;
    c.name = readString(obj, "name", default_name);
    c.enabled = readBool(obj, "enabled", c.enabled);
    c.roll_offset_deg = std::clamp(readDouble(obj, "roll_offset_deg", c.roll_offset_deg), -180.0, 180.0);
    c.pitch_offset_deg = std::clamp(readDouble(obj, "pitch_offset_deg", c.pitch_offset_deg), -180.0, 180.0);
    c.yaw_offset_deg = std::clamp(readDouble(obj, "yaw_offset_deg", c.yaw_offset_deg), -180.0, 180.0);
    c.horizontal_fov_deg = std::clamp(readDouble(obj, "horizontal_fov_deg", c.horizontal_fov_deg), 1.0, 179.0);
    c.vertical_fov_deg = std::clamp(readDouble(obj, "vertical_fov_deg", c.vertical_fov_deg), 1.0, 179.0);
    return c;
}

QJsonObject toJson(const AppConfigData& cfg) {
    QJsonObject scene;
    scene["world"] = cfg.scene.world;
    scene["viewer_width"] = cfg.scene.viewer_width;
    scene["initial_pitch_deg"] = cfg.scene.initial_pitch_deg;
    scene["third_person_view_enabled"] = cfg.scene.third_person_view_enabled;
    QJsonObject path_mode = pathModeToJson(cfg.path_mode);

    QJsonObject pose;
    pose["x"] = cfg.pose.x;
    pose["y"] = cfg.pose.y;
    pose["z"] = cfg.pose.z;
    pose["yaw_deg"] = cfg.pose.yaw_deg;
    pose["pitch_deg"] = cfg.pose.pitch_deg;
    pose["step_xy"] = cfg.pose.step_xy;
    pose["step_z"] = cfg.pose.step_z;
    pose["step_yaw_deg"] = cfg.pose.step_yaw_deg;
    pose["enable_auto_pose"] = cfg.pose.enable_auto_pose;

    QJsonObject sonar = sonarConfigToJson(cfg.sonar);
    QJsonObject environment = environmentConfigToJson(cfg.environment);
    QJsonObject mbes_sonar = sonarConfigToJson(cfg.mbes_sonar);

    QJsonObject camera;
    camera["yaw_deg"] = cfg.camera_system.main_camera.yaw_deg;
    camera["pitch_deg"] = cfg.camera_system.main_camera.pitch_deg;
    camera["horizontal_fov_deg"] = cfg.camera_system.main_camera.horizontal_fov_deg;
    camera["vertical_fov_deg"] = cfg.camera_system.main_camera.vertical_fov_deg;

    // Legacy compatibility mirror.
    QJsonObject legacy_camera;
    legacy_camera["yaw_deg"] = cfg.camera_system.main_camera.yaw_deg;
    legacy_camera["pitch_deg"] = cfg.camera_system.main_camera.pitch_deg;
    legacy_camera["horizontal_fov_deg"] = cfg.camera_system.main_camera.horizontal_fov_deg;
    legacy_camera["vertical_fov_deg"] = cfg.camera_system.main_camera.vertical_fov_deg;

    QJsonArray sub_cameras;
    for (const auto& sc : cfg.camera_system.sub_cameras) {
        sub_cameras.append(subCameraToJson(sc));
    }

    QJsonObject sonar_camera_binding;
    sonar_camera_binding["fls_camera"] = cfg.sonar_camera_binding.fls_camera;
    sonar_camera_binding["mbes_camera"] = cfg.sonar_camera_binding.mbes_camera;
    sonar_camera_binding["sss_camera_slot1"] = cfg.sonar_camera_binding.sss_camera_slot1;
    sonar_camera_binding["sss_camera_slot2"] = cfg.sonar_camera_binding.sss_camera_slot2;
    sonar_camera_binding["point_cloud_camera"] = cfg.sonar_camera_binding.point_cloud_camera;

    QJsonObject camera_system;
    camera_system["main_camera"] = camera;
    camera_system["sub_cameras"] = sub_cameras;

    // Legacy compatibility mirrors.
    camera["horizontal_fov_deg"] = cfg.camera.horizontal_fov_deg;
    camera["vertical_fov_deg"] = cfg.camera.vertical_fov_deg;
    QJsonObject sss_camera;
    sss_camera["sss_a"] = subCameraToJson(cfg.sss_camera.sss_a);
    sss_camera["sss_b"] = subCameraToJson(cfg.sss_camera.sss_b);
    QJsonObject side_scan_sonar = sideScanConfigToJson(cfg.side_scan_sonar);
    QJsonObject mbes_camera = subCameraToJson(cfg.mbes_camera);

    QJsonObject point_cloud_sonar = pointCloudConfigToJson(cfg.point_cloud_sonar);
    QJsonArray sonar_modules;
    for (const auto& module : cfg.sonar_modules) {
        QJsonObject m;
        m["name"] = module.name;
        m["type"] = sonarTypeToString(module.type);
        m["camera_binding"] = module.camera_binding;
        m["enabled"] = module.enabled;
        m["fls_config"] = sonarConfigToJson(module.fls_config);
        m["mbes_config"] = sonarConfigToJson(module.mbes_config);
        m["sss_config"] = sideScanConfigToJson(module.sss_config);
        m["point_cloud_config"] = pointCloudConfigToJson(module.point_cloud_config);
        m["sss_camera_slot1"] = module.sss_camera_slot1;
        m["sss_camera_slot2"] = module.sss_camera_slot2;
        m["sonar_param_json_name"] = module.sonar_param_json_name;
        sonar_modules.append(m);
    }

    QJsonObject ui_layout;
    ui_layout["sonar_window_docked_in_main"] = cfg.sonar_window_docked_in_main;
    ui_layout["sonar_workspace_split_layout"] = cfg.sonar_workspace_split_layout;
    ui_layout["sonar_workspace_single_tabs"] = stringListToJson(cfg.sonar_workspace_single_tabs);
    ui_layout["sonar_workspace_horizontal_left_tabs"] = stringListToJson(cfg.sonar_workspace_horizontal_left_tabs);
    ui_layout["sonar_workspace_horizontal_right_tabs"] = stringListToJson(cfg.sonar_workspace_horizontal_right_tabs);
    ui_layout["sonar_workspace_vertical_top_tabs"] = stringListToJson(cfg.sonar_workspace_vertical_top_tabs);
    ui_layout["sonar_workspace_vertical_bottom_tabs"] = stringListToJson(cfg.sonar_workspace_vertical_bottom_tabs);
    ui_layout["sonar_workspace_quad_top_left_tabs"] = stringListToJson(cfg.sonar_workspace_quad_top_left_tabs);
    ui_layout["sonar_workspace_quad_top_right_tabs"] = stringListToJson(cfg.sonar_workspace_quad_top_right_tabs);
    ui_layout["sonar_workspace_quad_bottom_left_tabs"] = stringListToJson(cfg.sonar_workspace_quad_bottom_left_tabs);
    ui_layout["sonar_workspace_quad_bottom_right_tabs"] = stringListToJson(cfg.sonar_workspace_quad_bottom_right_tabs);

    QJsonObject root;
    root["scene"] = scene;
    root["path_mode"] = path_mode;
    root["pose"] = pose;
    root["environment"] = environment;
    root["sonar"] = sonar;
    root["mbes_sonar"] = mbes_sonar;
    root["camera"] = legacy_camera;
    root["camera_system"] = camera_system;
    root["sonar_camera_binding"] = sonar_camera_binding;
    root["sss_camera"] = sss_camera;
    root["side_scan_sonar"] = side_scan_sonar;
    root["mbes_camera"] = mbes_camera;
    root["point_cloud_sonar"] = point_cloud_sonar;
    root["sonar_modules"] = sonar_modules;
    root["ui_layout"] = ui_layout;
    return root;
}

AppConfigData fromJson(const QJsonObject& root) {
    AppConfigData cfg;
    const QJsonObject scene = root.value("scene").toObject();
    const QJsonObject path_mode = root.value("path_mode").toObject();
    const QJsonObject pose = root.value("pose").toObject();
    const QJsonObject sonar = root.value("sonar").toObject();
    const QJsonObject environment = root.value("environment").toObject();
    const QJsonObject mbes_sonar = root.value("mbes_sonar").toObject();
    const QJsonObject camera = root.value("camera").toObject();
    const QJsonObject camera_system = root.value("camera_system").toObject();
    const QJsonObject sonar_camera_binding = root.value("sonar_camera_binding").toObject();
    const QJsonObject sss_camera = root.value("sss_camera").toObject();
    const QJsonObject side_scan_sonar = root.value("side_scan_sonar").toObject();
    const QJsonObject mbes_camera = root.value("mbes_camera").toObject();
    const QJsonObject point_cloud_sonar = root.value("point_cloud_sonar").toObject();
    const QJsonObject ui_layout = root.value("ui_layout").toObject();
    const QJsonArray sonar_modules = root.value("sonar_modules").toArray();

    cfg.scene.world = readString(scene, "world", cfg.scene.world);
    cfg.scene.viewer_width = std::max(320, readInt(scene, "viewer_width", cfg.scene.viewer_width));
    cfg.scene.initial_pitch_deg = readDouble(scene, "initial_pitch_deg", cfg.scene.initial_pitch_deg);
    cfg.scene.third_person_view_enabled = readBool(scene, "third_person_view_enabled", cfg.scene.third_person_view_enabled);
    cfg.path_mode = pathModeFromJson(path_mode, cfg.path_mode);

    cfg.pose.x = readDouble(pose, "x", cfg.pose.x);
    cfg.pose.y = readDouble(pose, "y", cfg.pose.y);
    cfg.pose.z = readDouble(pose, "z", cfg.pose.z);
    cfg.pose.yaw_deg = readDouble(pose, "yaw_deg", cfg.pose.yaw_deg);
    cfg.pose.pitch_deg = readDouble(pose, "pitch_deg", cfg.pose.pitch_deg);
    cfg.pose.step_xy = std::max(0.01, readDouble(pose, "step_xy", cfg.pose.step_xy));
    cfg.pose.step_z = std::max(0.01, readDouble(pose, "step_z", cfg.pose.step_z));
    cfg.pose.step_yaw_deg = std::clamp(readDouble(pose, "step_yaw_deg", cfg.pose.step_yaw_deg), 0.1, 90.0);
    cfg.pose.enable_auto_pose = readBool(pose, "enable_auto_pose", cfg.pose.enable_auto_pose);

    cfg.sonar = sonarConfigFromJson(sonar, cfg.sonar);
    cfg.mbes_sonar = sonarConfigFromJson(mbes_sonar, cfg.mbes_sonar);
    cfg.environment = environmentConfigFromJson(environment, cfg.environment);
    if (environment.isEmpty()) {
        // Backward-compatible migration from legacy sonar environment fields.
        cfg.environment.attenuation_frequency_khz = cfg.sonar.attenuation_frequency_khz;
        cfg.environment.temperature_c = cfg.sonar.temperature_c;
        cfg.environment.salinity_ppt = cfg.sonar.salinity_ppt;
        cfg.environment.acidity_ph = cfg.sonar.acidity_ph;
        cfg.environment.enable_reverb = cfg.sonar.enable_reverb;
        cfg.environment.enable_speckle = cfg.sonar.enable_speckle;
        cfg.environment.enable_attenuation = cfg.sonar.enable_attenuation;
        cfg.environment.sound_speed_mps = cfg.sonar.sound_speed_mps;
    }

    const QJsonObject main_camera = camera_system.value("main_camera").toObject();
    cfg.camera_system.main_camera.yaw_deg =
        std::clamp(readDouble(main_camera, "yaw_deg", readDouble(camera, "yaw_deg", cfg.camera_system.main_camera.yaw_deg)), -360.0, 360.0);
    cfg.camera_system.main_camera.pitch_deg =
        std::clamp(readDouble(main_camera, "pitch_deg", readDouble(camera, "pitch_deg", cfg.camera_system.main_camera.pitch_deg)), -89.0, 89.0);
    cfg.camera_system.main_camera.horizontal_fov_deg = std::clamp(
        readDouble(main_camera, "horizontal_fov_deg", readDouble(camera, "horizontal_fov_deg", cfg.camera_system.main_camera.horizontal_fov_deg)),
        5.0, 179.0);
    cfg.camera_system.main_camera.vertical_fov_deg = std::clamp(
        readDouble(main_camera, "vertical_fov_deg", readDouble(camera, "vertical_fov_deg", cfg.camera_system.main_camera.vertical_fov_deg)),
        5.0, 179.0);

    cfg.camera.horizontal_fov_deg = cfg.camera_system.main_camera.horizontal_fov_deg;
    cfg.camera.vertical_fov_deg = cfg.camera_system.main_camera.vertical_fov_deg;
    cfg.camera.yaw_deg = cfg.camera_system.main_camera.yaw_deg;
    cfg.camera.pitch_deg = cfg.camera_system.main_camera.pitch_deg;

    cfg.camera_system.sub_cameras.clear();
    const QJsonArray sub_cameras = camera_system.value("sub_cameras").toArray();
    for (int i = 0; i < sub_cameras.size(); ++i) {
        const QString default_name = QString("Camera %1").arg(i + 1);
        cfg.camera_system.sub_cameras.push_back(
            readSubCameraFromJson(sub_cameras.at(i).toObject(), SubCameraConfig(), default_name));
    }

    // Legacy migration if new camera list is absent.
    cfg.sss_camera.sss_a = readSubCameraFromJson(sss_camera.value("sss_a").toObject(), cfg.sss_camera.sss_a, "Camera 1");
    cfg.sss_camera.sss_b = readSubCameraFromJson(sss_camera.value("sss_b").toObject(), cfg.sss_camera.sss_b, "Camera 2");
    cfg.side_scan_sonar = sideScanConfigFromJson(side_scan_sonar, cfg.side_scan_sonar);
    cfg.mbes_camera = readSubCameraFromJson(mbes_camera, cfg.mbes_camera, "Camera 3");

    if (cfg.camera_system.sub_cameras.empty()) {
        if (cfg.sss_camera.sss_a.name.isEmpty()) cfg.sss_camera.sss_a.name = "Camera 1";
        if (cfg.sss_camera.sss_b.name.isEmpty()) cfg.sss_camera.sss_b.name = "Camera 2";
        if (cfg.mbes_camera.name.isEmpty()) cfg.mbes_camera.name = "Camera 3";
        cfg.camera_system.sub_cameras.push_back(cfg.sss_camera.sss_a);
        cfg.camera_system.sub_cameras.push_back(cfg.sss_camera.sss_b);
        cfg.camera_system.sub_cameras.push_back(cfg.mbes_camera);
    }

    cfg.point_cloud_sonar = pointCloudConfigFromJson(point_cloud_sonar, cfg.point_cloud_sonar);
    cfg.sonar_window_docked_in_main =
        readBool(ui_layout, "sonar_window_docked_in_main", cfg.sonar_window_docked_in_main);
    cfg.sonar_workspace_split_layout =
        readString(ui_layout, "sonar_workspace_split_layout", cfg.sonar_workspace_split_layout).trimmed().toLower();
    cfg.sonar_workspace_single_tabs = stringListFromJson(ui_layout, "sonar_workspace_single_tabs");
    cfg.sonar_workspace_horizontal_left_tabs = stringListFromJson(ui_layout, "sonar_workspace_horizontal_left_tabs");
    cfg.sonar_workspace_horizontal_right_tabs = stringListFromJson(ui_layout, "sonar_workspace_horizontal_right_tabs");
    cfg.sonar_workspace_vertical_top_tabs = stringListFromJson(ui_layout, "sonar_workspace_vertical_top_tabs");
    cfg.sonar_workspace_vertical_bottom_tabs = stringListFromJson(ui_layout, "sonar_workspace_vertical_bottom_tabs");
    cfg.sonar_workspace_quad_top_left_tabs = stringListFromJson(ui_layout, "sonar_workspace_quad_top_left_tabs");
    cfg.sonar_workspace_quad_top_right_tabs = stringListFromJson(ui_layout, "sonar_workspace_quad_top_right_tabs");
    cfg.sonar_workspace_quad_bottom_left_tabs = stringListFromJson(ui_layout, "sonar_workspace_quad_bottom_left_tabs");
    cfg.sonar_workspace_quad_bottom_right_tabs = stringListFromJson(ui_layout, "sonar_workspace_quad_bottom_right_tabs");

    cfg.sonar_modules.clear();
    for (int i = 0; i < sonar_modules.size(); ++i) {
        const QJsonObject m = sonar_modules.at(i).toObject();
        SonarModuleConfig sm;
        sm.name = readString(m, "name", QString("Sonar %1").arg(i + 1));
        sm.type = sonarTypeFromString(readString(m, "type", "FLS"));
        sm.camera_binding = readString(m, "camera_binding", "Main Camera");
        sm.enabled = readBool(m, "enabled", true);
        sm.fls_config = sonarConfigFromJson(m.value("fls_config").toObject(), sm.fls_config);
        sm.mbes_config = sonarConfigFromJson(m.value("mbes_config").toObject(), sm.mbes_config);
        sm.sss_config = sideScanConfigFromJson(m.value("sss_config").toObject(), sm.sss_config);
        sm.point_cloud_config = pointCloudConfigFromJson(m.value("point_cloud_config").toObject(), sm.point_cloud_config);
        sm.sss_camera_slot1 = readString(m, "sss_camera_slot1", "");
        sm.sss_camera_slot2 = readString(m, "sss_camera_slot2", "");
        sm.sonar_param_json_name = readString(m, "sonar_param_json_name", "");
        cfg.sonar_modules.push_back(sm);
    }
    if (cfg.sonar_modules.empty()) {
        SonarModuleConfig fls;
        fls.name = "FLS 1";
        fls.type = SonarModuleType::FLS;
        fls.camera_binding = cfg.sonar_camera_binding.fls_camera.isEmpty() ? "Main Camera" : cfg.sonar_camera_binding.fls_camera;
        fls.enabled = cfg.sonar.enable_2d_fls;
        fls.fls_config = cfg.sonar;
        fls.point_cloud_config = cfg.point_cloud_sonar;
        fls.sss_camera_slot1 = cfg.sonar_camera_binding.sss_camera_slot1;
        fls.sss_camera_slot2 = cfg.sonar_camera_binding.sss_camera_slot2;
        cfg.sonar_modules.push_back(fls);

        SonarModuleConfig mbes;
        mbes.name = "MBES 1";
        mbes.type = SonarModuleType::MBES;
        mbes.camera_binding = cfg.sonar_camera_binding.mbes_camera;
        mbes.enabled = cfg.mbes_sonar.enable_2d_fls;
        mbes.mbes_config = cfg.mbes_sonar;
        cfg.sonar_modules.push_back(mbes);

        SonarModuleConfig sss;
        sss.name = "SSS 1";
        sss.type = SonarModuleType::SSS;
        sss.camera_binding = cfg.sonar_camera_binding.sss_camera_slot1;
        sss.enabled = cfg.side_scan_sonar.enabled;
        sss.sss_config = cfg.side_scan_sonar;
        sss.sss_camera_slot1 = cfg.sonar_camera_binding.sss_camera_slot1;
        sss.sss_camera_slot2 = cfg.sonar_camera_binding.sss_camera_slot2;
        cfg.sonar_modules.push_back(sss);
    }

    cfg.sonar_camera_binding.fls_camera = readString(
        sonar_camera_binding, "fls_camera", cfg.sonar.enable_2d_fls ? QString("Main Camera") : QString());
    cfg.sonar_camera_binding.mbes_camera = readString(
        sonar_camera_binding, "mbes_camera", cfg.mbes_camera.enabled ? cfg.mbes_camera.name : QString());
    cfg.sonar_camera_binding.sss_camera_slot1 = readString(
        sonar_camera_binding, "sss_camera_slot1", cfg.sss_camera.sss_a.enabled ? cfg.sss_camera.sss_a.name : QString());
    cfg.sonar_camera_binding.sss_camera_slot2 = readString(
        sonar_camera_binding, "sss_camera_slot2", cfg.sss_camera.sss_b.enabled ? cfg.sss_camera.sss_b.name : QString());
    cfg.sonar_camera_binding.point_cloud_camera = readString(
        sonar_camera_binding, "point_cloud_camera", cfg.point_cloud_sonar.enabled ? QString("Main Camera") : QString());

    // Legacy mirrors from first modules of each type.
    syncLegacyFieldsFromSonarModules(cfg);
    return cfg;
}

} // namespace

AppConfigStore::AppConfigStore(QString config_path) {
    if (config_path.isEmpty()) {
        config_path = QCoreApplication::applicationDirPath() + QString("/default") + QString::fromUtf8(kEslprojSuffix);
    }
    config_path_ = std::move(config_path);
}

const QString& AppConfigStore::path() const {
    return config_path_;
}

AppConfigData AppConfigStore::load() {
    QFile file(config_path_);
    if (!file.exists()) {
        const QString legacy = QCoreApplication::applicationDirPath() + "/multibeam_gui_config.json";
        if (QFile::exists(legacy)) {
            QFile lf(legacy);
            if (lf.open(QIODevice::ReadOnly)) {
                QJsonParseError parse_error;
                const QJsonDocument doc = QJsonDocument::fromJson(lf.readAll(), &parse_error);
                if (parse_error.error == QJsonParseError::NoError && doc.isObject()) {
                    AppConfigData migrated = fromJson(doc.object());
                    save(migrated);
                    return migrated;
                }
            }
        }
        AppConfigData defaults;
        save(defaults);
        return defaults;
    }
    if (!file.open(QIODevice::ReadOnly)) {
        return AppConfigData();
    }
    QJsonParseError parse_error;
    const QJsonDocument doc = QJsonDocument::fromJson(file.readAll(), &parse_error);
    if (parse_error.error != QJsonParseError::NoError || !doc.isObject()) {
        AppConfigData defaults;
        save(defaults);
        return defaults;
    }
    return fromJson(doc.object());
}

bool AppConfigStore::save(const AppConfigData& config) const {
    QFile file(config_path_);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return false;
    }
    const QJsonDocument doc(toJson(config));
    const qint64 n = file.write(doc.toJson(QJsonDocument::Indented));
    return n > 0;
}

bool ensureSonarParamFilesForProject(AppConfigData& cfg, const QString& project_dir) {
    const QString template_path = sonarParamTemplatePath();
    if (!QFile::exists(template_path)) {
        return false;
    }
    const QString sonar_params_dir = QDir(project_dir).filePath("Sonar Params");
    QDir().mkpath(sonar_params_dir);

    bool changed = false;
    for (auto& sm : cfg.sonar_modules) {
        if (sm.type != SonarModuleType::FLS && sm.type != SonarModuleType::MBES) {
            continue;
        }
        QString rel_path = QDir::fromNativeSeparators(sm.sonar_param_json_name.trimmed());
        if (rel_path.isEmpty()) {
            rel_path = QStringLiteral("Sonar Params/%1.json").arg(sanitizedSonarParamBaseName(sm.name));
        } else {
            QFileInfo configured_info(rel_path);
            if (configured_info.isAbsolute()) {
                rel_path = QStringLiteral("Sonar Params/%1").arg(configured_info.fileName());
            } else if (!rel_path.startsWith(QStringLiteral("Sonar Params/"), Qt::CaseInsensitive)) {
                rel_path = QStringLiteral("Sonar Params/%1").arg(configured_info.fileName());
            }
        }
        if (sm.sonar_param_json_name != rel_path) {
            sm.sonar_param_json_name = rel_path;
            changed = true;
        }
        const QString json_path = QDir(project_dir).filePath(rel_path);
        QDir().mkpath(QFileInfo(json_path).absolutePath());
        if (!QFile::exists(json_path)) {
            QFile::copy(template_path, json_path);
            changed = true;
        }
        if (patchSonarParamJsonByType(json_path, sm)) {
            // Keep deterministic with module config even if file already exists.
            changed = true;
        }
    }
    return changed;
}

QString resolveProjectFileArgument(const QString& file_or_directory) {
    const QString trimmed = file_or_directory.trimmed();
    if (trimmed.isEmpty()) {
        return {};
    }
    QFileInfo fi(trimmed);
    if (!fi.exists()) {
        return {};
    }
    if (fi.isFile()) {
        return fi.absoluteFilePath();
    }
    if (fi.isDir()) {
        QDir d(fi.absoluteFilePath());
        QStringList files = d.entryList(QStringList() << QStringLiteral("*.eslproj"), QDir::Files, QDir::Name);
        if (files.isEmpty()) {
            return {};
        }
        const QString folder_name = d.dirName();
        for (const QString& f : files) {
            if (QFileInfo(f).completeBaseName().compare(folder_name, Qt::CaseInsensitive) == 0) {
                return d.absoluteFilePath(f);
            }
        }
        return d.absoluteFilePath(files.first());
    }
    return {};
}

QString resolveSceneWorldForLoad(const QString& world, const QString& eslproj_path) {
    const QString w = world.trimmed();
    if (w.isEmpty()) {
        return w;
    }
    QFileInfo wfi(w);
    if (wfi.isAbsolute()) {
        return QDir::cleanPath(wfi.absoluteFilePath());
    }
    const QString proj = eslproj_path.trimmed();
    if (proj.isEmpty()) {
        return w;
    }
    const QFileInfo proj_fi(proj);
    const QString base_dir = proj_fi.absolutePath();
    if (base_dir.isEmpty()) {
        return w;
    }
    const auto resolve_if_exists = [](const QString& candidate) -> QString {
        QFileInfo cand(candidate);
        if (cand.exists() && cand.isFile()) {
            return QDir::cleanPath(cand.absoluteFilePath());
        }
        return QString();
    };

    const QString direct_rel = resolve_if_exists(QDir(base_dir).filePath(w));
    if (!direct_rel.isEmpty()) {
        return direct_rel;
    }

    // Backward compatibility: old projects may still keep scene files under "scene/<name>/<name>.world".
    if (w.startsWith(QStringLiteral("scene/"), Qt::CaseInsensitive)) {
        const QString remapped = QStringLiteral("uwmodels/") + w.mid(QStringLiteral("scene/").size());
        const QString remapped_path = resolve_if_exists(QDir(base_dir).filePath(remapped));
        if (!remapped_path.isEmpty()) {
            return remapped_path;
        }
    }

    // When config stores a bare key (e.g. "mangalia"), prefer project-local world file.
    const QString key_path = resolve_if_exists(
        QDir(base_dir).filePath(QStringLiteral("uwmodels/scenes/%1/%1.world").arg(w)));
    if (!key_path.isEmpty()) {
        return key_path;
    }
    return w;
}

bool ensureProjectWorldDirectoryForSelection(const QString& world, const QString& eslproj_path, QString* error) {
    const QString w = world.trimmed();
    const QString proj = eslproj_path.trimmed();
    if (w.isEmpty() || proj.isEmpty()) {
        return true;
    }
    const QFileInfo proj_fi(proj);
    const QString project_dir = proj_fi.absolutePath();
    if (project_dir.isEmpty()) {
        return true;
    }

    const QString world_key = worldKeyFromSelection(w);
    if (world_key.isEmpty()) {
        return true;
    }
    const QString dst_scene_dir = QDir(project_dir).filePath(QStringLiteral("uwmodels/scenes/%1").arg(world_key));
    const QString dst_world_file = QDir(dst_scene_dir).filePath(world_key + QStringLiteral(".world"));
    if (QFileInfo::exists(dst_world_file)) {
        return true;
    }

    auto try_copy_from_dir = [&](const QString& src_dir) -> bool {
        if (src_dir.isEmpty()) {
            return false;
        }
        const QString src_world_file = QDir(src_dir).filePath(world_key + QStringLiteral(".world"));
        if (!QFileInfo::exists(src_world_file)) {
            return false;
        }
        QString copy_error;
        if (!copySceneDirRecursive(src_dir, dst_scene_dir, &copy_error)) {
            if (error) {
                *error = copy_error;
            }
            return false;
        }
        return QFileInfo::exists(dst_world_file);
    };

    const QFileInfo wfi(w);
    if (wfi.isAbsolute() && wfi.exists() && wfi.isFile()) {
        if (try_copy_from_dir(wfi.absolutePath())) {
            return true;
        }
    }

    const QString rel_world = QDir(project_dir).filePath(w);
    QFileInfo rel_fi(rel_world);
    if (rel_fi.exists() && rel_fi.isFile()) {
        if (try_copy_from_dir(rel_fi.absolutePath())) {
            return true;
        }
    }

    for (const QString& root : builtInSceneRoots()) {
        const QString src_dir = QDir(root).filePath(world_key);
        if (try_copy_from_dir(src_dir)) {
            return true;
        }
    }

    if (error) {
        *error = QStringLiteral("World '%1' was not found in project uwmodels/scenes or built-in scene roots.")
                     .arg(world_key);
    }
    return false;
}

AppConfigData makeWizardProjectConfig(const QString& project_display_name, bool include_fls, bool include_mbes, bool include_sss) {
    AppConfigData cfg;

    cfg.scene.world = QStringLiteral("mangalia");
    cfg.scene.viewer_width = 1360;
    cfg.scene.initial_pitch_deg = 30.0;
    cfg.scene.third_person_view_enabled = false;

    cfg.pose.x = -609.11;
    cfg.pose.y = 77.27;
    cfg.pose.z = 1.5;
    cfg.pose.yaw_deg = 0.0;
    cfg.pose.pitch_deg = 30.0;
    cfg.pose.step_xy = 1.0;
    cfg.pose.step_z = 1.0;
    cfg.pose.step_yaw_deg = 5.0;
    cfg.pose.enable_auto_pose = false;

    cfg.camera_system.main_camera.yaw_deg = -360.0;
    cfg.camera_system.main_camera.pitch_deg = 10.0;
    cfg.camera_system.main_camera.horizontal_fov_deg = 45.0;
    cfg.camera_system.main_camera.vertical_fov_deg = 45.0;

    cfg.camera_system.sub_cameras.clear();
    {
        SubCameraConfig ssa;
        ssa.name = QStringLiteral("SSS A");
        ssa.enabled = true;
        ssa.roll_offset_deg = 0.0;
        ssa.pitch_offset_deg = 20.0;
        ssa.yaw_offset_deg = 90.0;
        ssa.horizontal_fov_deg = 1.0;
        ssa.vertical_fov_deg = 120.0;
        SubCameraConfig ssb = ssa;
        ssb.name = QStringLiteral("SSS B");
        ssb.yaw_offset_deg = -90.0;
        SubCameraConfig mbes_cam;
        mbes_cam.name = QStringLiteral("MBES");
        mbes_cam.enabled = true;
        mbes_cam.roll_offset_deg = 0.0;
        mbes_cam.pitch_offset_deg = 90.0;
        mbes_cam.yaw_offset_deg = 0.0;
        mbes_cam.horizontal_fov_deg = 45.0;
        mbes_cam.vertical_fov_deg = 45.0;
        cfg.camera_system.sub_cameras.push_back(ssa);
        cfg.camera_system.sub_cameras.push_back(ssb);
        cfg.camera_system.sub_cameras.push_back(mbes_cam);
    }

    cfg.sss_camera.sss_a = cfg.camera_system.sub_cameras[0];
    cfg.sss_camera.sss_b = cfg.camera_system.sub_cameras[1];
    cfg.mbes_camera = cfg.camera_system.sub_cameras[2];

    cfg.sonar.range_m = 40.0;
    cfg.sonar.gain = 0.35;
    cfg.sonar.center_frequency_khz = 300.0;
    cfg.sonar.bandwidth_khz = 10.0;
    cfg.sonar.angular_resolution_deg = 1.0;
    cfg.sonar.bin_count = 533;
    cfg.sonar.beam_count = 120;
    cfg.sonar.beam_width_deg = 120.0;
    cfg.sonar.beam_height_deg = 20.0;
    cfg.sonar.attenuation_frequency_khz = 300.0;
    cfg.sonar.temperature_c = 25.0;
    cfg.sonar.salinity_ppt = 0.0;
    cfg.sonar.acidity_ph = 8.0;
    cfg.sonar.enable_reverb = true;
    cfg.sonar.enable_speckle = true;
    cfg.sonar.enable_attenuation = true;
    cfg.sonar.enable_2d_fls = true;
    cfg.sonar.max_fps = 15.0;
    cfg.sonar.viewer_max_fps = 5.0;
    cfg.sonar.sound_speed_mps = 1500.0;

    cfg.environment.attenuation_frequency_khz = 300.0;
    cfg.environment.temperature_c = 25.0;
    cfg.environment.salinity_ppt = 0.0;
    cfg.environment.acidity_ph = 8.0;
    cfg.environment.enable_reverb = true;
    cfg.environment.enable_speckle = true;
    cfg.environment.enable_attenuation = true;
    cfg.environment.sound_speed_mps = 1500.0;

    cfg.mbes_sonar.range_m = 88.0;
    cfg.mbes_sonar.gain = 0.3499999940395355;
    cfg.mbes_sonar.center_frequency_khz = 300.0;
    cfg.mbes_sonar.bandwidth_khz = 10.0;
    cfg.mbes_sonar.angular_resolution_deg = 1.0;
    cfg.mbes_sonar.bin_count = 1173;
    cfg.mbes_sonar.beam_count = 120;
    cfg.mbes_sonar.beam_width_deg = 120.0;
    cfg.mbes_sonar.beam_height_deg = 1.0;
    cfg.mbes_sonar.attenuation_frequency_khz = 300.0;
    cfg.mbes_sonar.temperature_c = 25.0;
    cfg.mbes_sonar.salinity_ppt = 0.0;
    cfg.mbes_sonar.acidity_ph = 8.0;
    cfg.mbes_sonar.enable_reverb = true;
    cfg.mbes_sonar.enable_speckle = true;
    cfg.mbes_sonar.enable_attenuation = true;
    cfg.mbes_sonar.enable_2d_fls = true;
    cfg.mbes_sonar.max_fps = 20.0;
    cfg.mbes_sonar.viewer_max_fps = 60.0;
    cfg.mbes_sonar.sound_speed_mps = 1500.0;

    cfg.side_scan_sonar.enabled = true;
    cfg.side_scan_sonar.range_m = 150.0;
    cfg.side_scan_sonar.gain = 0.3700000047683716;
    cfg.side_scan_sonar.center_frequency_khz = 30.0;
    cfg.side_scan_sonar.bandwidth_khz = 29.9;
    cfg.side_scan_sonar.beam_width_deg = 1.0;
    cfg.side_scan_sonar.beam_height_deg = 90.0;
    cfg.side_scan_sonar.angular_resolution_deg = 1.0;
    cfg.side_scan_sonar.window_width = 960;
    cfg.side_scan_sonar.window_height = 320;
    cfg.side_scan_sonar.update_stride = 2;

    cfg.point_cloud_sonar.enabled = true;
    cfg.point_cloud_sonar.range_m = 40.0;
    cfg.point_cloud_sonar.frequency_khz = 300.0;
    cfg.point_cloud_sonar.bandwidth_khz = 60.0;
    cfg.point_cloud_sonar.horizontal_angle_resolution_deg = 0.75;
    cfg.point_cloud_sonar.vertical_angle_resolution_deg = 0.75;
    cfg.point_cloud_sonar.horizontal_fov_deg = 45.0;
    cfg.point_cloud_sonar.vertical_fov_deg = 45.0;
    cfg.point_cloud_sonar.max_point_count = 120000;
    cfg.point_cloud_sonar.palette_index = 1;
    cfg.point_cloud_sonar.show_coordinate_overlay = true;
    cfg.point_cloud_sonar.tcp_output_enabled = false;
    cfg.point_cloud_sonar.tcp_host = QStringLiteral("0.0.0.0");
    cfg.point_cloud_sonar.tcp_port = 30001;

    cfg.sonar_camera_binding.fls_camera = QStringLiteral("Main Camera");
    cfg.sonar_camera_binding.mbes_camera = QStringLiteral("MBES");
    cfg.sonar_camera_binding.point_cloud_camera = QStringLiteral("Main Camera");
    cfg.sonar_camera_binding.sss_camera_slot1 = QStringLiteral("SSS A");
    cfg.sonar_camera_binding.sss_camera_slot2 = QStringLiteral("SSS B");

    cfg.sonar_modules.clear();
    if (include_fls) {
        SonarModuleConfig m;
        m.name = project_display_name + QStringLiteral(" FLS");
        m.type = SonarModuleType::FLS;
        m.enabled = true;
        m.camera_binding = QStringLiteral("Main Camera");
        m.fls_config = cfg.sonar;
        m.point_cloud_config = cfg.point_cloud_sonar;
        cfg.sonar_modules.push_back(m);
    }
    if (include_mbes) {
        SonarModuleConfig m;
        m.name = project_display_name + QStringLiteral(" MBES");
        m.type = SonarModuleType::MBES;
        m.enabled = true;
        m.camera_binding = QStringLiteral("MBES");
        m.mbes_config = cfg.mbes_sonar;
        cfg.sonar_modules.push_back(m);
    }
    if (include_sss) {
        SonarModuleConfig m;
        m.name = project_display_name + QStringLiteral(" SSS");
        m.type = SonarModuleType::SSS;
        m.enabled = true;
        m.camera_binding = QStringLiteral("SSS A");
        m.sss_config = cfg.side_scan_sonar;
        m.sss_camera_slot1 = QStringLiteral("SSS A");
        m.sss_camera_slot2 = QStringLiteral("SSS B");
        cfg.sonar_modules.push_back(m);
    }

    syncLegacyFieldsFromSonarModules(cfg);
    return cfg;
}

} // namespace standalone_mvp
