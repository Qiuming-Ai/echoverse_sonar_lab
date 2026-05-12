#pragma once

#include <QString>
#include <vector>

namespace standalone_mvp {

/// Project file suffix (JSON content; use \ref AppConfigStore for load/save).
inline constexpr const char* kEslprojSuffix = ".eslproj";

/// Minimum validated 2D sonar beam width/height in degrees (must be > 0; upper bound \ref kMaxSonarBeamDeg).
inline constexpr double kMinSonarBeamDeg = 1e-9;
inline constexpr double kMaxSonarBeamDeg = 179.0;

struct SceneConfig {
    QString world = "ssiv_bahia";
    int viewer_width = 1360;
    double initial_pitch_deg = 30.0;
    bool third_person_view_enabled = false;
};

struct PoseConfig {
    double x = 49.34;
    double y = 106.01;
    double z = -5.87;
    double yaw_deg = 0.0;
    double pitch_deg = 30.0;
    double step_xy = 1.0;
    double step_z = 1.0;
    double step_yaw_deg = 5.0;
    bool enable_auto_pose = false;
};

struct SonarConfigUi {
    double range_m = 18.0;
    double gain = 0.35;
    double beam_width_deg = 120.0;
    double beam_height_deg = 20.0;
    double center_frequency_khz = 300.0;
    double bandwidth_khz = 60.0;
    double angular_resolution_deg = 0.5;
    int bin_count = 750;   // derived
    int beam_count = 256;  // derived
    double attenuation_frequency_khz = 300.0;
    double temperature_c = 25.0;
    double salinity_ppt = 0.0;
    double acidity_ph = 8.0;
    bool enable_reverb = true;
    bool enable_speckle = true;
    bool enable_attenuation = true;
    // Whether to run the 2D FLS depth-composer sonar image pipeline.
    // When disabled, only optional 3D point-cloud sonar (if enabled) will run.
    bool enable_2d_fls = true;
    double max_fps = 20.0;
    double viewer_max_fps = 60.0;
    double sound_speed_mps = 1500.0;
};

struct EnvironmentConfig {
    double attenuation_frequency_khz = 300.0;
    double temperature_c = 25.0;
    double salinity_ppt = 0.0;
    double acidity_ph = 8.0;
    bool enable_reverb = true;
    bool enable_speckle = true;
    bool enable_attenuation = true;
    double sound_speed_mps = 1500.0;
};

struct CameraConfig {
    double yaw_deg = 0.0;
    double pitch_deg = 30.0;
    double horizontal_fov_deg = 130.0;
    double vertical_fov_deg = 20.0;
};

struct SubCameraConfig {
    QString name;
    bool enabled = false;
    double roll_offset_deg = 0.0;
    double pitch_offset_deg = 0.0;
    double yaw_offset_deg = 0.0;
    double horizontal_fov_deg = 120.0;
    double vertical_fov_deg = 20.0;
};

struct SssCameraConfig {
    SubCameraConfig sss_a;
    SubCameraConfig sss_b;
};

struct CameraSystemConfig {
    CameraConfig main_camera;
    std::vector<SubCameraConfig> sub_cameras;
};

struct SonarCameraBindingConfig {
    QString fls_camera = "Main Camera";
    QString mbes_camera;
    QString sss_camera_slot1;
    QString sss_camera_slot2;
    QString point_cloud_camera;
};

enum class SonarModuleType {
    FLS = 0,
    MBES = 1,
    SSS = 2,
};

struct SideScanSonarConfigUi {
    bool enabled = true;
    /// Acoustic simulator range (m) for side-scan displays; independent from 2D FLS / MBES range.
    double range_m = 30.0;
    /// Shared by SSS A / B acoustic simulators (independent from 2D FLS / MBES sonar).
    double gain = 0.35;
    double center_frequency_khz = 300.0;
    double bandwidth_khz = 10.0;
    double beam_width_deg = 120.0;
    double beam_height_deg = 20.0;
    double angular_resolution_deg = 1.0;
    int window_width = 960;
    int window_height = 320;
    int update_stride = 2;
};

struct PointCloudSonarConfigUi {
    bool enabled = true;
    double range_m = 30.0;
    double frequency_khz = 300.0;
    double bandwidth_khz = 60.0;
    double horizontal_angle_resolution_deg = 0.75;
    double vertical_angle_resolution_deg = 0.75;
    double horizontal_fov_deg = 120.0;
    double vertical_fov_deg = 20.0;
    int max_point_count = 120000;
    int palette_index = 1; // 0 Jet, 1 Hot, 2 Gray, 3 Bronze
    bool show_coordinate_overlay = true;
    bool tcp_output_enabled = false;
    bool file_output_enabled = false;
    QString tcp_host = "0.0.0.0";
    int tcp_port = 30001;
};

struct SonarModuleConfig {
    QString name = "Sonar 1";
    SonarModuleType type = SonarModuleType::FLS;
    QString camera_binding = "Main Camera";
    bool enabled = true;
    SonarConfigUi fls_config;
    SonarConfigUi mbes_config;
    SideScanSonarConfigUi sss_config;
    PointCloudSonarConfigUi point_cloud_config;
    QString sss_camera_slot1;
    QString sss_camera_slot2;
    QString sonar_param_json_name;
};

struct AppConfigData {
    SceneConfig scene;
    PoseConfig pose;
    EnvironmentConfig environment;
    SonarConfigUi sonar;
    SonarConfigUi mbes_sonar;
    std::vector<SonarModuleConfig> sonar_modules;
    CameraSystemConfig camera_system;
    SonarCameraBindingConfig sonar_camera_binding;

    // Legacy fields kept for backward compatibility.
    CameraConfig camera;
    SssCameraConfig sss_camera;
    SideScanSonarConfigUi side_scan_sonar;
    SubCameraConfig mbes_camera;
    PointCloudSonarConfigUi point_cloud_sonar;
};

class AppConfigStore {
public:
    explicit AppConfigStore(QString config_path = QString());
    const QString& path() const;
    AppConfigData load();
    bool save(const AppConfigData& config) const;

private:
    QString config_path_;
};

/// Resolve a command-line project argument: `.eslproj` file path, or a directory containing one `.eslproj`.
/// Returns empty if the path does not exist or no project file can be resolved.
QString resolveProjectFileArgument(const QString& file_or_directory);

/// If `world` is relative, and a file exists at `<eslproj_dir>/<world>`, return that absolute path.
/// Otherwise returns `world` unchanged (bare scene keys still resolve in SharedScene).
QString resolveSceneWorldForLoad(const QString& world, const QString& eslproj_path);

/// Initial configuration for the new-project wizard (at most one module per type).
AppConfigData makeWizardProjectConfig(const QString& project_display_name, bool include_fls, bool include_mbes, bool include_sss);

/// Ensure per-module sonar parameter json files exist under "<project>/Sonar Params".
/// Returns true when cfg was modified.
bool ensureSonarParamFilesForProject(AppConfigData& cfg, const QString& project_dir);

} // namespace standalone_mvp
