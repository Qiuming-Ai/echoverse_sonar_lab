#pragma once

#include "AppConfig.hpp"

#include <QComboBox>
#include <QCheckBox>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QSpinBox>
#include <QTabWidget>
#include <QLineEdit>
#include <QTableWidget>
#include <vector>

namespace standalone_mvp {

class SettingsDialog : public QDialog {
public:
    explicit SettingsDialog(QWidget* parent = nullptr);
    void setFromConfig(const AppConfigData& config);
    AppConfigData configFromUi();
    void setRestartHintVisible(bool visible);
    QPushButton* applyButton() const;
    QPushButton* saveButton() const;

private:
    void refreshWorldList();
    void refreshSonarTable();
    void refreshSonarTypeRow(int row);
    void openSonarConfigDialog(int row);

    QComboBox* world_;
    QTableWidget* sonar_table_;
    QPushButton* add_sonar_button_;
    QPushButton* remove_sonar_button_;

    QPushButton* scan_worlds_button_;
    QSpinBox* viewer_width_;
    QDoubleSpinBox* initial_pitch_deg_;
    QCheckBox* third_person_view_enabled_;

    QDoubleSpinBox* range_m_;
    QDoubleSpinBox* gain_;
    QDoubleSpinBox* center_frequency_khz_;
    QDoubleSpinBox* bandwidth_khz_;
    QDoubleSpinBox* angular_resolution_deg_;
    QDoubleSpinBox* beam_width_deg_;
    QDoubleSpinBox* beam_height_deg_;
    QDoubleSpinBox* attenuation_frequency_khz_;
    QDoubleSpinBox* temperature_c_;
    QDoubleSpinBox* salinity_ppt_;
    QDoubleSpinBox* acidity_ph_;
    QCheckBox* enable_reverb_;
    QCheckBox* enable_speckle_;
    QCheckBox* enable_attenuation_;
    QCheckBox* enable_2d_fls_;
    QDoubleSpinBox* max_fps_;
    QDoubleSpinBox* viewer_max_fps_;
    QDoubleSpinBox* sound_speed_mps_;

    QDoubleSpinBox* camera_yaw_deg_;
    QDoubleSpinBox* camera_pitch_deg_;
    QDoubleSpinBox* camera_horizontal_fov_deg_;
    QDoubleSpinBox* camera_vertical_fov_deg_;
    QTableWidget* aux_camera_table_;
    QPushButton* add_camera_button_;
    QPushButton* remove_camera_button_;

    QComboBox* fls_camera_binding_;
    QComboBox* mbes_camera_binding_;
    QComboBox* sss_camera_slot1_binding_;
    QComboBox* sss_camera_slot2_binding_;
    QComboBox* point_cloud_camera_binding_;
    QDoubleSpinBox* pos_x_;
    QDoubleSpinBox* pos_y_;
    QDoubleSpinBox* pos_z_;
    QDoubleSpinBox* step_xy_;
    QDoubleSpinBox* step_z_;
    QDoubleSpinBox* step_yaw_deg_;
    QCheckBox* enable_auto_pose_;

    QDoubleSpinBox* mbes_range_m_;
    QDoubleSpinBox* mbes_gain_;
    QDoubleSpinBox* mbes_center_frequency_khz_;
    QDoubleSpinBox* mbes_bandwidth_khz_;
    QDoubleSpinBox* mbes_angular_resolution_deg_;
    QDoubleSpinBox* mbes_beam_width_deg_;
    QDoubleSpinBox* mbes_beam_height_deg_;
    QCheckBox* mbes_enable_2d_fls_;
    QCheckBox* mbes_point_cloud_enabled_;
    QDoubleSpinBox* mbes_point_cloud_range_m_;
    QDoubleSpinBox* mbes_point_cloud_frequency_khz_;
    QDoubleSpinBox* mbes_point_cloud_bandwidth_khz_;
    QDoubleSpinBox* mbes_point_cloud_horizontal_angle_resolution_deg_;
    QDoubleSpinBox* mbes_point_cloud_vertical_angle_resolution_deg_;
    QDoubleSpinBox* mbes_point_cloud_horizontal_fov_deg_;
    QDoubleSpinBox* mbes_point_cloud_vertical_fov_deg_;
    QSpinBox* mbes_point_cloud_max_point_count_;
    QCheckBox* mbes_point_cloud_tcp_output_enabled_;
    QCheckBox* mbes_point_cloud_file_output_enabled_;
    QLineEdit* mbes_point_cloud_tcp_host_;
    QSpinBox* mbes_point_cloud_tcp_port_;
    QComboBox* mbes_point_cloud_camera_binding_;

    QCheckBox* side_scan_enabled_;
    QDoubleSpinBox* side_scan_range_m_;
    QDoubleSpinBox* side_scan_gain_;
    QDoubleSpinBox* side_scan_center_frequency_khz_;
    QDoubleSpinBox* side_scan_bandwidth_khz_;
    QDoubleSpinBox* side_scan_beam_width_deg_;
    QDoubleSpinBox* side_scan_beam_height_deg_;
    QDoubleSpinBox* side_scan_angular_resolution_deg_;
    QSpinBox* side_scan_window_width_;
    QSpinBox* side_scan_window_height_;
    QSpinBox* side_scan_update_stride_;

    QCheckBox* point_cloud_enabled_;
    QDoubleSpinBox* point_cloud_range_m_;
    QDoubleSpinBox* point_cloud_frequency_khz_;
    QDoubleSpinBox* point_cloud_bandwidth_khz_;
    QDoubleSpinBox* point_cloud_horizontal_angle_resolution_deg_;
    QDoubleSpinBox* point_cloud_vertical_angle_resolution_deg_;
    QDoubleSpinBox* point_cloud_horizontal_fov_deg_;
    QDoubleSpinBox* point_cloud_vertical_fov_deg_;
    QSpinBox* point_cloud_max_point_count_;
    QCheckBox* point_cloud_tcp_output_enabled_;
    QCheckBox* point_cloud_file_output_enabled_;
    QLineEdit* point_cloud_tcp_host_;
    QSpinBox* point_cloud_tcp_port_;
    QDialog* fls_dialog_;
    QDialog* mbes_dialog_;
    QDialog* side_scan_dialog_;

    QLabel* restart_hint_;
    QPushButton* apply_button_;
    QPushButton* save_button_;
    std::vector<SonarModuleConfig> sonar_modules_;

    QStringList cameraNameOptions() const;
    void refreshCameraBindingOptions();
};

} // namespace standalone_mvp
