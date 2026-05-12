#pragma once

#include <QLabel>
#include <QWidget>

namespace standalone_mvp {

struct RuntimeInfoSnapshot {
    QString world;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw_deg = 0.0;
    double pitch_deg = 0.0;
    double range_m = 0.0;
    double gain = 0.0;
    int beam_count = 0;
    int bin_count = 0;
    double center_frequency_khz = 0.0;
    double step_xy = 0.0;
    double step_z = 0.0;
    bool auto_pose = false;
    double beam_width_deg = 0.0;
    double beam_height_deg = 0.0;
    int resolution = 0;
    bool enable_reverb = false;
    bool enable_speckle = false;
    bool enable_attenuation = false;
    double attenuation_frequency_khz = 0.0;
    double temperature_c = 0.0;
    double salinity_ppt = 0.0;
    double acidity_ph = 0.0;
    double sonar_actual_fps = 0.0;
    double sonar_target_fps = 0.0;
    double sonar_max_fps = 0.0;
};

class InfoPanelWidget : public QWidget {
public:
    explicit InfoPanelWidget(QWidget* parent = nullptr);
    void updateSnapshot(const RuntimeInfoSnapshot& snapshot);

private:
    QLabel* run_label_;
    QLabel* pose_label_;
    QLabel* sonar_label_;
};

} // namespace standalone_mvp
