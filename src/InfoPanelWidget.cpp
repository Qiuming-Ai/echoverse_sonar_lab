#include "InfoPanelWidget.hpp"

#include <QFrame>
#include <QVBoxLayout>

namespace standalone_mvp {
namespace {

QLabel* makeSectionTitle(const QString& text) {
    auto* label = new QLabel(text);
    label->setStyleSheet("color:#9bd1ff;font-weight:600;font-size:13px;");
    return label;
}

QLabel* makeBodyLabel() {
    auto* label = new QLabel();
    label->setStyleSheet("color:#e8edf2;font-size:12px;");
    label->setWordWrap(true);
    return label;
}

} // namespace

InfoPanelWidget::InfoPanelWidget(QWidget* parent)
    : QWidget(parent),
      run_label_(makeBodyLabel()),
      pose_label_(makeBodyLabel()),
      sonar_label_(makeBodyLabel()) {
    setObjectName("info_panel");
    setStyleSheet(
        "#info_panel{background-color:rgba(9,18,30,190);border:1px solid rgba(120,170,220,120);border-radius:8px;}"
        "QLabel{background:transparent;}");
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(10, 10, 10, 10);
    outer->setSpacing(8);

    auto* tips_label = makeBodyLabel();
    tips_label->setText(
        "W/S: Forward/Backward  A/D: Left/Right\n"
        "Q/E: Yaw Left/Right  U/J: Pitch Up/Down\n"
        "I/K: Move Up/Down");
    outer->addWidget(makeSectionTitle("Operation Tips"));
    outer->addWidget(tips_label);

    outer->addWidget(makeSectionTitle("Runtime"));
    outer->addWidget(run_label_);
    outer->addWidget(makeSectionTitle("Pose Details"));
    outer->addWidget(pose_label_);
    outer->addWidget(makeSectionTitle("Sonar Status"));
    outer->addWidget(sonar_label_);
}

void InfoPanelWidget::updateSnapshot(const RuntimeInfoSnapshot& s) {
    run_label_->setText(
        QString("Position: X %1  Y %2  Z %3\nYaw/Pitch: %4 / %5 deg\nRange/Gain: %6 m / %7\nCenter Freq: %8 kHz\nBin/Beam: %9 / %10")
            .arg(s.x, 0, 'f', 2)
            .arg(s.y, 0, 'f', 2)
            .arg(s.z, 0, 'f', 2)
            .arg(s.yaw_deg, 0, 'f', 2)
            .arg(s.pitch_deg, 0, 'f', 2)
            .arg(s.range_m, 0, 'f', 2)
            .arg(s.gain, 0, 'f', 3)
            .arg(s.center_frequency_khz, 0, 'f', 1)
            .arg(s.bin_count)
            .arg(s.beam_count));

    pose_label_->setText(
        QString("World: %1\nPosition: (%2, %3, %4)\nYaw/Pitch: %5 / %6 deg\nStep XY/Z: %7 / %8\nAuto Pose: %9")
            .arg(s.world)
            .arg(s.x, 0, 'f', 2)
            .arg(s.y, 0, 'f', 2)
            .arg(s.z, 0, 'f', 2)
            .arg(s.yaw_deg, 0, 'f', 2)
            .arg(s.pitch_deg, 0, 'f', 2)
            .arg(s.step_xy, 0, 'f', 2)
            .arg(s.step_z, 0, 'f', 2)
            .arg(s.auto_pose ? "Enabled" : "Disabled"));

    sonar_label_->setText(
        QString("Beam Width x Height: %1 x %2 deg\nResolution: %3\nSonar FPS: %4 / Max %5 (Target %6)\n"
                "Reverb/Speckle: %7 / %8\nAttenuation: %9\nFreq/Temp/Salinity/pH: %10 / %11 / %12 / %13")
            .arg(s.beam_width_deg, 0, 'f', 2)
            .arg(s.beam_height_deg, 0, 'f', 2)
            .arg(s.resolution)
            .arg(s.sonar_actual_fps, 0, 'f', 1)
            .arg(s.sonar_max_fps, 0, 'f', 1)
            .arg(s.sonar_target_fps, 0, 'f', 1)
            .arg(s.enable_reverb ? "On" : "Off")
            .arg(s.enable_speckle ? "On" : "Off")
            .arg(s.enable_attenuation ? "On" : "Off")
            .arg(s.attenuation_frequency_khz, 0, 'f', 1)
            .arg(s.temperature_c, 0, 'f', 1)
            .arg(s.salinity_ppt, 0, 'f', 1)
            .arg(s.acidity_ph, 0, 'f', 2));
}

} // namespace standalone_mvp
