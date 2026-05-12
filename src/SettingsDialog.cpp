#include "SettingsDialog.hpp"
#include "AppConfig.hpp"

#include <QCoreApplication>
#include <QDialogButtonBox>
#include <QDir>
#include <QDirIterator>
#include <QFormLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QSet>
#include <QVBoxLayout>
#include <algorithm>

namespace standalone_mvp {
namespace {

QDoubleSpinBox* makeDouble(double min_v, double max_v, double step, int decimals = 2) {
    auto* box = new QDoubleSpinBox();
    box->setRange(min_v, max_v);
    box->setSingleStep(step);
    box->setDecimals(decimals);
    return box;
}

QStringList discoverWorldSpecs() {
    QStringList scene_roots;
#if defined(STANDALONE_SIMULATION_DIR)
    scene_roots << QStringLiteral(STANDALONE_SIMULATION_DIR) + QStringLiteral("/uwmodels/scenes");
#endif
    scene_roots << (QDir::currentPath() + QStringLiteral("/uwmodels/scenes"));
    scene_roots << (QCoreApplication::applicationDirPath() + QStringLiteral("/uwmodels/scenes"));

    QSet<QString> world_names;
    for (const QString& root : scene_roots) {
        QDir root_dir(root);
        if (!root_dir.exists()) {
            continue;
        }
        QDirIterator it(root_dir.absolutePath(), QStringList() << QStringLiteral("*.world"), QDir::Files, QDirIterator::Subdirectories);
        while (it.hasNext()) {
            const QString file_path = it.next();
            const QFileInfo info(file_path);
            if (!info.baseName().isEmpty()) {
                world_names.insert(info.baseName());
            }
        }
    }
    if (world_names.isEmpty()) {
        return QStringList{QStringLiteral("ssiv_bahia")};
    }
    QStringList options = world_names.values();
    options.sort(Qt::CaseInsensitive);
    if (!options.contains(QStringLiteral("ssiv_bahia"), Qt::CaseInsensitive)) {
        options.prepend(QStringLiteral("ssiv_bahia"));
    }
    return options;
}

} // namespace

SettingsDialog::SettingsDialog(QWidget* parent)
    : QDialog(parent) {
    setWindowTitle("Settings");
    resize(760, 820);
    setStyleSheet(
        "QDialog{background:#ffffff;color:#111111;}"
        "QWidget{background:#ffffff;color:#111111;}"
        "QLabel{color:#111111;background:#ffffff;}"
        "QTabWidget::pane{border:1px solid #a8a8a8;background:#ffffff;}"
        "QTabBar::tab{background:#f2f2f2;color:#111111;border:1px solid #a8a8a8;padding:6px 10px;}"
        "QTabBar::tab:selected{background:#ffffff;}"
        "QLineEdit,QComboBox,QSpinBox,QDoubleSpinBox,QTableWidget{color:#111111;background:#ffffff;border:1px solid #b8b8b8;}"
        "QCheckBox{color:#111111;background:#ffffff;}"
        "QDialogButtonBox QPushButton{background:#f6f6f6;color:#111111;border:1px solid #b8b8b8;padding:6px 12px;}"
        "QDialogButtonBox QPushButton:hover{background:#ececec;}");

    auto* tabs = new QTabWidget(this);
    auto* scene_tab = new QWidget();
    auto* sonar_tab = new QWidget();
    auto* camera_tab = new QWidget();
    auto* pose_tab = new QWidget();
    auto* environment_tab = new QWidget();

    world_ = new QComboBox();
    world_->setEditable(true);
    world_->setInsertPolicy(QComboBox::NoInsert);
    world_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    scan_worlds_button_ = new QPushButton("Scan Worlds");
    auto* world_row = new QWidget();
    auto* world_row_layout = new QHBoxLayout(world_row);
    world_row_layout->setContentsMargins(0, 0, 0, 0);
    world_row_layout->addWidget(world_, 1);
    world_row_layout->addWidget(scan_worlds_button_);
    viewer_width_ = new QSpinBox();
    viewer_width_->setRange(320, 4096);
    initial_pitch_deg_ = makeDouble(-89.0, 89.0, 0.5);
    third_person_view_enabled_ = new QCheckBox("Enable Third-Person View");
    auto* scene_form = new QFormLayout(scene_tab);
    scene_form->addRow("World", world_row);
    scene_form->addRow("Viewer Width", viewer_width_);
    scene_form->addRow("Initial Pitch (deg)", initial_pitch_deg_);
    scene_form->addRow("", third_person_view_enabled_);
    connect(scan_worlds_button_, &QPushButton::clicked, this, &SettingsDialog::refreshWorldList);
    refreshWorldList();

    range_m_ = makeDouble(1.0, 1000.0, 0.5);
    gain_ = makeDouble(0.0, 1.0, 0.01, 3);
    center_frequency_khz_ = makeDouble(1.0, 2000.0, 1.0);
    bandwidth_khz_ = makeDouble(0.1, 1999.0, 1.0);
    angular_resolution_deg_ = makeDouble(0.05, 30.0, 0.05, 3);
    beam_width_deg_ = makeDouble(kMinSonarBeamDeg, kMaxSonarBeamDeg, 0.01, 6);
    beam_height_deg_ = makeDouble(kMinSonarBeamDeg, kMaxSonarBeamDeg, 0.01, 6);
    enable_2d_fls_ = new QCheckBox("Enable 2D FLS");
    enable_2d_fls_->setChecked(true);
    fls_camera_binding_ = new QComboBox();
    auto* fls_form = new QFormLayout();
    fls_form->addRow("Range (m)", range_m_);
    fls_form->addRow("Gain", gain_);
    fls_form->addRow("Center Frequency (kHz)", center_frequency_khz_);
    fls_form->addRow("Bandwidth (kHz)", bandwidth_khz_);
    fls_form->addRow("Beam Width (deg)", beam_width_deg_);
    fls_form->addRow("Beam Height (deg)", beam_height_deg_);
    fls_form->addRow("Angle Resolution (deg)", angular_resolution_deg_);
    fls_form->addRow("", enable_2d_fls_);
    fls_form->addRow("Camera", fls_camera_binding_);

    point_cloud_enabled_ = new QCheckBox("Enable 3D Point Cloud");
    point_cloud_range_m_ = makeDouble(0.0, 100.0, 0.5);
    point_cloud_frequency_khz_ = makeDouble(1.0, 2000.0, 1.0);
    point_cloud_bandwidth_khz_ = makeDouble(0.1, 2000.0, 1.0);
    point_cloud_horizontal_angle_resolution_deg_ = makeDouble(0.01, 30.0, 0.01, 3);
    point_cloud_vertical_angle_resolution_deg_ = makeDouble(0.01, 30.0, 0.01, 3);
    point_cloud_horizontal_fov_deg_ = makeDouble(1.0, 179.0, 0.5);
    point_cloud_vertical_fov_deg_ = makeDouble(1.0, 179.0, 0.5);
    point_cloud_max_point_count_ = new QSpinBox();
    point_cloud_max_point_count_->setRange(1000, 500000);
    point_cloud_max_point_count_->setSingleStep(1000);
    point_cloud_tcp_output_enabled_ = new QCheckBox("Enable TCP Output");
    point_cloud_file_output_enabled_ = new QCheckBox("Enable File Output");
    point_cloud_tcp_host_ = new QLineEdit();
    point_cloud_tcp_host_->setPlaceholderText("0.0.0.0");
    point_cloud_tcp_port_ = new QSpinBox();
    point_cloud_tcp_port_->setRange(1, 65535);
    point_cloud_tcp_port_->setValue(30001);
    point_cloud_camera_binding_ = new QComboBox();
    auto* point_cloud_form = new QFormLayout();
    point_cloud_form->addRow("", point_cloud_enabled_);
    point_cloud_form->addRow("Range (m)", point_cloud_range_m_);
    point_cloud_form->addRow("Frequency (kHz)", point_cloud_frequency_khz_);
    point_cloud_form->addRow("Bandwidth (kHz)", point_cloud_bandwidth_khz_);
    point_cloud_form->addRow("Horizontal Angle Res (deg)", point_cloud_horizontal_angle_resolution_deg_);
    point_cloud_form->addRow("Vertical Angle Res (deg)", point_cloud_vertical_angle_resolution_deg_);
    point_cloud_form->addRow("Horizontal FOV (deg)", point_cloud_horizontal_fov_deg_);
    point_cloud_form->addRow("Vertical FOV (deg)", point_cloud_vertical_fov_deg_);
    point_cloud_form->addRow("Max Point Count", point_cloud_max_point_count_);
    point_cloud_form->addRow("", point_cloud_tcp_output_enabled_);
    point_cloud_form->addRow("", point_cloud_file_output_enabled_);
    point_cloud_form->addRow("TCP Host", point_cloud_tcp_host_);
    point_cloud_form->addRow("TCP Port", point_cloud_tcp_port_);
    point_cloud_form->addRow("Camera", point_cloud_camera_binding_);
    auto* point_cloud_box = new QGroupBox("3D Point Cloud");
    point_cloud_box->setLayout(point_cloud_form);

    fls_dialog_ = new QDialog(this);
    fls_dialog_->setWindowTitle("FLS Settings");
    fls_dialog_->resize(560, 720);
    auto* fls_root = new QVBoxLayout(fls_dialog_);
    fls_root->addLayout(fls_form);
    fls_root->addWidget(point_cloud_box);
    {
        auto* bb = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, fls_dialog_);
        QObject::connect(bb, &QDialogButtonBox::accepted, fls_dialog_, &QDialog::accept);
        QObject::connect(bb, &QDialogButtonBox::rejected, fls_dialog_, &QDialog::reject);
        fls_root->addWidget(bb);
    }

    mbes_range_m_ = makeDouble(1.0, 1000.0, 0.5);
    mbes_gain_ = makeDouble(0.0, 1.0, 0.01, 3);
    mbes_center_frequency_khz_ = makeDouble(1.0, 2000.0, 1.0);
    mbes_bandwidth_khz_ = makeDouble(0.1, 1999.0, 1.0);
    mbes_angular_resolution_deg_ = makeDouble(0.05, 30.0, 0.05, 3);
    mbes_beam_width_deg_ = makeDouble(kMinSonarBeamDeg, kMaxSonarBeamDeg, 0.01, 6);
    mbes_beam_height_deg_ = makeDouble(kMinSonarBeamDeg, kMaxSonarBeamDeg, 0.01, 6);
    mbes_enable_2d_fls_ = new QCheckBox("Enable MBES Sonar");
    mbes_camera_binding_ = new QComboBox();
    mbes_point_cloud_enabled_ = new QCheckBox("Enable 3D Point Cloud");
    mbes_point_cloud_range_m_ = makeDouble(0.0, 100.0, 0.5);
    mbes_point_cloud_frequency_khz_ = makeDouble(1.0, 2000.0, 1.0);
    mbes_point_cloud_bandwidth_khz_ = makeDouble(0.1, 2000.0, 1.0);
    mbes_point_cloud_horizontal_angle_resolution_deg_ = makeDouble(0.01, 30.0, 0.01, 3);
    mbes_point_cloud_vertical_angle_resolution_deg_ = makeDouble(0.01, 30.0, 0.01, 3);
    mbes_point_cloud_horizontal_fov_deg_ = makeDouble(1.0, 179.0, 0.5);
    mbes_point_cloud_vertical_fov_deg_ = makeDouble(1.0, 179.0, 0.5);
    mbes_point_cloud_max_point_count_ = new QSpinBox();
    mbes_point_cloud_max_point_count_->setRange(1000, 500000);
    mbes_point_cloud_max_point_count_->setSingleStep(1000);
    mbes_point_cloud_tcp_output_enabled_ = new QCheckBox("Enable TCP Output");
    mbes_point_cloud_file_output_enabled_ = new QCheckBox("Enable File Output");
    mbes_point_cloud_tcp_host_ = new QLineEdit();
    mbes_point_cloud_tcp_host_->setPlaceholderText("0.0.0.0");
    mbes_point_cloud_tcp_port_ = new QSpinBox();
    mbes_point_cloud_tcp_port_->setRange(1, 65535);
    mbes_point_cloud_tcp_port_->setValue(30002);
    mbes_point_cloud_camera_binding_ = new QComboBox();
    mbes_dialog_ = new QDialog(this);
    mbes_dialog_->setWindowTitle("MBES Sonar Settings");
    mbes_dialog_->resize(560, 760);
    auto* mbes_root = new QVBoxLayout(mbes_dialog_);
    auto* mbes_sonar_form = new QFormLayout();
    mbes_sonar_form->addRow("Range (m)", mbes_range_m_);
    mbes_sonar_form->addRow("Gain", mbes_gain_);
    mbes_sonar_form->addRow("Center Frequency (kHz)", mbes_center_frequency_khz_);
    mbes_sonar_form->addRow("Bandwidth (kHz)", mbes_bandwidth_khz_);
    mbes_sonar_form->addRow("Beam Width (deg)", mbes_beam_width_deg_);
    mbes_sonar_form->addRow("Beam Height (deg)", mbes_beam_height_deg_);
    mbes_sonar_form->addRow("Angle Resolution (deg)", mbes_angular_resolution_deg_);
    mbes_sonar_form->addRow("", mbes_enable_2d_fls_);
    mbes_sonar_form->addRow("Camera", mbes_camera_binding_);
    auto* mbes_point_cloud_form = new QFormLayout();
    mbes_point_cloud_form->addRow("", mbes_point_cloud_enabled_);
    mbes_point_cloud_form->addRow("Range (m)", mbes_point_cloud_range_m_);
    mbes_point_cloud_form->addRow("Frequency (kHz)", mbes_point_cloud_frequency_khz_);
    mbes_point_cloud_form->addRow("Bandwidth (kHz)", mbes_point_cloud_bandwidth_khz_);
    mbes_point_cloud_form->addRow("Horizontal Angle Res (deg)", mbes_point_cloud_horizontal_angle_resolution_deg_);
    mbes_point_cloud_form->addRow("Vertical Angle Res (deg)", mbes_point_cloud_vertical_angle_resolution_deg_);
    mbes_point_cloud_form->addRow("Horizontal FOV (deg)", mbes_point_cloud_horizontal_fov_deg_);
    mbes_point_cloud_form->addRow("Vertical FOV (deg)", mbes_point_cloud_vertical_fov_deg_);
    mbes_point_cloud_form->addRow("Max Point Count", mbes_point_cloud_max_point_count_);
    mbes_point_cloud_form->addRow("", mbes_point_cloud_tcp_output_enabled_);
    mbes_point_cloud_form->addRow("", mbes_point_cloud_file_output_enabled_);
    mbes_point_cloud_form->addRow("TCP Host", mbes_point_cloud_tcp_host_);
    mbes_point_cloud_form->addRow("TCP Port", mbes_point_cloud_tcp_port_);
    mbes_point_cloud_form->addRow("Camera", mbes_point_cloud_camera_binding_);
    auto* mbes_point_cloud_box = new QGroupBox("3D Point Cloud");
    mbes_point_cloud_box->setLayout(mbes_point_cloud_form);
    mbes_root->addLayout(mbes_sonar_form);
    mbes_root->addWidget(mbes_point_cloud_box);
    {
        auto* bb = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, mbes_dialog_);
        QObject::connect(bb, &QDialogButtonBox::accepted, mbes_dialog_, &QDialog::accept);
        QObject::connect(bb, &QDialogButtonBox::rejected, mbes_dialog_, &QDialog::reject);
        mbes_root->addWidget(bb);
    }

    side_scan_enabled_ = new QCheckBox("Enable Side Scan Sonar Window");
    side_scan_range_m_ = makeDouble(1.0, 1000.0, 0.5);
    side_scan_gain_ = makeDouble(0.0, 1.0, 0.01, 3);
    side_scan_center_frequency_khz_ = makeDouble(1.0, 2000.0, 1.0);
    side_scan_bandwidth_khz_ = makeDouble(0.1, 1999.0, 1.0);
    side_scan_beam_width_deg_ = makeDouble(kMinSonarBeamDeg, kMaxSonarBeamDeg, 0.01, 6);
    side_scan_beam_height_deg_ = makeDouble(kMinSonarBeamDeg, kMaxSonarBeamDeg, 0.01, 6);
    side_scan_angular_resolution_deg_ = makeDouble(0.05, 30.0, 0.05, 3);
    side_scan_window_width_ = new QSpinBox();
    side_scan_window_width_->setRange(320, 4096);
    side_scan_window_height_ = new QSpinBox();
    side_scan_window_height_->setRange(120, 2160);
    side_scan_update_stride_ = new QSpinBox();
    side_scan_update_stride_->setRange(1, 30);
    sss_camera_slot1_binding_ = new QComboBox();
    sss_camera_slot2_binding_ = new QComboBox();
    side_scan_dialog_ = new QDialog(this);
    side_scan_dialog_->setWindowTitle("Side Scan Sonar Settings");
    side_scan_dialog_->resize(560, 620);
    auto* side_scan_form = new QFormLayout(side_scan_dialog_);
    side_scan_form->addRow("", side_scan_enabled_);
    side_scan_form->addRow("Detection Range (m)", side_scan_range_m_);
    side_scan_form->addRow("Gain (0-1)", side_scan_gain_);
    side_scan_form->addRow("Center Frequency (kHz)", side_scan_center_frequency_khz_);
    side_scan_form->addRow("Bandwidth (kHz)", side_scan_bandwidth_khz_);
    side_scan_form->addRow("Beam Width (deg)", side_scan_beam_width_deg_);
    side_scan_form->addRow("Beam Height (deg)", side_scan_beam_height_deg_);
    side_scan_form->addRow("Angular Resolution (deg)", side_scan_angular_resolution_deg_);
    side_scan_form->addRow("Window Width", side_scan_window_width_);
    side_scan_form->addRow("Window Height", side_scan_window_height_);
    side_scan_form->addRow("Update Stride (frames)", side_scan_update_stride_);
    side_scan_form->addRow("Camera Slot 1", sss_camera_slot1_binding_);
    side_scan_form->addRow("Camera Slot 2", sss_camera_slot2_binding_);
    {
        auto* bb = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, side_scan_dialog_);
        QObject::connect(bb, &QDialogButtonBox::accepted, side_scan_dialog_, &QDialog::accept);
        QObject::connect(bb, &QDialogButtonBox::rejected, side_scan_dialog_, &QDialog::reject);
        side_scan_form->addRow(bb);
    }

    connect(center_frequency_khz_, qOverload<double>(&QDoubleSpinBox::valueChanged), [this](double f_khz) {
        const double max_bw = std::max(0.1, f_khz - 0.1);
        bandwidth_khz_->setMaximum(max_bw);
        if (bandwidth_khz_->value() > max_bw) {
            bandwidth_khz_->setValue(max_bw);
        }
    });
    connect(mbes_center_frequency_khz_, qOverload<double>(&QDoubleSpinBox::valueChanged), [this](double f_khz) {
        const double max_bw = std::max(0.1, f_khz - 0.1);
        mbes_bandwidth_khz_->setMaximum(max_bw);
        if (mbes_bandwidth_khz_->value() > max_bw) {
            mbes_bandwidth_khz_->setValue(max_bw);
        }
    });

    connect(side_scan_center_frequency_khz_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double v) {
        const double mx = std::max(0.1, v - 0.1);
        side_scan_bandwidth_khz_->setMaximum(mx);
        if (side_scan_bandwidth_khz_->value() > mx) {
            side_scan_bandwidth_khz_->setValue(mx);
        }
    });

    camera_yaw_deg_ = makeDouble(-360.0, 360.0, 0.5);
    camera_pitch_deg_ = makeDouble(-89.0, 89.0, 0.5);
    camera_horizontal_fov_deg_ = makeDouble(5.0, 179.0, 0.5);
    camera_vertical_fov_deg_ = makeDouble(5.0, 179.0, 0.5);
    auto* main_camera_form = new QFormLayout();
    main_camera_form->addRow("Yaw (deg)", camera_yaw_deg_);
    main_camera_form->addRow("Pitch (deg)", camera_pitch_deg_);
    main_camera_form->addRow("Horizontal FOV (deg)", camera_horizontal_fov_deg_);
    main_camera_form->addRow("Vertical FOV (deg)", camera_vertical_fov_deg_);
    auto* main_camera_box = new QGroupBox("Main Camera");
    main_camera_box->setLayout(main_camera_form);

    aux_camera_table_ = new QTableWidget(0, 7);
    aux_camera_table_->setHorizontalHeaderLabels(
        QStringList() << "Name" << "Enable" << "Roll Offset" << "Pitch Offset" << "Yaw Offset" << "H FOV" << "V FOV");
    aux_camera_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    add_camera_button_ = new QPushButton("Add Camera");
    remove_camera_button_ = new QPushButton("Delete Camera");
    connect(add_camera_button_, &QPushButton::clicked, this, [this]() {
        const int row = aux_camera_table_->rowCount();
        aux_camera_table_->insertRow(row);
        aux_camera_table_->setItem(row, 0, new QTableWidgetItem(QString("Camera %1").arg(row + 1)));
        auto* enabled = new QTableWidgetItem();
        enabled->setCheckState(Qt::Checked);
        aux_camera_table_->setItem(row, 1, enabled);
        for (int col = 2; col <= 6; ++col) {
            aux_camera_table_->setItem(row, col, new QTableWidgetItem(col <= 4 ? "0" : "20"));
        }
        refreshCameraBindingOptions();
    });
    connect(remove_camera_button_, &QPushButton::clicked, this, [this]() {
        const int row = aux_camera_table_->currentRow();
        if (row >= 0) {
            aux_camera_table_->removeRow(row);
            refreshCameraBindingOptions();
        }
    });
    connect(aux_camera_table_, &QTableWidget::itemChanged, this, [this](QTableWidgetItem*) { refreshCameraBindingOptions(); });
    auto* aux_btn = new QHBoxLayout();
    aux_btn->addWidget(add_camera_button_);
    aux_btn->addWidget(remove_camera_button_);
    aux_btn->addStretch(1);
    auto* camera_layout = new QVBoxLayout(camera_tab);
    camera_layout->addWidget(main_camera_box);
    camera_layout->addWidget(new QLabel("Aux Cameras"));
    camera_layout->addWidget(aux_camera_table_, 1);
    camera_layout->addLayout(aux_btn);

    pos_x_ = makeDouble(-100000.0, 100000.0, 0.1);
    pos_y_ = makeDouble(-100000.0, 100000.0, 0.1);
    pos_z_ = makeDouble(-100000.0, 100000.0, 0.1);
    step_xy_ = makeDouble(0.01, 1000.0, 0.1);
    step_z_ = makeDouble(0.01, 1000.0, 0.1);
    step_yaw_deg_ = makeDouble(0.1, 90.0, 0.1);
    enable_auto_pose_ = new QCheckBox("Enable Auto Pose");
    auto* pose_form = new QFormLayout(pose_tab);
    pose_form->addRow("Pos X", pos_x_);
    pose_form->addRow("Pos Y", pos_y_);
    pose_form->addRow("Pos Z", pos_z_);
    pose_form->addRow("Step XY", step_xy_);
    pose_form->addRow("Step Z", step_z_);
    pose_form->addRow("Step Yaw (deg)", step_yaw_deg_);
    pose_form->addRow("", enable_auto_pose_);

    attenuation_frequency_khz_ = makeDouble(1.0, 1000.0, 1.0);
    temperature_c_ = makeDouble(-2.0, 40.0, 0.1);
    salinity_ppt_ = makeDouble(0.0, 40.0, 0.1);
    acidity_ph_ = makeDouble(6.5, 8.5, 0.01, 2);
    enable_reverb_ = new QCheckBox("Enable Reverb");
    enable_speckle_ = new QCheckBox("Enable Speckle");
    enable_attenuation_ = new QCheckBox("Enable Attenuation");
    max_fps_ = makeDouble(1.0, 240.0, 1.0);
    viewer_max_fps_ = makeDouble(1.0, 240.0, 1.0);
    sound_speed_mps_ = makeDouble(1000.0, 1800.0, 1.0);
    auto* environment_form = new QFormLayout(environment_tab);
    environment_form->addRow("Attenuation Frequency (kHz)", attenuation_frequency_khz_);
    environment_form->addRow("Temperature (C)", temperature_c_);
    environment_form->addRow("Salinity (ppt)", salinity_ppt_);
    environment_form->addRow("Acidity (pH)", acidity_ph_);
    environment_form->addRow("Max FPS", max_fps_);
    environment_form->addRow("3D View Max FPS", viewer_max_fps_);
    environment_form->addRow("Sound Speed (m/s)", sound_speed_mps_);
    environment_form->addRow("", enable_reverb_);
    environment_form->addRow("", enable_speckle_);
    environment_form->addRow("", enable_attenuation_);

    sonar_table_ = new QTableWidget(0, 5, sonar_tab);
    sonar_table_->setHorizontalHeaderLabels(QStringList() << "Name" << "Type" << "Camera" << "Enabled" << "Config");
    sonar_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    add_sonar_button_ = new QPushButton("Add Sonar", sonar_tab);
    remove_sonar_button_ = new QPushButton("Delete Sonar", sonar_tab);
    QObject::connect(add_sonar_button_, &QPushButton::clicked, this, [this]() {
        SonarModuleConfig sm;
        sm.name = QString("Sonar %1").arg(static_cast<int>(sonar_modules_.size() + 1));
        sonar_modules_.push_back(sm);
        refreshSonarTable();
    });
    QObject::connect(remove_sonar_button_, &QPushButton::clicked, this, [this]() {
        const int row = sonar_table_->currentRow();
        if (row >= 0 && row < static_cast<int>(sonar_modules_.size())) {
            sonar_modules_.erase(sonar_modules_.begin() + row);
            refreshSonarTable();
        }
    });
    auto* sonar_buttons = new QHBoxLayout();
    sonar_buttons->addWidget(add_sonar_button_);
    sonar_buttons->addWidget(remove_sonar_button_);
    sonar_buttons->addStretch(1);
    auto* sonar_root = new QVBoxLayout(sonar_tab);
    sonar_root->addWidget(new QLabel("Configure sonar modules list. Each row has independent config.", sonar_tab));
    sonar_root->addWidget(sonar_table_, 1);
    sonar_root->addLayout(sonar_buttons);

    tabs->addTab(scene_tab, "Scene");
    tabs->addTab(sonar_tab, "Sonar");
    tabs->addTab(camera_tab, "Camera");
    tabs->addTab(pose_tab, "Pose");
    tabs->addTab(environment_tab, "Environment");

    auto* buttons = new QDialogButtonBox();
    apply_button_ = buttons->addButton("Apply", QDialogButtonBox::ApplyRole);
    save_button_ = buttons->addButton("Save", QDialogButtonBox::AcceptRole);
    buttons->addButton("Close", QDialogButtonBox::RejectRole);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);

    restart_hint_ = new QLabel("Apply or Save will restart the application to apply settings.");
    restart_hint_->setStyleSheet("color:#8a5a00;");
    restart_hint_->setVisible(false);

    auto* root = new QVBoxLayout(this);
    root->addWidget(tabs);
    root->addWidget(restart_hint_);
    root->addWidget(buttons);
}

QStringList SettingsDialog::cameraNameOptions() const {
    QStringList out;
    out << "Main Camera";
    for (int row = 0; row < aux_camera_table_->rowCount(); ++row) {
        const auto* name_item = aux_camera_table_->item(row, 0);
        if (name_item && !name_item->text().trimmed().isEmpty()) {
            out << name_item->text().trimmed();
        }
    }
    return out;
}

void SettingsDialog::refreshCameraBindingOptions() {
    const QStringList names = cameraNameOptions();
    auto refill = [&](QComboBox* combo) {
        const QString current = combo->currentText();
        combo->clear();
        combo->addItem("");
        combo->addItems(names);
        const int idx = combo->findText(current);
        if (idx >= 0) combo->setCurrentIndex(idx);
    };
    refill(fls_camera_binding_);
    refill(mbes_camera_binding_);
    refill(sss_camera_slot1_binding_);
    refill(sss_camera_slot2_binding_);
    refill(point_cloud_camera_binding_);
    refill(mbes_point_cloud_camera_binding_);
    if (sonar_table_) {
        refreshSonarTable();
    }
}

void SettingsDialog::refreshSonarTypeRow(int row) {
    if (row < 0 || row >= static_cast<int>(sonar_modules_.size())) return;
    auto* type_combo = qobject_cast<QComboBox*>(sonar_table_->cellWidget(row, 1));
    auto* enabled_item = sonar_table_->item(row, 3);
    auto* name_item = sonar_table_->item(row, 0);
    if (name_item) sonar_modules_[row].name = name_item->text().trimmed();
    if (type_combo) {
        sonar_modules_[row].type = static_cast<SonarModuleType>(type_combo->currentData().toInt());
    }
    if (enabled_item) {
        sonar_modules_[row].enabled = (enabled_item->checkState() == Qt::Checked);
    }
    QString camera_text;
    if (sonar_modules_[row].type == SonarModuleType::SSS) {
        const QString slot1 = sonar_modules_[row].sss_camera_slot1.trimmed();
        const QString slot2 = sonar_modules_[row].sss_camera_slot2.trimmed();
        camera_text = slot1 + " | " + slot2;
        if (slot1.isEmpty() && slot2.isEmpty()) {
            camera_text.clear();
        }
    } else {
        camera_text = sonar_modules_[row].camera_binding;
    }
    if (auto* cam_item = sonar_table_->item(row, 2)) {
        cam_item->setText(camera_text);
    }
}

void SettingsDialog::openSonarConfigDialog(int row) {
    if (row < 0 || row >= static_cast<int>(sonar_modules_.size())) return;
    SonarModuleConfig& sm = sonar_modules_[row];
    if (sm.type == SonarModuleType::FLS) {
        range_m_->setValue(sm.fls_config.range_m);
        gain_->setValue(sm.fls_config.gain);
        center_frequency_khz_->setValue(sm.fls_config.center_frequency_khz);
        bandwidth_khz_->setValue(sm.fls_config.bandwidth_khz);
        angular_resolution_deg_->setValue(sm.fls_config.angular_resolution_deg);
        beam_width_deg_->setValue(sm.fls_config.beam_width_deg);
        beam_height_deg_->setValue(sm.fls_config.beam_height_deg);
        enable_2d_fls_->setChecked(sm.fls_config.enable_2d_fls);
        point_cloud_enabled_->setChecked(sm.point_cloud_config.enabled);
        point_cloud_range_m_->setValue(sm.point_cloud_config.range_m);
        point_cloud_frequency_khz_->setValue(sm.point_cloud_config.frequency_khz);
        point_cloud_bandwidth_khz_->setValue(sm.point_cloud_config.bandwidth_khz);
        point_cloud_horizontal_angle_resolution_deg_->setValue(sm.point_cloud_config.horizontal_angle_resolution_deg);
        point_cloud_vertical_angle_resolution_deg_->setValue(sm.point_cloud_config.vertical_angle_resolution_deg);
        point_cloud_horizontal_fov_deg_->setValue(sm.point_cloud_config.horizontal_fov_deg);
        point_cloud_vertical_fov_deg_->setValue(sm.point_cloud_config.vertical_fov_deg);
        point_cloud_max_point_count_->setValue(sm.point_cloud_config.max_point_count);
        point_cloud_tcp_output_enabled_->setChecked(sm.point_cloud_config.tcp_output_enabled);
        point_cloud_file_output_enabled_->setChecked(sm.point_cloud_config.file_output_enabled);
        point_cloud_tcp_host_->setText(sm.point_cloud_config.tcp_host);
        point_cloud_tcp_port_->setValue(sm.point_cloud_config.tcp_port);
        fls_camera_binding_->setCurrentText(sm.camera_binding);
        point_cloud_camera_binding_->setCurrentText(sm.camera_binding);
        if (fls_dialog_->exec() == QDialog::Accepted) {
            sm.camera_binding = fls_camera_binding_->currentText().trimmed();
            sm.fls_config.range_m = range_m_->value();
            sm.fls_config.gain = gain_->value();
            sm.fls_config.center_frequency_khz = center_frequency_khz_->value();
            sm.fls_config.bandwidth_khz = bandwidth_khz_->value();
            sm.fls_config.angular_resolution_deg = angular_resolution_deg_->value();
            sm.fls_config.beam_width_deg = beam_width_deg_->value();
            sm.fls_config.beam_height_deg = beam_height_deg_->value();
            sm.fls_config.enable_2d_fls = enable_2d_fls_->isChecked();
            sm.point_cloud_config.enabled = point_cloud_enabled_->isChecked();
            sm.point_cloud_config.range_m = point_cloud_range_m_->value();
            sm.point_cloud_config.frequency_khz = point_cloud_frequency_khz_->value();
            sm.point_cloud_config.bandwidth_khz = point_cloud_bandwidth_khz_->value();
            sm.point_cloud_config.horizontal_angle_resolution_deg = point_cloud_horizontal_angle_resolution_deg_->value();
            sm.point_cloud_config.vertical_angle_resolution_deg = point_cloud_vertical_angle_resolution_deg_->value();
            sm.point_cloud_config.horizontal_fov_deg = point_cloud_horizontal_fov_deg_->value();
            sm.point_cloud_config.vertical_fov_deg = point_cloud_vertical_fov_deg_->value();
            sm.point_cloud_config.max_point_count = point_cloud_max_point_count_->value();
            sm.point_cloud_config.tcp_output_enabled = point_cloud_tcp_output_enabled_->isChecked();
            sm.point_cloud_config.file_output_enabled = point_cloud_file_output_enabled_->isChecked();
            sm.point_cloud_config.tcp_host = point_cloud_tcp_host_->text().trimmed();
            sm.point_cloud_config.tcp_port = point_cloud_tcp_port_->value();
        }
    } else if (sm.type == SonarModuleType::MBES) {
        mbes_range_m_->setValue(sm.mbes_config.range_m);
        mbes_gain_->setValue(sm.mbes_config.gain);
        mbes_center_frequency_khz_->setValue(sm.mbes_config.center_frequency_khz);
        mbes_bandwidth_khz_->setValue(sm.mbes_config.bandwidth_khz);
        mbes_angular_resolution_deg_->setValue(sm.mbes_config.angular_resolution_deg);
        mbes_beam_width_deg_->setValue(sm.mbes_config.beam_width_deg);
        mbes_beam_height_deg_->setValue(sm.mbes_config.beam_height_deg);
        mbes_enable_2d_fls_->setChecked(sm.mbes_config.enable_2d_fls);
        mbes_camera_binding_->setCurrentText(sm.camera_binding);
        mbes_point_cloud_enabled_->setChecked(sm.point_cloud_config.enabled);
        mbes_point_cloud_range_m_->setValue(sm.point_cloud_config.range_m);
        mbes_point_cloud_frequency_khz_->setValue(sm.point_cloud_config.frequency_khz);
        mbes_point_cloud_bandwidth_khz_->setValue(sm.point_cloud_config.bandwidth_khz);
        mbes_point_cloud_horizontal_angle_resolution_deg_->setValue(sm.point_cloud_config.horizontal_angle_resolution_deg);
        mbes_point_cloud_vertical_angle_resolution_deg_->setValue(sm.point_cloud_config.vertical_angle_resolution_deg);
        mbes_point_cloud_horizontal_fov_deg_->setValue(sm.point_cloud_config.horizontal_fov_deg);
        mbes_point_cloud_vertical_fov_deg_->setValue(sm.point_cloud_config.vertical_fov_deg);
        mbes_point_cloud_max_point_count_->setValue(sm.point_cloud_config.max_point_count);
        mbes_point_cloud_tcp_output_enabled_->setChecked(sm.point_cloud_config.tcp_output_enabled);
        mbes_point_cloud_file_output_enabled_->setChecked(sm.point_cloud_config.file_output_enabled);
        mbes_point_cloud_tcp_host_->setText(sm.point_cloud_config.tcp_host);
        mbes_point_cloud_tcp_port_->setValue(sm.point_cloud_config.tcp_port);
        mbes_point_cloud_camera_binding_->setCurrentText(sm.camera_binding);
        if (mbes_dialog_->exec() == QDialog::Accepted) {
            sm.camera_binding = mbes_camera_binding_->currentText().trimmed();
            const QString cloud_camera = mbes_point_cloud_camera_binding_->currentText().trimmed();
            if (!cloud_camera.isEmpty()) {
                sm.camera_binding = cloud_camera;
            }
            sm.mbes_config.range_m = mbes_range_m_->value();
            sm.mbes_config.gain = mbes_gain_->value();
            sm.mbes_config.center_frequency_khz = mbes_center_frequency_khz_->value();
            sm.mbes_config.bandwidth_khz = mbes_bandwidth_khz_->value();
            sm.mbes_config.angular_resolution_deg = mbes_angular_resolution_deg_->value();
            sm.mbes_config.beam_width_deg = mbes_beam_width_deg_->value();
            sm.mbes_config.beam_height_deg = mbes_beam_height_deg_->value();
            sm.mbes_config.enable_2d_fls = mbes_enable_2d_fls_->isChecked();
            sm.point_cloud_config.enabled = mbes_point_cloud_enabled_->isChecked();
            sm.point_cloud_config.range_m = mbes_point_cloud_range_m_->value();
            sm.point_cloud_config.frequency_khz = mbes_point_cloud_frequency_khz_->value();
            sm.point_cloud_config.bandwidth_khz = mbes_point_cloud_bandwidth_khz_->value();
            sm.point_cloud_config.horizontal_angle_resolution_deg = mbes_point_cloud_horizontal_angle_resolution_deg_->value();
            sm.point_cloud_config.vertical_angle_resolution_deg = mbes_point_cloud_vertical_angle_resolution_deg_->value();
            sm.point_cloud_config.horizontal_fov_deg = mbes_point_cloud_horizontal_fov_deg_->value();
            sm.point_cloud_config.vertical_fov_deg = mbes_point_cloud_vertical_fov_deg_->value();
            sm.point_cloud_config.max_point_count = mbes_point_cloud_max_point_count_->value();
            sm.point_cloud_config.tcp_output_enabled = mbes_point_cloud_tcp_output_enabled_->isChecked();
            sm.point_cloud_config.file_output_enabled = mbes_point_cloud_file_output_enabled_->isChecked();
            sm.point_cloud_config.tcp_host = mbes_point_cloud_tcp_host_->text().trimmed();
            sm.point_cloud_config.tcp_port = mbes_point_cloud_tcp_port_->value();
        }
    } else {
        const QString old_slot1 = sm.sss_camera_slot1;
        const QString old_slot2 = sm.sss_camera_slot2;
        const SideScanSonarConfigUi old_sss_cfg = sm.sss_config;
        side_scan_enabled_->setChecked(sm.sss_config.enabled);
        side_scan_range_m_->setValue(sm.sss_config.range_m);
        side_scan_gain_->setValue(sm.sss_config.gain);
        side_scan_center_frequency_khz_->setValue(sm.sss_config.center_frequency_khz);
        side_scan_bandwidth_khz_->setValue(sm.sss_config.bandwidth_khz);
        side_scan_beam_width_deg_->setValue(sm.sss_config.beam_width_deg);
        side_scan_beam_height_deg_->setValue(sm.sss_config.beam_height_deg);
        side_scan_angular_resolution_deg_->setValue(sm.sss_config.angular_resolution_deg);
        side_scan_window_width_->setValue(sm.sss_config.window_width);
        side_scan_window_height_->setValue(sm.sss_config.window_height);
        side_scan_update_stride_->setValue(sm.sss_config.update_stride);
        sss_camera_slot1_binding_->setCurrentText(sm.sss_camera_slot1);
        sss_camera_slot2_binding_->setCurrentText(sm.sss_camera_slot2);
        if (side_scan_dialog_->exec() == QDialog::Accepted) {
            const QString slot1 = sss_camera_slot1_binding_->currentText().trimmed();
            const QString slot2 = sss_camera_slot2_binding_->currentText().trimmed();
            if (slot1.isEmpty() || slot2.isEmpty()) {
                QMessageBox::warning(
                    this,
                    QStringLiteral("SSS Camera Required"),
                    QStringLiteral("SSS 声纳必须指定两个 Camera（Slot 1 和 Slot 2）。"));
                sm.sss_camera_slot1 = old_slot1;
                sm.sss_camera_slot2 = old_slot2;
                sm.sss_config = old_sss_cfg;
                refreshSonarTable();
                return;
            }
            sm.camera_binding = slot1;
            sm.sss_camera_slot1 = slot1;
            sm.sss_camera_slot2 = slot2;
            sm.sss_config.enabled = side_scan_enabled_->isChecked();
            sm.sss_config.range_m = side_scan_range_m_->value();
            sm.sss_config.gain = side_scan_gain_->value();
            sm.sss_config.center_frequency_khz = side_scan_center_frequency_khz_->value();
            sm.sss_config.bandwidth_khz = side_scan_bandwidth_khz_->value();
            sm.sss_config.beam_width_deg = side_scan_beam_width_deg_->value();
            sm.sss_config.beam_height_deg = side_scan_beam_height_deg_->value();
            sm.sss_config.angular_resolution_deg = side_scan_angular_resolution_deg_->value();
            sm.sss_config.window_width = side_scan_window_width_->value();
            sm.sss_config.window_height = side_scan_window_height_->value();
            sm.sss_config.update_stride = side_scan_update_stride_->value();
        }
    }
    refreshSonarTable();
}

void SettingsDialog::refreshSonarTable() {
    sonar_table_->setRowCount(0);
    for (int row = 0; row < static_cast<int>(sonar_modules_.size()); ++row) {
        sonar_table_->insertRow(row);
        auto& sm = sonar_modules_[row];
        sonar_table_->setItem(row, 0, new QTableWidgetItem(sm.name));

        auto* type_combo = new QComboBox(sonar_table_);
        type_combo->addItem("FLS", static_cast<int>(SonarModuleType::FLS));
        type_combo->addItem("MBES", static_cast<int>(SonarModuleType::MBES));
        type_combo->addItem("SSS", static_cast<int>(SonarModuleType::SSS));
        type_combo->setCurrentIndex(type_combo->findData(static_cast<int>(sm.type)));
        QObject::connect(type_combo, &QComboBox::currentIndexChanged, this, [this, row](int) { refreshSonarTypeRow(row); });
        sonar_table_->setCellWidget(row, 1, type_combo);

        QString camera_text;
        if (sm.type == SonarModuleType::SSS) {
            const QString slot1 = sm.sss_camera_slot1.trimmed();
            const QString slot2 = sm.sss_camera_slot2.trimmed();
            camera_text = slot1 + " | " + slot2;
            if (slot1.isEmpty() && slot2.isEmpty()) {
                camera_text = "";
            }
        } else {
            camera_text = sm.camera_binding;
        }
        auto* cam_item = new QTableWidgetItem(camera_text);
        cam_item->setFlags(cam_item->flags() & ~Qt::ItemIsEditable);
        sonar_table_->setItem(row, 2, cam_item);

        auto* enabled = new QTableWidgetItem();
        enabled->setCheckState(sm.enabled ? Qt::Checked : Qt::Unchecked);
        sonar_table_->setItem(row, 3, enabled);

        auto* cfg_btn = new QPushButton("Configure...", sonar_table_);
        QObject::connect(cfg_btn, &QPushButton::clicked, this, [this, row]() { openSonarConfigDialog(row); });
        sonar_table_->setCellWidget(row, 4, cfg_btn);
    }
}

void SettingsDialog::setFromConfig(const AppConfigData& cfg) {
    refreshWorldList();
    world_->setCurrentText(cfg.scene.world);
    viewer_width_->setValue(cfg.scene.viewer_width);
    initial_pitch_deg_->setValue(cfg.scene.initial_pitch_deg);
    third_person_view_enabled_->setChecked(cfg.scene.third_person_view_enabled);

    pos_x_->setValue(cfg.pose.x);
    pos_y_->setValue(cfg.pose.y);
    pos_z_->setValue(cfg.pose.z);
    step_xy_->setValue(cfg.pose.step_xy);
    step_z_->setValue(cfg.pose.step_z);
    step_yaw_deg_->setValue(cfg.pose.step_yaw_deg);
    enable_auto_pose_->setChecked(cfg.pose.enable_auto_pose);

    camera_yaw_deg_->setValue(cfg.camera_system.main_camera.yaw_deg);
    camera_pitch_deg_->setValue(cfg.camera_system.main_camera.pitch_deg);
    camera_horizontal_fov_deg_->setValue(cfg.camera_system.main_camera.horizontal_fov_deg);
    camera_vertical_fov_deg_->setValue(cfg.camera_system.main_camera.vertical_fov_deg);

    aux_camera_table_->setRowCount(0);
    for (std::size_t i = 0; i < cfg.camera_system.sub_cameras.size(); ++i) {
        const auto& sc = cfg.camera_system.sub_cameras[i];
        const int row = aux_camera_table_->rowCount();
        aux_camera_table_->insertRow(row);
        aux_camera_table_->setItem(row, 0, new QTableWidgetItem(sc.name.isEmpty() ? QString("Camera %1").arg(static_cast<int>(i + 1)) : sc.name));
        auto* enabled = new QTableWidgetItem();
        enabled->setCheckState(sc.enabled ? Qt::Checked : Qt::Unchecked);
        aux_camera_table_->setItem(row, 1, enabled);
        aux_camera_table_->setItem(row, 2, new QTableWidgetItem(QString::number(sc.roll_offset_deg)));
        aux_camera_table_->setItem(row, 3, new QTableWidgetItem(QString::number(sc.pitch_offset_deg)));
        aux_camera_table_->setItem(row, 4, new QTableWidgetItem(QString::number(sc.yaw_offset_deg)));
        aux_camera_table_->setItem(row, 5, new QTableWidgetItem(QString::number(sc.horizontal_fov_deg)));
        aux_camera_table_->setItem(row, 6, new QTableWidgetItem(QString::number(sc.vertical_fov_deg)));
    }
    if (aux_camera_table_->rowCount() == 0) {
        add_camera_button_->click();
    }
    refreshCameraBindingOptions();

    attenuation_frequency_khz_->setValue(cfg.environment.attenuation_frequency_khz);
    temperature_c_->setValue(cfg.environment.temperature_c);
    salinity_ppt_->setValue(cfg.environment.salinity_ppt);
    acidity_ph_->setValue(cfg.environment.acidity_ph);
    max_fps_->setValue(cfg.sonar.max_fps);
    viewer_max_fps_->setValue(cfg.sonar.viewer_max_fps);
    sound_speed_mps_->setValue(cfg.environment.sound_speed_mps);
    enable_reverb_->setChecked(cfg.environment.enable_reverb);
    enable_speckle_->setChecked(cfg.environment.enable_speckle);
    enable_attenuation_->setChecked(cfg.environment.enable_attenuation);

    sonar_modules_ = cfg.sonar_modules;
    if (sonar_modules_.empty()) {
        SonarModuleConfig d;
        sonar_modules_.push_back(d);
    }
    refreshSonarTable();
}

AppConfigData SettingsDialog::configFromUi() {
    AppConfigData cfg;
    cfg.scene.world = world_->currentText().trimmed().isEmpty() ? QString("ssiv_bahia") : world_->currentText().trimmed();
    cfg.scene.viewer_width = viewer_width_->value();
    cfg.scene.initial_pitch_deg = initial_pitch_deg_->value();
    cfg.scene.third_person_view_enabled = third_person_view_enabled_->isChecked();

    cfg.pose.x = pos_x_->value();
    cfg.pose.y = pos_y_->value();
    cfg.pose.z = pos_z_->value();
    cfg.pose.step_xy = step_xy_->value();
    cfg.pose.step_z = step_z_->value();
    cfg.pose.step_yaw_deg = step_yaw_deg_->value();
    cfg.pose.enable_auto_pose = enable_auto_pose_->isChecked();

    cfg.camera_system.main_camera.yaw_deg = camera_yaw_deg_->value();
    cfg.camera_system.main_camera.pitch_deg = camera_pitch_deg_->value();
    cfg.camera_system.main_camera.horizontal_fov_deg = camera_horizontal_fov_deg_->value();
    cfg.camera_system.main_camera.vertical_fov_deg = camera_vertical_fov_deg_->value();
    cfg.camera = cfg.camera_system.main_camera;

    cfg.camera_system.sub_cameras.clear();
    for (int row = 0; row < aux_camera_table_->rowCount(); ++row) {
        SubCameraConfig sc;
        const auto* name_item = aux_camera_table_->item(row, 0);
        const auto* enabled_item = aux_camera_table_->item(row, 1);
        auto parse = [&](int col, double fallback) {
            bool ok = false;
            const double v = aux_camera_table_->item(row, col) ? aux_camera_table_->item(row, col)->text().toDouble(&ok) : fallback;
            return ok ? v : fallback;
        };
        sc.name = (name_item && !name_item->text().trimmed().isEmpty()) ? name_item->text().trimmed() : QString("Camera %1").arg(row + 1);
        sc.enabled = enabled_item ? (enabled_item->checkState() == Qt::Checked) : true;
        sc.roll_offset_deg = parse(2, 0.0);
        sc.pitch_offset_deg = parse(3, 0.0);
        sc.yaw_offset_deg = parse(4, 0.0);
        sc.horizontal_fov_deg = std::clamp(parse(5, 120.0), 1.0, 179.0);
        sc.vertical_fov_deg = std::clamp(parse(6, 20.0), 1.0, 179.0);
        cfg.camera_system.sub_cameras.push_back(sc);
    }

    cfg.environment.attenuation_frequency_khz = attenuation_frequency_khz_->value();
    cfg.environment.temperature_c = temperature_c_->value();
    cfg.environment.salinity_ppt = salinity_ppt_->value();
    cfg.environment.acidity_ph = acidity_ph_->value();
    cfg.sonar.max_fps = max_fps_->value();
    cfg.sonar.viewer_max_fps = viewer_max_fps_->value();
    cfg.environment.sound_speed_mps = sound_speed_mps_->value();
    cfg.environment.enable_reverb = enable_reverb_->isChecked();
    cfg.environment.enable_speckle = enable_speckle_->isChecked();
    cfg.environment.enable_attenuation = enable_attenuation_->isChecked();

    for (int row = 0; row < sonar_table_->rowCount(); ++row) {
        refreshSonarTypeRow(row);
    }
    cfg.sonar_modules = sonar_modules_;
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

    return cfg;
}

void SettingsDialog::refreshWorldList() {
    const QString current_world = world_->currentText().trimmed();
    const QStringList options = discoverWorldSpecs();
    world_->clear();
    world_->addItems(options);
    if (!current_world.isEmpty() && world_->findText(current_world) < 0) {
        world_->addItem(current_world);
    }
    world_->setCurrentText(current_world.isEmpty() ? QStringLiteral("ssiv_bahia") : current_world);
}

void SettingsDialog::setRestartHintVisible(bool visible) {
    restart_hint_->setVisible(visible);
}

QPushButton* SettingsDialog::applyButton() const {
    return apply_button_;
}

QPushButton* SettingsDialog::saveButton() const {
    return save_button_;
}

} // namespace standalone_mvp
