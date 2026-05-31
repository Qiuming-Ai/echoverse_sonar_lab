#pragma once

#include "PointCloudSonarSimulation.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QTimer>
#include <QWidget>

#include <sonar_palette/PaletteRamp.hpp>
#include <sonar_palette/PaletteTypes.hpp>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Group>
#include <osg/Vec4>
#include <osg/ref_ptr>
#include <osgViewer/Viewer>

namespace standalone_mvp {

class PointCloudViewerWindow : public QWidget {
public:
    explicit PointCloudViewerWindow(QWidget* parent = nullptr);
    void setInitialViewFromPose(const Eigen::Affine3d& pose_world);

    void setConfig(const PointCloudSonarConfig& cfg);
    void setRangeMeters(double range_m);
    PointCloudSonarConfig configFromUi() const;
    bool consumePendingConfig(PointCloudSonarConfig& cfg);
    void setRenderBlockedByMainViewer(bool blocked);

    void updatePointCloudFrame(const PointCloudFrame& frame);
    void setTcpRuntimeStatus(bool running, bool client_connected, std::uint64_t last_seq, std::size_t last_payload_bytes);
    void renderFrame();

private:
    bool eventFilter(QObject* watched, QEvent* event) override;
    void moveEvent(QMoveEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void showEvent(QShowEvent* event) override;
    void hideEvent(QHideEvent* event) override;
    void markConfigDirty();
    void buildUi();
    void updateInfoDrawerGeometry();
    void updateSettingsDrawerGeometry();
    void buildPointCloudScene();
    void updateStatusLabel(const PointCloudFrame& frame);
    void autoFrameCameraToPoints();
    void rebuildPalette(int palette_index);
    bool canRenderFrame() const;
    void syncEmbeddedViewport();
    void updateCoordinateOverlay(const PointCloudFrame& frame);

    QDoubleSpinBox* range_m_ = nullptr;
    QDoubleSpinBox* frequency_khz_ = nullptr;
    QDoubleSpinBox* bandwidth_khz_ = nullptr;
    QDoubleSpinBox* horizontal_res_deg_ = nullptr;
    QDoubleSpinBox* vertical_res_deg_ = nullptr;
    QDoubleSpinBox* horizontal_fov_deg_ = nullptr;
    QDoubleSpinBox* vertical_fov_deg_ = nullptr;
    QFormLayout* controls_form_ = nullptr;
    QComboBox* palette_combo_ = nullptr;
    QCheckBox* show_coordinate_overlay_ = nullptr;
    QCheckBox* tcp_output_enabled_ = nullptr;
    QCheckBox* file_output_enabled_ = nullptr;
    QLineEdit* tcp_host_ = nullptr;
    QSpinBox* tcp_port_ = nullptr;
    QLabel* status_label_ = nullptr;
    QWidget* settings_drawer_ = nullptr;
    QPushButton* settings_drawer_toggle_button_ = nullptr;
    bool settings_drawer_expanded_ = false;
    QWidget* info_drawer_ = nullptr;
    QPushButton* info_drawer_toggle_button_ = nullptr;
    bool info_drawer_expanded_ = false;
    QWidget* osg_container_ = nullptr;
    QWidget* palette_colorbar_ = nullptr;
    QTimer* render_timer_ = nullptr;
    bool render_blocked_by_main_viewer_ = false;
    bool config_dirty_ = false;

    osgViewer::Viewer viewer_;
    osg::ref_ptr<osg::Group> scene_root_;
    osg::ref_ptr<osg::Geode> point_geode_;
    osg::ref_ptr<osg::Geometry> point_geometry_;
    osg::ref_ptr<osg::Vec3Array> point_vertices_;
    osg::ref_ptr<osg::Vec4Array> point_colors_;
    osg::ref_ptr<osg::Geode> overlay_geode_;
    osg::ref_ptr<osg::Geometry> overlay_geometry_;
    osg::ref_ptr<osg::Vec3Array> overlay_vertices_;
    osg::ref_ptr<osg::Vec4Array> overlay_colors_;
    sonar_palette::PaletteRamp color_gradient_;
    std::vector<osg::Vec4> palette_;
    int palette_index_ = 1;
    bool has_auto_framed_ = false;
    bool has_external_initial_view_ = false;
    QWidget* tracked_host_window_ = nullptr;
    mutable std::size_t render_skip_count_ = 0;
    int last_viewport_w_ = -1;
    int last_viewport_h_ = -1;
};

} // namespace standalone_mvp
