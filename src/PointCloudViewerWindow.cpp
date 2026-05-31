#include "PointCloudViewerWindow.hpp"

#include <QFormLayout>
#include <QFrame>
#include <QEvent>
#include <QFontMetrics>
#include <QHideEvent>
#include <QHBoxLayout>
#include <QMoveEvent>
#include <QPainter>
#include <QResizeEvent>
#include <QSignalBlocker>
#include <QShowEvent>
#include <QVBoxLayout>
#include <QWindow>
#include <QString>

#include <osg/LineWidth>
#include <osg/Point>
#include <osg/StateSet>
#include <osg/Viewport>
#include <osgViewer/GraphicsWindow>
#include <osgViewer/GraphicsWindow>
#if defined(_WIN32)
#include <osgViewer/api/Win32/GraphicsWindowWin32>
#endif
#include <osgGA/TrackballManipulator>

#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <Eigen/Geometry>

namespace standalone_mvp {
namespace {

constexpr int kInfoDrawerMargin = 8;
constexpr int kInfoDrawerMinHeight = 56;
constexpr int kInfoDrawerToggleWidth = 124;
constexpr int kInfoDrawerToggleHeight = 28;
constexpr int kInfoDrawerToggleGap = 6;
constexpr int kInfoDrawerInnerPadding = 8;
constexpr int kSettingsDrawerWidth = 310;
constexpr int kSettingsDrawerMargin = 8;
constexpr int kSettingsDrawerToggleWidth = 140;
constexpr int kSettingsDrawerToggleHeight = 28;
constexpr int kSettingsDrawerToggleGap = 6;

QDoubleSpinBox* makeDouble(double min_v, double max_v, double step, int decimals = 2) {
    auto* box = new QDoubleSpinBox();
    box->setRange(min_v, max_v);
    box->setSingleStep(step);
    box->setDecimals(decimals);
    return box;
}

struct ViewAttitude {
    double yaw_deg = 0.0;
    double pitch_deg = 0.0;
    double roll_deg = 0.0;
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
};

class PointCloudColorBar final : public QWidget {
public:
    explicit PointCloudColorBar(QWidget* parent = nullptr)
        : QWidget(parent) {
        colors_.assign(256, QColor(255, 255, 255));
        setMinimumWidth(54);
    }

    void setPaletteColors(const std::vector<osg::Vec4>& palette) {
        colors_.clear();
        colors_.reserve(palette.size());
        for (const auto& c : palette) {
            const int r = std::clamp(static_cast<int>(std::lround(c.r() * 255.0f)), 0, 255);
            const int g = std::clamp(static_cast<int>(std::lround(c.g() * 255.0f)), 0, 255);
            const int b = std::clamp(static_cast<int>(std::lround(c.b() * 255.0f)), 0, 255);
            colors_.emplace_back(r, g, b);
        }
        if (colors_.empty()) {
            colors_.assign(256, QColor(255, 255, 255));
        }
        update();
    }

protected:
    void paintEvent(QPaintEvent* event) override {
        QWidget::paintEvent(event);
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, false);
        painter.fillRect(rect(), QColor(2, 5, 10));

        const QRect bar_rect(10, 8, 20, std::max(10, height() - 16));
        const int color_count = static_cast<int>(colors_.size());
        for (int y = 0; y < bar_rect.height(); ++y) {
            const double t = 1.0 - static_cast<double>(y) / static_cast<double>(std::max(1, bar_rect.height() - 1));
            const int idx = std::clamp(static_cast<int>(std::lround(t * static_cast<double>(color_count - 1))), 0, color_count - 1);
            painter.setPen(colors_[static_cast<std::size_t>(idx)]);
            painter.drawLine(bar_rect.left(), bar_rect.top() + y, bar_rect.right(), bar_rect.top() + y);
        }
        painter.setPen(QPen(QColor(220, 232, 255), 1));
        painter.drawRect(bar_rect);
        painter.drawText(QRect(bar_rect.right() + 6, bar_rect.top() - 2, 18, 16),
                         Qt::AlignLeft | Qt::AlignVCenter, "1.0");
        painter.drawText(QRect(bar_rect.right() + 6, bar_rect.bottom() - 14, 18, 16),
                         Qt::AlignLeft | Qt::AlignVCenter, "0.0");
    }

private:
    std::vector<QColor> colors_;
};

class InfoDrawerFrame final : public QFrame {
public:
    explicit InfoDrawerFrame(QWidget* parent = nullptr)
        : QFrame(parent) {}

protected:
    void paintEvent(QPaintEvent* event) override {
        QFrame::paintEvent(event);
        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setBrush(Qt::NoBrush);
        painter.setPen(QPen(QColor(78, 109, 144, 255), 2.0));
        painter.drawRoundedRect(rect().adjusted(1, 1, -1, -1), 8.0, 8.0);
    }
};

double normalizeSignedDegrees(double deg) {
    constexpr double kFullTurn = 360.0;
    while (deg >= 180.0) {
        deg -= kFullTurn;
    }
    while (deg < -180.0) {
        deg += kFullTurn;
    }
    return deg;
}

ViewAttitude currentViewAttitude(osgViewer::Viewer& viewer) {
    ViewAttitude att;
    auto* tb = dynamic_cast<osgGA::TrackballManipulator*>(viewer.getCameraManipulator());
    if (!tb) {
        return att;
    }

    osg::Vec3d eye;
    osg::Vec3d center;
    osg::Vec3d up;
    tb->getTransformation(eye, center, up);

    Eigen::Vector3d f(center.x() - eye.x(), center.y() - eye.y(), center.z() - eye.z());
    Eigen::Vector3d u(up.x(), up.y(), up.z());
    if (f.norm() < 1e-9 || u.norm() < 1e-9) {
        return att;
    }
    f.normalize();
    u.normalize();

    Eigen::Vector3d l = u.cross(f);
    if (l.norm() < 1e-9) {
        return att;
    }
    l.normalize();
    u = f.cross(l);
    u.normalize();

    // Display frame convention: X forward, Y left, Z up.
    Eigen::Matrix3d R;
    R.col(0) = f;
    R.col(1) = l;
    R.col(2) = u;

    att.q = Eigen::Quaterniond(R);
    constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
    // Euler extraction using R = Rz(yaw) * Ry(pitch) * Rx(roll).
    const double pitch = std::asin(std::clamp(-R(2, 0), -1.0, 1.0));
    const double cp = std::cos(pitch);
    double roll = 0.0;
    double yaw = 0.0;
    if (std::abs(cp) > 1e-6) {
        roll = std::atan2(R(2, 1), R(2, 2));
        yaw = std::atan2(R(1, 0), R(0, 0));
    } else {
        roll = 0.0;
        yaw = std::atan2(-R(0, 1), R(1, 1));
    }
    att.yaw_deg = normalizeSignedDegrees(yaw * kRad2Deg);
    att.pitch_deg = normalizeSignedDegrees(pitch * kRad2Deg);
    att.roll_deg = normalizeSignedDegrees(roll * kRad2Deg);
    return att;
}

} // namespace

PointCloudViewerWindow::PointCloudViewerWindow(QWidget* parent)
    : QWidget(parent) {
    buildUi();
    buildPointCloudScene();
    rebuildPalette(palette_index_);
    render_timer_ = new QTimer(this);
    render_timer_->setInterval(33); // ~30 FPS, reduces context churn with main viewer
    connect(render_timer_, &QTimer::timeout, this, [this]() { renderFrame(); });
    render_timer_->start();
}

void PointCloudViewerWindow::setRenderBlockedByMainViewer(bool blocked) {
    render_blocked_by_main_viewer_ = blocked;
}

void PointCloudViewerWindow::buildUi() {
    setWindowTitle("3D Sonar Point Cloud");
    resize(1080, 760);
    setStyleSheet(
        "QWidget{background:#05090f;color:#eaf2ff;}"
        "QFrame{background:#02050a;border:1px solid #4e6d90;}"
        "QLabel{color:#d9e7ff;}"
        "QDoubleSpinBox{background:#091423;color:#eaf2ff;border:1px solid #4e6d90;}");

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(8, 8, 8, 8);
    root->setSpacing(8);

    auto* content = new QHBoxLayout();
    content->setSpacing(8);
    root->addLayout(content, 1);

    settings_drawer_ = new QFrame(this);
    settings_drawer_->setWindowFlags(Qt::Tool | Qt::FramelessWindowHint);
    settings_drawer_->setAttribute(Qt::WA_ShowWithoutActivating, true);
    settings_drawer_->setStyleSheet(
        "QFrame{background:rgba(9,20,35,220);border:1px solid #4e6d90;border-radius:8px;}");
    controls_form_ = new QFormLayout(settings_drawer_);
    auto* form = controls_form_;
    form->setContentsMargins(6, 6, 6, 6);
    form->setSpacing(6);

    range_m_ = makeDouble(0.0, 100.0, 0.5, 2);
    frequency_khz_ = makeDouble(1.0, 2000.0, 1.0, 1);
    bandwidth_khz_ = makeDouble(0.1, 2000.0, 0.5, 1);
    horizontal_res_deg_ = makeDouble(0.01, 20.0, 0.01, 3);
    vertical_res_deg_ = makeDouble(0.01, 20.0, 0.01, 3);
    horizontal_fov_deg_ = makeDouble(1.0, 179.0, 0.5, 2);
    vertical_fov_deg_ = makeDouble(1.0, 179.0, 0.5, 2);
    palette_combo_ = new QComboBox(settings_drawer_);
    palette_combo_->addItem("Jet", 0);
    palette_combo_->addItem("Hot", 1);
    palette_combo_->addItem("Gray", 2);
    palette_combo_->addItem("Bronze", 3);
    show_coordinate_overlay_ = new QCheckBox("Show Coordinate Overlay", settings_drawer_);
    show_coordinate_overlay_->setChecked(true);
    tcp_output_enabled_ = new QCheckBox("Enable TCP Output", settings_drawer_);
    file_output_enabled_ = new QCheckBox("Enable File Output", settings_drawer_);
    tcp_host_ = new QLineEdit(settings_drawer_);
    tcp_host_->setPlaceholderText("0.0.0.0");
    tcp_port_ = new QSpinBox(settings_drawer_);
    tcp_port_->setRange(1, 65535);
    tcp_port_->setValue(30001);

    form->addRow("Range (m)", range_m_);
    form->addRow("Frequency (kHz)", frequency_khz_);
    form->addRow("Bandwidth (kHz)", bandwidth_khz_);
    form->addRow("Horizontal Res (deg)", horizontal_res_deg_);
    form->addRow("Vertical Res (deg)", vertical_res_deg_);
    form->addRow("Horizontal FOV (deg)", horizontal_fov_deg_);
    form->addRow("Vertical FOV (deg)", vertical_fov_deg_);
    form->addRow("Palette", palette_combo_);
    form->addRow("", show_coordinate_overlay_);
    form->addRow("", tcp_output_enabled_);
    form->addRow("", file_output_enabled_);
    form->addRow("TCP Host", tcp_host_);
    form->addRow("TCP Port", tcp_port_);
    auto connect_dirty = [this](QDoubleSpinBox* box) {
        connect(box, qOverload<double>(&QDoubleSpinBox::valueChanged), [this](double) { markConfigDirty(); });
    };
    connect_dirty(range_m_);
    connect_dirty(frequency_khz_);
    connect_dirty(bandwidth_khz_);
    connect_dirty(horizontal_res_deg_);
    connect_dirty(vertical_res_deg_);
    connect_dirty(horizontal_fov_deg_);
    connect_dirty(vertical_fov_deg_);
    connect(palette_combo_, qOverload<int>(&QComboBox::currentIndexChanged), [this](int) {
        const int idx = palette_combo_->currentData().toInt();
        rebuildPalette(idx);
        markConfigDirty();
    });
    connect(show_coordinate_overlay_, &QCheckBox::toggled, [this](bool on) {
        if (overlay_geode_) {
            overlay_geode_->setNodeMask(on ? ~0u : 0u);
        }
        markConfigDirty();
    });
    connect(tcp_output_enabled_, &QCheckBox::toggled, [this](bool) { markConfigDirty(); });
    connect(file_output_enabled_, &QCheckBox::toggled, [this](bool) { markConfigDirty(); });
    connect(tcp_host_, &QLineEdit::textChanged, [this](const QString&) { markConfigDirty(); });
    connect(tcp_port_, qOverload<int>(&QSpinBox::valueChanged), [this](int) { markConfigDirty(); });

    auto* viewer_frame = new QFrame(this);
    content->addWidget(viewer_frame, 1);

    viewer_.setUpViewInWindow(80, 80, 900, 680);
    viewer_.setCameraManipulator(new osgGA::TrackballManipulator());
    viewer_.getCamera()->setClearColor(osg::Vec4(0.01f, 0.02f, 0.03f, 1.0f));
    viewer_.setThreadingModel(osgViewer::Viewer::SingleThreaded);

#if defined(_WIN32)
    auto* gw_win32 = dynamic_cast<osgViewer::GraphicsWindowWin32*>(viewer_.getCamera()->getGraphicsContext());
    QWindow* osg_foreign_window = gw_win32 ? QWindow::fromWinId(reinterpret_cast<WId>(gw_win32->getHWND())) : nullptr;
#else
    QWindow* osg_foreign_window = nullptr;
#endif
    auto* frame_layout = new QHBoxLayout(viewer_frame);
    frame_layout->setContentsMargins(4, 4, 4, 4);
    frame_layout->setSpacing(6);
    if (osg_foreign_window) {
        osg_container_ = QWidget::createWindowContainer(osg_foreign_window, viewer_frame);
        osg_container_->setFocusPolicy(Qt::StrongFocus);
        frame_layout->addWidget(osg_container_, 1);
    } else {
        auto* fallback = new QLabel("OSG viewer unavailable", viewer_frame);
        fallback->setAlignment(Qt::AlignCenter);
        fallback->setStyleSheet("QLabel{border:1px dashed #4e6d90;color:#9fb5d3;}");
        frame_layout->addWidget(fallback, 1);
    }
    palette_colorbar_ = new PointCloudColorBar(viewer_frame);
    frame_layout->addWidget(palette_colorbar_, 0);

    settings_drawer_toggle_button_ = new QPushButton();
    settings_drawer_toggle_button_->setAttribute(Qt::WA_ShowWithoutActivating, true);
    settings_drawer_toggle_button_->setParent(this, Qt::Tool | Qt::FramelessWindowHint);
    settings_drawer_toggle_button_->setText("Show Settings");
    settings_drawer_toggle_button_->setCursor(Qt::PointingHandCursor);
    settings_drawer_toggle_button_->setStyleSheet(
        "QPushButton{background:rgba(9,20,35,200);color:#eaf2ff;border:1px solid #4e6d90;border-radius:6px;padding:4px 10px;}"
        "QPushButton:hover{background:rgba(20,35,58,220);}");
    connect(settings_drawer_toggle_button_, &QPushButton::clicked, this, [this]() {
        settings_drawer_expanded_ = !settings_drawer_expanded_;
        if (settings_drawer_) {
            settings_drawer_->setVisible(settings_drawer_expanded_);
        }
        if (settings_drawer_toggle_button_) {
            settings_drawer_toggle_button_->setText(settings_drawer_expanded_ ? "Hide Settings" : "Show Settings");
        }
        updateSettingsDrawerGeometry();
    });

    info_drawer_ = new InfoDrawerFrame(this);
    info_drawer_->setWindowFlags(Qt::Tool | Qt::FramelessWindowHint);
    info_drawer_->setAttribute(Qt::WA_ShowWithoutActivating, true);
    info_drawer_->setAttribute(Qt::WA_TranslucentBackground, true);
    info_drawer_->setAttribute(Qt::WA_StyledBackground, true);
    info_drawer_->setStyleSheet(
        "QFrame{background:rgba(0,0,0,0);border:none;}");
    auto* drawer_layout = new QVBoxLayout(info_drawer_);
    drawer_layout->setContentsMargins(kInfoDrawerInnerPadding, kInfoDrawerInnerPadding, kInfoDrawerInnerPadding, kInfoDrawerInnerPadding);
    drawer_layout->setSpacing(0);
    status_label_ = new QLabel(info_drawer_);
    status_label_->setWordWrap(true);
    status_label_->setStyleSheet(
        "QLabel{background:rgba(9,20,35,220);border:none;border-radius:6px;color:#d9e7ff;padding:6px;}");
    drawer_layout->addWidget(status_label_);

    info_drawer_toggle_button_ = new QPushButton();
    info_drawer_toggle_button_->setAttribute(Qt::WA_ShowWithoutActivating, true);
    info_drawer_toggle_button_->setParent(this, Qt::Tool | Qt::FramelessWindowHint);
    info_drawer_toggle_button_->setText("Show Info");
    info_drawer_toggle_button_->setCursor(Qt::PointingHandCursor);
    info_drawer_toggle_button_->setStyleSheet(
        "QPushButton{background:rgba(9,20,35,200);color:#eaf2ff;border:1px solid #4e6d90;border-radius:6px;padding:4px 10px;}"
        "QPushButton:hover{background:rgba(20,35,58,220);}");
    connect(info_drawer_toggle_button_, &QPushButton::clicked, this, [this]() {
        info_drawer_expanded_ = !info_drawer_expanded_;
        if (info_drawer_) {
            info_drawer_->setVisible(info_drawer_expanded_);
        }
        if (info_drawer_toggle_button_) {
            info_drawer_toggle_button_->setText(info_drawer_expanded_ ? "Hide Info" : "Show Info");
        }
        updateInfoDrawerGeometry();
    });

    updateInfoDrawerGeometry();
    updateSettingsDrawerGeometry();
}

void PointCloudViewerWindow::moveEvent(QMoveEvent* event) {
    QWidget::moveEvent(event);
    updateInfoDrawerGeometry();
    updateSettingsDrawerGeometry();
}

void PointCloudViewerWindow::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    updateInfoDrawerGeometry();
    updateSettingsDrawerGeometry();
}

void PointCloudViewerWindow::showEvent(QShowEvent* event) {
    QWidget::showEvent(event);
    QWidget* host = window();
    if (host != tracked_host_window_) {
        if (tracked_host_window_) {
            tracked_host_window_->removeEventFilter(this);
        }
        tracked_host_window_ = host;
        if (tracked_host_window_) {
            tracked_host_window_->installEventFilter(this);
        }
    }
    updateInfoDrawerGeometry();
    updateSettingsDrawerGeometry();
    if (settings_drawer_toggle_button_) {
        settings_drawer_toggle_button_->show();
        settings_drawer_toggle_button_->raise();
    }
    if (settings_drawer_) {
        settings_drawer_->setVisible(settings_drawer_expanded_);
        settings_drawer_->raise();
    }
    if (info_drawer_toggle_button_) {
        info_drawer_toggle_button_->show();
        info_drawer_toggle_button_->raise();
    }
    if (info_drawer_) {
        info_drawer_->setVisible(info_drawer_expanded_);
        info_drawer_->raise();
    }
}

void PointCloudViewerWindow::hideEvent(QHideEvent* event) {
    QWidget::hideEvent(event);
    if (info_drawer_) {
        info_drawer_->hide();
    }
    if (info_drawer_toggle_button_) {
        info_drawer_toggle_button_->hide();
    }
    if (settings_drawer_) {
        settings_drawer_->hide();
    }
    if (settings_drawer_toggle_button_) {
        settings_drawer_toggle_button_->hide();
    }
}

bool PointCloudViewerWindow::eventFilter(QObject* watched, QEvent* event) {
    if (watched == tracked_host_window_) {
        switch (event->type()) {
        case QEvent::Move:
        case QEvent::Resize:
        case QEvent::WindowStateChange:
        case QEvent::Show:
            updateInfoDrawerGeometry();
            updateSettingsDrawerGeometry();
            break;
        case QEvent::Hide:
            if (info_drawer_) info_drawer_->hide();
            if (info_drawer_toggle_button_) info_drawer_toggle_button_->hide();
            if (settings_drawer_) settings_drawer_->hide();
            if (settings_drawer_toggle_button_) settings_drawer_toggle_button_->hide();
            break;
        default:
            break;
        }
    }
    return QWidget::eventFilter(watched, event);
}

void PointCloudViewerWindow::updateSettingsDrawerGeometry() {
    if (!settings_drawer_ || !settings_drawer_toggle_button_) {
        return;
    }
    if (!isVisible() || !window() || !window()->isVisible()) {
        settings_drawer_->setVisible(false);
        settings_drawer_toggle_button_->setVisible(false);
        return;
    }
    const int drawer_w = kSettingsDrawerWidth;
    const int drawer_h = std::max(240, std::min(height() - 2 * kSettingsDrawerMargin - kSettingsDrawerToggleHeight - kSettingsDrawerToggleGap, 520));
    const int drawer_x = kSettingsDrawerMargin;
    const int drawer_y = kSettingsDrawerMargin + kSettingsDrawerToggleHeight + kSettingsDrawerToggleGap;
    const int button_x = kSettingsDrawerMargin;
    const int button_y = kSettingsDrawerMargin;
    const QPoint top_left = mapToGlobal(QPoint(0, 0));
    settings_drawer_->setGeometry(top_left.x() + drawer_x, top_left.y() + drawer_y, drawer_w, drawer_h);
    settings_drawer_toggle_button_->setGeometry(
        top_left.x() + button_x, top_left.y() + button_y, kSettingsDrawerToggleWidth, kSettingsDrawerToggleHeight);
    settings_drawer_->setVisible(isVisible() && settings_drawer_expanded_);
    settings_drawer_toggle_button_->setVisible(isVisible());
    settings_drawer_->raise();
    settings_drawer_toggle_button_->raise();
}

void PointCloudViewerWindow::updateInfoDrawerGeometry() {
    if (!info_drawer_ || !info_drawer_toggle_button_) {
        return;
    }
    if (!isVisible() || !window() || !window()->isVisible()) {
        info_drawer_->setVisible(false);
        info_drawer_toggle_button_->setVisible(false);
        return;
    }
    const int button_x = (width() - kInfoDrawerToggleWidth) / 2;
    const int drawer_w = std::max(120, width() - 2 * kInfoDrawerMargin);
    const int text_w = std::max(80, drawer_w - 2 * kInfoDrawerInnerPadding - 12);
    int drawer_h = kInfoDrawerMinHeight;
    if (status_label_) {
        const QFontMetrics fm(status_label_->font());
        const QRect text_rect = fm.boundingRect(QRect(0, 0, text_w, 100000), Qt::TextWordWrap, status_label_->text());
        drawer_h = text_rect.height() + 2 * kInfoDrawerInnerPadding + 12;
    }
    const int max_drawer_h = std::max(kInfoDrawerMinHeight, height() - 2 * kInfoDrawerMargin - kInfoDrawerToggleHeight - kInfoDrawerToggleGap);
    drawer_h = std::clamp(drawer_h, kInfoDrawerMinHeight, max_drawer_h);
    const int drawer_x = (width() - drawer_w) / 2;
    const int drawer_y = std::max(kInfoDrawerMargin, height() - drawer_h - kInfoDrawerMargin);
    const int button_y = info_drawer_expanded_
        ? std::max(kInfoDrawerMargin, drawer_y - kInfoDrawerToggleHeight - kInfoDrawerToggleGap)
        : std::max(kInfoDrawerMargin, height() - kInfoDrawerToggleHeight - kInfoDrawerMargin);
    const QPoint top_left = mapToGlobal(QPoint(0, 0));
    info_drawer_->setGeometry(top_left.x() + drawer_x, top_left.y() + drawer_y, drawer_w, drawer_h);
    info_drawer_toggle_button_->setGeometry(
        top_left.x() + button_x, top_left.y() + button_y, kInfoDrawerToggleWidth, kInfoDrawerToggleHeight);
    info_drawer_->setVisible(isVisible() && info_drawer_expanded_);
    info_drawer_toggle_button_->setVisible(isVisible());
    info_drawer_->raise();
    info_drawer_toggle_button_->raise();
}

void PointCloudViewerWindow::setRangeMeters(double range_m) {
    if (!range_m_) {
        return;
    }
    QSignalBlocker blocker(range_m_);
    range_m_->setValue(range_m);
}

void PointCloudViewerWindow::buildPointCloudScene() {
    scene_root_ = new osg::Group();
    point_geode_ = new osg::Geode();
    point_geometry_ = new osg::Geometry();
    point_vertices_ = new osg::Vec3Array();
    point_colors_ = new osg::Vec4Array();
    point_colors_->push_back(osg::Vec4(0.2f, 0.95f, 1.0f, 1.0f));

    point_geometry_->setVertexArray(point_vertices_);
    point_geometry_->setColorArray(point_colors_, osg::Array::BIND_PER_VERTEX);
    point_geometry_->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, 0));
    point_geode_->addDrawable(point_geometry_);
    osg::StateSet* point_state = point_geode_->getOrCreateStateSet();
    point_state->setAttributeAndModes(new osg::Point(3.0f), osg::StateAttribute::ON);
    point_state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    overlay_geode_ = new osg::Geode();
    overlay_geometry_ = new osg::Geometry();
    overlay_vertices_ = new osg::Vec3Array();
    overlay_colors_ = new osg::Vec4Array();
    overlay_geometry_->setVertexArray(overlay_vertices_);
    overlay_geometry_->setColorArray(overlay_colors_, osg::Array::BIND_OVERALL);
    overlay_colors_->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    overlay_geometry_->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 0));
    overlay_geode_->addDrawable(overlay_geometry_);
    osg::StateSet* overlay_state = overlay_geode_->getOrCreateStateSet();
    overlay_state->setAttributeAndModes(new osg::LineWidth(1.5f), osg::StateAttribute::ON);
    overlay_state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    osg::ref_ptr<osg::Geode> axis_geode = new osg::Geode();
    osg::ref_ptr<osg::Geometry> axis = new osg::Geometry();
    osg::ref_ptr<osg::Vec3Array> axis_vertices = new osg::Vec3Array();
    osg::ref_ptr<osg::Vec4Array> axis_colors = new osg::Vec4Array();
    const float axis_len = 2.5f;
    axis_vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
    axis_vertices->push_back(osg::Vec3(axis_len, 0.0f, 0.0f)); // X forward
    axis_vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
    axis_vertices->push_back(osg::Vec3(0.0f, axis_len, 0.0f)); // Y left
    axis_vertices->push_back(osg::Vec3(0.0f, 0.0f, 0.0f));
    axis_vertices->push_back(osg::Vec3(0.0f, 0.0f, axis_len)); // Z up
    axis_colors->push_back(osg::Vec4(1.0f, 0.2f, 0.2f, 1.0f));
    axis_colors->push_back(osg::Vec4(1.0f, 0.2f, 0.2f, 1.0f));
    axis_colors->push_back(osg::Vec4(0.2f, 1.0f, 0.2f, 1.0f));
    axis_colors->push_back(osg::Vec4(0.2f, 1.0f, 0.2f, 1.0f));
    axis_colors->push_back(osg::Vec4(0.2f, 0.4f, 1.0f, 1.0f));
    axis_colors->push_back(osg::Vec4(0.2f, 0.4f, 1.0f, 1.0f));
    axis->setVertexArray(axis_vertices);
    axis->setColorArray(axis_colors, osg::Array::BIND_PER_VERTEX);
    axis->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, static_cast<GLsizei>(axis_vertices->size())));
    axis_geode->addDrawable(axis);
    osg::StateSet* axis_state = axis_geode->getOrCreateStateSet();
    axis_state->setAttributeAndModes(new osg::LineWidth(2.0f), osg::StateAttribute::ON);
    axis_state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    scene_root_->addChild(axis_geode);
    scene_root_->addChild(point_geode_);
    scene_root_->addChild(overlay_geode_);
    viewer_.setSceneData(scene_root_);
    if (!has_external_initial_view_) {
        if (auto* tb = dynamic_cast<osgGA::TrackballManipulator*>(viewer_.getCameraManipulator())) {
        // Default home uses a flipped up-axis baseline (reported roll: -180deg).
        tb->setHomePosition(osg::Vec3d(-8.0, -8.0, 5.0), osg::Vec3d(0.0, 0.0, 0.0), osg::Vec3d(0.0, 0.0, -1.0));
        tb->home(0.0);
    }
    }
}

void PointCloudViewerWindow::setInitialViewFromPose(const Eigen::Affine3d& pose_world) {
    (void)pose_world;
    auto* tb = dynamic_cast<osgGA::TrackballManipulator*>(viewer_.getCameraManipulator());
    if (!tb) {
        return;
    }

    // Point-cloud display uses camera-local frame; keep startup view fixed at 0/0/-180.
    const Eigen::Vector3d forward = Eigen::Vector3d::UnitX();
    const Eigen::Vector3d up = -Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d center = Eigen::Vector3d::Zero();
    const Eigen::Vector3d eye = center - forward * 6.0;

    const osg::Vec3d eye_osg(eye.x(), eye.y(), eye.z());
    const osg::Vec3d center_osg(center.x(), center.y(), center.z());
    const osg::Vec3d up_osg(up.x(), up.y(), up.z());
    tb->setHomePosition(eye_osg, center_osg, up_osg);
    tb->setTransformation(eye_osg, center_osg, up_osg);
    has_external_initial_view_ = true;
    has_auto_framed_ = true;
}

void PointCloudViewerWindow::setConfig(const PointCloudSonarConfig& cfg) {
    range_m_->setValue(cfg.range_m);
    frequency_khz_->setValue(cfg.frequency_khz);
    bandwidth_khz_->setValue(cfg.bandwidth_khz);
    horizontal_res_deg_->setValue(cfg.horizontal_angle_resolution_deg);
    vertical_res_deg_->setValue(cfg.vertical_angle_resolution_deg);
    horizontal_fov_deg_->setValue(cfg.horizontal_fov_deg);
    vertical_fov_deg_->setValue(cfg.vertical_fov_deg);
    const int combo_idx = palette_combo_->findData(cfg.palette_index);
    if (combo_idx >= 0) {
        palette_combo_->setCurrentIndex(combo_idx);
    }
    show_coordinate_overlay_->setChecked(cfg.show_coordinate_overlay);
    tcp_output_enabled_->setChecked(cfg.tcp_output_enabled);
    file_output_enabled_->setChecked(cfg.file_output_enabled);
    tcp_host_->setText(QString::fromStdString(cfg.tcp_host));
    tcp_port_->setValue(static_cast<int>(cfg.tcp_port));
    if (overlay_geode_) {
        overlay_geode_->setNodeMask(cfg.show_coordinate_overlay ? ~0u : 0u);
    }
    rebuildPalette(cfg.palette_index);
    config_dirty_ = false;
}

PointCloudSonarConfig PointCloudViewerWindow::configFromUi() const {
    PointCloudSonarConfig cfg;
    cfg.enabled = true;
    cfg.range_m = range_m_->value();
    cfg.frequency_khz = frequency_khz_->value();
    cfg.bandwidth_khz = bandwidth_khz_->value();
    cfg.horizontal_angle_resolution_deg = horizontal_res_deg_->value();
    cfg.vertical_angle_resolution_deg = vertical_res_deg_->value();
    cfg.horizontal_fov_deg = horizontal_fov_deg_->value();
    cfg.vertical_fov_deg = vertical_fov_deg_->value();
    cfg.palette_index = palette_index_;
    cfg.show_coordinate_overlay = show_coordinate_overlay_->isChecked();
    cfg.tcp_output_enabled = tcp_output_enabled_->isChecked();
    cfg.file_output_enabled = file_output_enabled_->isChecked();
    cfg.tcp_host = tcp_host_->text().trimmed().isEmpty() ? "0.0.0.0" : tcp_host_->text().trimmed().toStdString();
    cfg.tcp_port = static_cast<std::uint16_t>(tcp_port_->value());
    return cfg;
}

bool PointCloudViewerWindow::consumePendingConfig(PointCloudSonarConfig& cfg) {
    if (!config_dirty_) {
        return false;
    }
    cfg = configFromUi();
    config_dirty_ = false;
    return true;
}

void PointCloudViewerWindow::updatePointCloudFrame(const PointCloudFrame& frame) {
    static std::uint64_t update_counter = 0;
    ++update_counter;
    point_vertices_->clear();
    point_vertices_->reserve(frame.points_world.size());
    point_colors_->clear();
    point_colors_->reserve(frame.points_world.size());
    const Eigen::Vector3d t = frame.pose_position_world;
    const Eigen::Matrix3d Rt = frame.pose_rotation_world.transpose();
    for (const osg::Vec3f& p : frame.points_world) {
        // Display point cloud in camera-local frame:
        // p_local = R^T * (p_world - t_camera)
        const Eigen::Vector3d pw(static_cast<double>(p.x()), static_cast<double>(p.y()), static_cast<double>(p.z()));
        const Eigen::Vector3d pl = Rt * (pw - t);
        point_vertices_->push_back(osg::Vec3f(static_cast<float>(pl.x()),
                                              static_cast<float>(pl.y()),
                                              static_cast<float>(pl.z())));
    }
    float min_i = std::numeric_limits<float>::max();
    float max_i = std::numeric_limits<float>::lowest();
    double sum_i = 0.0;
    const std::size_t intensity_count = frame.point_intensities.size();
    for (std::size_t i = 0; i < intensity_count; ++i) {
        const float x = std::clamp(frame.point_intensities[i], 0.0f, 1.0f);
        min_i = std::min(min_i, x);
        max_i = std::max(max_i, x);
        sum_i += static_cast<double>(x);
    }
    if (intensity_count == 0) {
        min_i = 0.0f;
        max_i = 0.0f;
    }
    for (std::size_t i = 0; i < frame.points_world.size(); ++i) {
        const float raw_intensity = (i < intensity_count) ? frame.point_intensities[i] : 0.0f;
        // Keep color mapping identical to Rock SonarWidget SonarPlot:
        // QColor c = colorMap[round(intensity * 255)].
        const float normalized = std::clamp(raw_intensity, 0.0f, 1.0f);
        const int palette_idx = std::clamp(static_cast<int>(std::lround(normalized * 255.0f)), 0, 255);
        if (!palette_.empty()) {
            point_colors_->push_back(palette_[palette_idx]);
        } else {
            point_colors_->push_back(osg::Vec4(normalized, normalized, normalized, 1.0f));
        }
    }
    auto* draw = dynamic_cast<osg::DrawArrays*>(point_geometry_->getPrimitiveSet(0));
    if (draw) {
        draw->setCount(static_cast<GLsizei>(point_vertices_->size()));
    }
    point_geometry_->dirtyDisplayList();
    point_geometry_->dirtyBound();
    point_vertices_->dirty();
    point_colors_->dirty();
    updateCoordinateOverlay(frame);
    autoFrameCameraToPoints();

    updateStatusLabel(frame);
    (void)update_counter;
    (void)min_i;
    (void)max_i;
    (void)sum_i;
    (void)intensity_count;
}

void PointCloudViewerWindow::updateCoordinateOverlay(const PointCloudFrame& frame) {
    if (!overlay_geometry_ || !overlay_vertices_) {
        return;
    }
    if (!show_coordinate_overlay_->isChecked()) {
        auto* draw_empty = dynamic_cast<osg::DrawArrays*>(overlay_geometry_->getPrimitiveSet(0));
        if (draw_empty) {
            draw_empty->setCount(0);
        }
        overlay_geometry_->dirtyDisplayList();
        overlay_geometry_->dirtyBound();
        overlay_vertices_->clear();
        overlay_vertices_->dirty();
        return;
    }

    constexpr double kPi = 3.14159265358979323846;
    const double range_m = std::max(0.1, frame.config.range_m);
    const double hfov = frame.config.horizontal_fov_deg * kPi / 180.0;
    const double vfov = frame.config.vertical_fov_deg * kPi / 180.0;
    const double hy = hfov * 0.5;
    const double vz = vfov * 0.5;

    auto toLocal = [](const Eigen::Vector3d& local) {
        return osg::Vec3(static_cast<float>(local.x()),
                         static_cast<float>(local.y()),
                         static_cast<float>(local.z()));
    };
    auto dirFromYawPitch = [](double yaw, double pitch) -> Eigen::Vector3d {
        const double cp = std::cos(pitch);
        Eigen::Vector3d v(std::cos(yaw) * cp, std::sin(yaw) * cp, std::sin(pitch));
        if (v.norm() < 1e-9) {
            return Eigen::Vector3d::UnitX();
        }
        return v.normalized();
    };

    overlay_vertices_->clear();
    // Distance center line + horizontal/vertical boundary rays (white)
    const Eigen::Vector3d d_center = dirFromYawPitch(0.0, 0.0);
    const Eigen::Vector3d d_h_l = dirFromYawPitch(+hy, 0.0);
    const Eigen::Vector3d d_h_r = dirFromYawPitch(-hy, 0.0);
    const Eigen::Vector3d d_v_u = dirFromYawPitch(0.0, +vz);
    const Eigen::Vector3d d_v_d = dirFromYawPitch(0.0, -vz);
    const std::array<Eigen::Vector3d, 5> rays{d_center, d_h_l, d_h_r, d_v_u, d_v_d};
    for (const auto& d : rays) {
        overlay_vertices_->push_back(toLocal(Eigen::Vector3d::Zero()));
        overlay_vertices_->push_back(toLocal(d * range_m));
    }

    // Horizontal angle arc at max range (x-y local plane)
    constexpr int kArcSegments = 48;
    for (int i = 0; i < kArcSegments; ++i) {
        const double t0 = static_cast<double>(i) / static_cast<double>(kArcSegments);
        const double t1 = static_cast<double>(i + 1) / static_cast<double>(kArcSegments);
        const double yaw0 = -hy + (2.0 * hy) * t0;
        const double yaw1 = -hy + (2.0 * hy) * t1;
        const Eigen::Vector3d p0 = dirFromYawPitch(yaw0, 0.0) * range_m;
        const Eigen::Vector3d p1 = dirFromYawPitch(yaw1, 0.0) * range_m;
        overlay_vertices_->push_back(toLocal(p0));
        overlay_vertices_->push_back(toLocal(p1));
    }

    // Vertical angle arc at max range (x-z local plane)
    for (int i = 0; i < kArcSegments; ++i) {
        const double t0 = static_cast<double>(i) / static_cast<double>(kArcSegments);
        const double t1 = static_cast<double>(i + 1) / static_cast<double>(kArcSegments);
        const double p0 = -vz + (2.0 * vz) * t0;
        const double p1 = -vz + (2.0 * vz) * t1;
        const Eigen::Vector3d v0 = dirFromYawPitch(0.0, p0) * range_m;
        const Eigen::Vector3d v1 = dirFromYawPitch(0.0, p1) * range_m;
        overlay_vertices_->push_back(toLocal(v0));
        overlay_vertices_->push_back(toLocal(v1));
    }

    auto* draw = dynamic_cast<osg::DrawArrays*>(overlay_geometry_->getPrimitiveSet(0));
    if (draw) {
        draw->setCount(static_cast<GLsizei>(overlay_vertices_->size()));
    }
    overlay_geometry_->dirtyDisplayList();
    overlay_geometry_->dirtyBound();
    overlay_vertices_->dirty();
}

void PointCloudViewerWindow::autoFrameCameraToPoints() {
    if (has_external_initial_view_) {
        return;
    }
    if (!point_vertices_ || point_vertices_->empty()) {
        return;
    }
    auto* tb = dynamic_cast<osgGA::TrackballManipulator*>(viewer_.getCameraManipulator());
    if (!tb) {
        return;
    }

    osg::Vec3d pmin(std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max());
    osg::Vec3d pmax(std::numeric_limits<double>::lowest(),
                    std::numeric_limits<double>::lowest(),
                    std::numeric_limits<double>::lowest());
    for (const osg::Vec3f& p : *point_vertices_) {
        pmin.x() = std::min(pmin.x(), static_cast<double>(p.x()));
        pmin.y() = std::min(pmin.y(), static_cast<double>(p.y()));
        pmin.z() = std::min(pmin.z(), static_cast<double>(p.z()));
        pmax.x() = std::max(pmax.x(), static_cast<double>(p.x()));
        pmax.y() = std::max(pmax.y(), static_cast<double>(p.y()));
        pmax.z() = std::max(pmax.z(), static_cast<double>(p.z()));
    }
    const osg::Vec3d center = (pmin + pmax) * 0.5;
    const osg::Vec3d extents = (pmax - pmin) * 0.5;
    const double radius = std::max(1.0, extents.length());
    const osg::Vec3d eye = center + osg::Vec3d(-2.5 * radius, -2.2 * radius, 1.4 * radius);
    // Keep auto-framed view consistent with the flipped up-axis baseline.
    const osg::Vec3d up(0.0, 0.0, -1.0);

    tb->setTransformation(eye, center, up);
    if (!has_auto_framed_) {
        tb->setHomePosition(eye, center, up);
        has_auto_framed_ = true;
    }
}

void PointCloudViewerWindow::rebuildPalette(int palette_index) {
    const int clamped = std::clamp(palette_index, 0, 3);
    palette_index_ = clamped;
    palette_.clear();
    palette_.reserve(256);
    color_gradient_.colormapSelector(static_cast<sonar_palette::PaletteType>(clamped));
    for (int i = 0; i < 256; ++i) {
        float r = 0.0f;
        float g = 0.0f;
        float b = 0.0f;
        try {
            color_gradient_.getColorAtValue((1.0f / 255.0f) * static_cast<float>(i), r, g, b);
            palette_.emplace_back(r, g, b, 1.0f);
        } catch (const std::out_of_range&) {
            palette_.emplace_back(1.0f, 1.0f, 1.0f, 1.0f);
        }
    }
    if (auto* colorbar = dynamic_cast<PointCloudColorBar*>(palette_colorbar_)) {
        colorbar->setPaletteColors(palette_);
    }
}

void PointCloudViewerWindow::renderFrame() {
    if (!isVisible() || !window() || !window()->isVisible()) {
        return;
    }
    // Keep floating tool windows aligned with host window even when move events
    // don't reach this page (e.g. inside tabbed containers).
    updateInfoDrawerGeometry();
    updateSettingsDrawerGeometry();
    if (render_blocked_by_main_viewer_) {
        return;
    }
    if (!canRenderFrame()) {
        return;
    }
    syncEmbeddedViewport();
    viewer_.frame();
}

bool PointCloudViewerWindow::canRenderFrame() const {
    if (!isVisible()) {
        return false;
    }
    const osg::Camera* cam = viewer_.getCamera();
    if (!cam) {
        return false;
    }
    const osg::GraphicsContext* gc = cam->getGraphicsContext();
    if (!gc) {
        return false;
    }
    return true;
}

void PointCloudViewerWindow::syncEmbeddedViewport() {
    if (!osg_container_) {
        return;
    }
    const int w = std::max(1, osg_container_->width());
    const int h = std::max(1, osg_container_->height());
    if (w == last_viewport_w_ && h == last_viewport_h_) {
        return;
    }

    osg::Camera* cam = viewer_.getCamera();
    if (!cam) {
        return;
    }
    cam->setViewport(new osg::Viewport(0, 0, w, h));
    const double aspect = static_cast<double>(w) / static_cast<double>(h);
    cam->setProjectionMatrixAsPerspective(45.0, aspect, 0.05, 20000.0);

    if (auto* gw = dynamic_cast<osgViewer::GraphicsWindow*>(cam->getGraphicsContext())) {
        gw->resized(0, 0, w, h);
        if (gw->getEventQueue()) {
            gw->getEventQueue()->windowResize(0, 0, w, h);
        }
    }

    last_viewport_w_ = w;
    last_viewport_h_ = h;
}

void PointCloudViewerWindow::markConfigDirty() {
    config_dirty_ = true;
}

void PointCloudViewerWindow::updateStatusLabel(const PointCloudFrame& frame) {
    const auto& c = frame.config;
    const auto& s = frame.sampling;
    const ViewAttitude view = currentViewAttitude(viewer_);
    status_label_->setText(
        QString("Coordinate Frame: %1 (display: camera_local)\nPoints: %2 (requested %3, budget %4)\nRange: %5 m\n"
                "FOV H/V: %6 / %7 deg\nRes H/V: %8 / %9 deg\nFrequency/Bandwidth: %10 / %11 kHz\n"
                "Samples HxV: %12 x %13, rounding=%14\nPalette: %15  Speckle: %16  Attenuation: %17\nPose xyz: (%18, %19, %20)\n"
                "View Yaw/Pitch/Roll (deg): (%21, %22, %23)\nView Quaternion (wxyz): (%24, %25, %26, %27)")
            .arg(QString::fromStdString(frame.coordinate_frame))
            .arg(static_cast<qulonglong>(s.recovered_point_count))
            .arg(static_cast<qulonglong>(s.requested_point_count))
            .arg(static_cast<qulonglong>(s.budgeted_point_count))
            .arg(c.range_m, 0, 'f', 2)
            .arg(c.horizontal_fov_deg, 0, 'f', 2)
            .arg(c.vertical_fov_deg, 0, 'f', 2)
            .arg(c.horizontal_angle_resolution_deg, 0, 'f', 3)
            .arg(c.vertical_angle_resolution_deg, 0, 'f', 3)
            .arg(c.frequency_khz, 0, 'f', 1)
            .arg(c.bandwidth_khz, 0, 'f', 1)
            .arg(static_cast<qulonglong>(s.horizontal_samples))
            .arg(static_cast<qulonglong>(s.vertical_samples))
            .arg(QString::fromStdString(s.rounding_policy))
            .arg(c.palette_index)
            .arg(c.enable_speckle ? "On" : "Off")
            .arg(c.enable_attenuation ? "On" : "Off")
            .arg(0.0, 0, 'f', 2)
            .arg(0.0, 0, 'f', 2)
            .arg(0.0, 0, 'f', 2)
            .arg(view.yaw_deg, 0, 'f', 2)
            .arg(view.pitch_deg, 0, 'f', 2)
            .arg(view.roll_deg, 0, 'f', 2)
            .arg(view.q.w(), 0, 'f', 4)
            .arg(view.q.x(), 0, 'f', 4)
            .arg(view.q.y(), 0, 'f', 4)
            .arg(view.q.z(), 0, 'f', 4));
}

void PointCloudViewerWindow::setTcpRuntimeStatus(bool running,
                                                 bool client_connected,
                                                 std::uint64_t last_seq,
                                                 std::size_t last_payload_bytes) {
    const QString append =
        QString("\nTCP: %1  Client: %2  Last Seq: %3  Last Payload: %4 bytes")
            .arg(running ? "running" : "stopped")
            .arg(client_connected ? "connected" : "disconnected")
            .arg(static_cast<qulonglong>(last_seq))
            .arg(static_cast<qulonglong>(last_payload_bytes));
    if (!status_label_) {
        return;
    }
    const QString current = status_label_->text();
    const int tcp_pos = current.indexOf("\nTCP:");
    if (tcp_pos >= 0) {
        status_label_->setText(current.left(tcp_pos) + append);
    } else {
        status_label_->setText(current + append);
    }
}

} // namespace standalone_mvp
