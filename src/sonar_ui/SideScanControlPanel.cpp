#include "SideScanControlPanel.hpp"
#include "SideScanWaterfallCanvas.hpp"

#include <QDoubleSpinBox>
#include <QFrame>
#include <QFormLayout>
#include <QPainter>
#include <QSignalBlocker>
#include <QSpinBox>
#include <sonar_palette/PaletteRamp.hpp>
#include <sonar_palette/PaletteTypes.hpp>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#define ROCK_PALETTE_TEXT_ROLE QPalette::WindowText
#define ROCK_PALETTE_FILL_ROLE QPalette::Window
#else
#define ROCK_PALETTE_TEXT_ROLE QPalette::Foreground
#define ROCK_PALETTE_FILL_ROLE QPalette::Background
#endif

namespace {
constexpr int kPanelMargin = 10;
constexpr int kAdvancedPanelWidth = 300;
constexpr int kAdvancedDrawerToggleWidth = 140;
constexpr int kAdvancedDrawerToggleHeight = 28;
constexpr int kAdvancedDrawerToggleGap = 6;
constexpr int kSonarFrameMargin = 6;
constexpr int kSonarToColorbarGap = 8;
constexpr int kColorbarWidth = 54;
constexpr int kControlRowHeight = 30;
constexpr int kControlRowSpacing = 8;
constexpr int kBottomPanelHeight = 70;

QDoubleSpinBox* makeDouble(double min_v, double max_v, double step, int decimals = 2) {
    auto* box = new QDoubleSpinBox();
    box->setRange(min_v, max_v);
    box->setSingleStep(step);
    box->setDecimals(decimals);
    return box;
}

class SideScanColorBarWidget final : public QWidget {
public:
    explicit SideScanColorBarWidget(QWidget* parent = nullptr) : QWidget(parent) {
        setMinimumWidth(kColorbarWidth);
        setPaletteIndex(1);
    }

    void setPaletteIndex(int idx) {
        const int clamped = std::clamp(idx, 0, 3);
        colors_.clear();
        colors_.reserve(256);
        sonar_palette::PaletteRamp gradient;
        gradient.colormapSelector(static_cast<sonar_palette::PaletteType>(clamped));
        for (int i = 0; i < 256; ++i) {
            float r = 0.0f;
            float g = 0.0f;
            float b = 0.0f;
            try {
                gradient.getColorAtValue((1.0f / 255.0f) * static_cast<float>(i), r, g, b);
                colors_.emplace_back(
                    static_cast<int>(std::clamp(r, 0.0f, 1.0f) * 255.0f),
                    static_cast<int>(std::clamp(g, 0.0f, 1.0f) * 255.0f),
                    static_cast<int>(std::clamp(b, 0.0f, 1.0f) * 255.0f));
            } catch (const std::out_of_range&) {
                colors_.emplace_back(Qt::white);
            }
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
        if (color_count > 0) {
            for (int y = 0; y < bar_rect.height(); ++y) {
                const double t = 1.0 - static_cast<double>(y) / static_cast<double>(std::max(1, bar_rect.height() - 1));
                const int idx = std::clamp(static_cast<int>(std::lround(t * static_cast<double>(color_count - 1))), 0, color_count - 1);
                painter.setPen(colors_[static_cast<size_t>(idx)]);
                painter.drawLine(bar_rect.left(), bar_rect.top() + y, bar_rect.right(), bar_rect.top() + y);
            }
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
} // namespace

SideScanControlPanel::SideScanControlPanel(QWidget* parent) : QWidget(parent) {
    setMinimumSize(320, 220);
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

    sonarDisplayFrame = new QFrame(this);
    sonarDisplayFrame->setStyleSheet("QFrame{background:#02050a;border:1px solid #4e6d90;}");
    plot = new SideScanWaterfallCanvas(sonarDisplayFrame);
    sonarColorBar = new SideScanColorBarWidget(sonarDisplayFrame);
    connect(this, SIGNAL(rangeChanged(int)), plot, SLOT(rangeChanged(int)));
    connect(this, SIGNAL(sonarPaletteChanged(int)), plot, SLOT(sonarPaletteChanged(int)));
    connect(this, SIGNAL(gridChanged(bool)), plot, SLOT(gridChanged(bool)));

    QPalette widgetPalette(palette());
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    widgetPalette.setColor(QPalette::Window, plot->palette().color(QPalette::Window));
#else
    widgetPalette.setColor(QPalette::Background, plot->palette().color(QPalette::Background));
#endif
    setAutoFillBackground(true);
    setPalette(widgetPalette);

    createAdvancedSettingsPanel();
    createGainComponent();
    createRangeComponent();
    createPaletteComponent();
    createGridComponent();

    advancedDrawerToggleButton = new QPushButton(this);
    advancedDrawerToggleButton->setText("Show Settings");
    advancedDrawerToggleButton->setCursor(Qt::PointingHandCursor);
    advancedDrawerToggleButton->setStyleSheet(
        "QPushButton{background:rgba(9,20,35,200);color:#eaf2ff;border:1px solid #4e6d90;border-radius:6px;padding:4px 10px;}"
        "QPushButton:hover{background:rgba(20,35,58,220);}");
    connect(advancedDrawerToggleButton, &QPushButton::clicked, [this]() {
        advancedDrawerVisible = !advancedDrawerVisible;
        if (advancedDrawerToggleButton) {
            advancedDrawerToggleButton->setText(advancedDrawerVisible ? "Hide Settings" : "Show Settings");
        }
        layoutChildren();
    });
    advancedDrawerToggleButton->setVisible(false);
}

SideScanControlPanel::~SideScanControlPanel() = default;

void SideScanControlPanel::layoutChildren() {
    const int leftPanelWidth = 0;
    const int plotLeft = kPanelMargin + leftPanelWidth;
    const int plotWidth = std::max(50, width() - plotLeft - kPanelMargin);
    const int plotHeight = advancedPanelEnabled
        ? std::max(50, height() - 2 * kPanelMargin)
        : std::max(50, height() - kBottomPanelHeight);
    layoutSonarDisplayArea(plotLeft, kPanelMargin, plotWidth, plotHeight);
    if (advancedPanel) {
        const bool panel_visible = advancedPanelEnabled && advancedDrawerVisible;
        advancedPanel->setVisible(panel_visible);
        const int panel_x = kPanelMargin;
        const int panel_y = kPanelMargin + kAdvancedDrawerToggleHeight + kAdvancedDrawerToggleGap;
        const int panel_h = std::max(120, height() - panel_y - kPanelMargin);
        advancedPanel->setGeometry(panel_x, panel_y, kAdvancedPanelWidth, panel_h);
        if (panel_visible) {
            advancedPanel->raise();
        }
    }
    if (advancedDrawerToggleButton) {
        advancedDrawerToggleButton->setVisible(advancedPanelEnabled);
        advancedDrawerToggleButton->setGeometry(
            kPanelMargin, kPanelMargin, kAdvancedDrawerToggleWidth, kAdvancedDrawerToggleHeight);
        if (advancedPanelEnabled) {
            advancedDrawerToggleButton->raise();
        }
    }

    const int firstRowY = height() - kBottomPanelHeight + 4;
    const int secondRowY = firstRowY + kControlRowHeight + kControlRowSpacing;
    if (advancedPanelEnabled) {
        if (lbGain) lbGain->setVisible(false);
        if (slGain) slGain->setVisible(false);
        if (edGain) edGain->setVisible(false);
        if (lbRange) lbRange->setVisible(false);
        if (slRange) slRange->setVisible(false);
        if (edRange) edRange->setVisible(false);
        if (lbGrid) lbGrid->setVisible(false);
        if (lbPalette) lbPalette->setVisible(false);
        if (boxGrid) boxGrid->setVisible(advancedDrawerVisible);
        if (comboPalette) comboPalette->setVisible(advancedDrawerVisible);
    } else {
        if (lbGain) {
            lbGain->setVisible(true);
            lbGain->setGeometry(plotLeft, firstRowY, 50, 20);
        }
        if (slGain) {
            slGain->setVisible(true);
            slGain->setGeometry(plotLeft + 60, firstRowY, 150, 20);
        }
        if (edGain) {
            edGain->setVisible(true);
            edGain->setGeometry(plotLeft + 220, firstRowY, 50, 20);
        }

        if (lbRange) {
            lbRange->setVisible(true);
            lbRange->setGeometry(plotLeft, secondRowY, 50, 20);
        }
        if (slRange) {
            slRange->setVisible(true);
            slRange->setGeometry(plotLeft + 60, secondRowY, 150, 20);
        }
        if (edRange) {
            edRange->setVisible(true);
            edRange->setGeometry(plotLeft + 220, secondRowY, 50, 20);
        }

        if (lbGrid) {
            lbGrid->setVisible(true);
            lbGrid->setGeometry(width() - 160, firstRowY, 50, 20);
        }
        if (boxGrid) {
            boxGrid->setVisible(true);
            boxGrid->setGeometry(width() - 100, firstRowY, 80, 20);
        }

        if (lbPalette) {
            lbPalette->setVisible(true);
            lbPalette->setGeometry(width() - 160, secondRowY, 50, 20);
        }
        if (comboPalette) {
            comboPalette->setVisible(true);
            comboPalette->setGeometry(width() - 100, secondRowY, 80, 20);
        }
    }
}

void SideScanControlPanel::layoutSonarDisplayArea(int x, int y, int w, int h) {
    if (!sonarDisplayFrame || !plot || !sonarColorBar) {
        return;
    }
    sonarDisplayFrame->setGeometry(x, y, w, h);

    const int inner_x = kSonarFrameMargin;
    const int inner_y = kSonarFrameMargin;
    const int inner_w = std::max(10, w - 2 * kSonarFrameMargin);
    const int inner_h = std::max(10, h - 2 * kSonarFrameMargin);

    const int colorbar_w = kColorbarWidth;
    const int colorbar_h = std::max(40, inner_h);
    const int colorbar_x = inner_x + std::max(0, inner_w - colorbar_w);
    const int colorbar_y = inner_y;
    sonarColorBar->setGeometry(colorbar_x, colorbar_y, colorbar_w, colorbar_h);

    const int sonar_area_w = std::max(20, inner_w - colorbar_w - kSonarToColorbarGap);
    const int sonar_area_h = inner_h;
    plot->setGeometry(inner_x, inner_y, sonar_area_w, sonar_area_h);
}

void SideScanControlPanel::resizeEvent(QResizeEvent* event) {
    layoutChildren();
    QWidget::resizeEvent(event);
}

void SideScanControlPanel::setPortStarboardData(const sonar_types_v2::samples::Sonar& port,
                                                const sonar_types_v2::samples::Sonar& starboard) {
    if (plot) {
        plot->setPortStarboardPing(port, starboard);
    }
}

void SideScanControlPanel::setGain(int value) {
    if (slGain) {
        slGain->setValue(value);
    }
    if (spGain) {
        const QSignalBlocker blocker(spGain);
        spGain->setValue(std::clamp(static_cast<double>(value) / 100.0, 0.0, 1.0));
    }
}

int SideScanControlPanel::getGain() const {
    return slGain ? slGain->value() : 0;
}

void SideScanControlPanel::setRange(int value) {
    if (slRange) {
        slRange->setValue(value);
    }
    if (spRange) {
        const QSignalBlocker blocker(spRange);
        spRange->setValue(std::max(0.1, static_cast<double>(value)));
    }
}

int SideScanControlPanel::getRange() const {
    return slRange ? slRange->value() : 0;
}

void SideScanControlPanel::setMinRange(int value) {
    if (slRange) {
        slRange->setMinimum(value);
    }
}

void SideScanControlPanel::setMaxRange(int value) {
    if (slRange) {
        slRange->setMaximum(value);
    }
    if (spRange) {
        spRange->setMaximum(static_cast<double>(value));
    }
}

void SideScanControlPanel::setSonarPalette(int value) {
    if (comboPalette) {
        comboPalette->setCurrentIndex(value);
    }
}

void SideScanControlPanel::onSlGainChanged(int value) {
    if (edGain) {
        edGain->setText(QString::number(value) + " %");
    }
    if (spGain && !syncingAdvancedControls) {
        const QSignalBlocker blocker(spGain);
        spGain->setValue(std::clamp(static_cast<double>(value) / 100.0, 0.0, 1.0));
    }
    emit gainChanged(value);
    emitAdvancedSssConfigChanged();
}

void SideScanControlPanel::onSlRangeChanged(int value) {
    if (edRange) {
        edRange->setText(QString::number(value) + " m");
    }
    if (spRange && !syncingAdvancedControls) {
        const QSignalBlocker blocker(spRange);
        spRange->setValue(std::max(0.1, static_cast<double>(value)));
    }
    emit rangeChanged(value);
    emitAdvancedSssConfigChanged();
}

void SideScanControlPanel::onComboPaletteChanged(int value) {
    emit sonarPaletteChanged(value);
    if (auto* bar = dynamic_cast<SideScanColorBarWidget*>(sonarColorBar)) {
        bar->setPaletteIndex(value);
    }
}

void SideScanControlPanel::onCheckboxGridChanged(bool value) {
    emit gridChanged(value);
}

void SideScanControlPanel::createGainComponent() {
    lbGain = new QLabel(this);
    lbGain->setText("Gain:");
    QPalette palette = lbGain->palette();
    palette.setColor(ROCK_PALETTE_TEXT_ROLE, Qt::white);
    lbGain->setPalette(palette);

    slGain = new QSlider(Qt::Horizontal, this);
    slGain->setRange(0, 100);
    slGain->setValue(50);

    edGain = new QLineEdit(this);
    edGain->setReadOnly(true);
    edGain->setAlignment(Qt::AlignRight);
    edGain->setText("50 %");
    palette = edGain->palette();
    palette.setColor(QPalette::Base, plot->palette().color(ROCK_PALETTE_FILL_ROLE));
    palette.setColor(QPalette::Text, Qt::white);
    edGain->setPalette(palette);

    connect(slGain, SIGNAL(valueChanged(int)), this, SLOT(onSlGainChanged(int)));
}

void SideScanControlPanel::createRangeComponent() {
    lbRange = new QLabel(this);
    lbRange->setText("Range:");
    QPalette palette = lbRange->palette();
    palette.setColor(ROCK_PALETTE_TEXT_ROLE, Qt::white);
    lbRange->setPalette(palette);

    slRange = new QSlider(Qt::Horizontal, this);
    slRange->setRange(1, 1000);
    slRange->setValue(30);

    edRange = new QLineEdit(this);
    edRange->setReadOnly(true);
    edRange->setAlignment(Qt::AlignRight);
    edRange->setText("30 m");
    palette = edRange->palette();
    palette.setColor(QPalette::Base, plot->palette().color(ROCK_PALETTE_FILL_ROLE));
    palette.setColor(QPalette::Text, Qt::white);
    edRange->setPalette(palette);

    connect(slRange, SIGNAL(valueChanged(int)), this, SLOT(onSlRangeChanged(int)));
}

void SideScanControlPanel::createPaletteComponent() {
    lbPalette = new QLabel(this);
    lbPalette->setText("Palette:");
    lbPalette->setAlignment(Qt::AlignRight);
    lbPalette->setStyleSheet("background-color: #505050; color: white;");

    comboPalette = new QComboBox(this);
    QPalette palette = comboPalette->palette();
    palette.setColor(comboPalette->backgroundRole(), plot->palette().color(ROCK_PALETTE_FILL_ROLE));
    palette.setColor(comboPalette->foregroundRole(), Qt::white);
    comboPalette->setPalette(palette);
    comboPalette->addItem("Jet");
    comboPalette->addItem("Hot");
    comboPalette->addItem("Gray");
    comboPalette->addItem("Bronze");

    connect(comboPalette, SIGNAL(currentIndexChanged(int)), this, SLOT(onComboPaletteChanged(int)));
}

void SideScanControlPanel::createGridComponent() {
    lbGrid = new QLabel(this);
    lbGrid->setText("Grid:");
    lbGrid->setAlignment(Qt::AlignRight);
    lbGrid->setStyleSheet("background-color: #505050; color: white;");

    boxGrid = new QCheckBox(this);
    boxGrid->setStyleSheet("background-color: #505050;");
    boxGrid->setChecked(true);

    connect(boxGrid, SIGNAL(clicked(bool)), this, SLOT(onCheckboxGridChanged(bool)));
}

void SideScanControlPanel::createAdvancedSettingsPanel() {
    advancedPanel = new QWidget(this);
    advancedPanel->setStyleSheet(
        "QWidget{background:#05090f;color:#eaf2ff;}"
        "QLabel{color:#d9e7ff;}"
        "QDoubleSpinBox,QSpinBox{background:#091423;color:#eaf2ff;border:1px solid #4e6d90;}");
    advancedForm = new QFormLayout(advancedPanel);
    advancedForm->setContentsMargins(6, 6, 6, 6);
    advancedForm->setSpacing(6);

    spRange = makeDouble(0.1, 1000.0, 0.5, 2);
    spGain = makeDouble(0.0, 1.0, 0.01, 3);
    spCenterFrequency = makeDouble(1.0, 2000.0, 1.0, 1);
    spBandwidth = makeDouble(0.1, 2000.0, 0.5, 1);
    spBeamWidth = makeDouble(0.01, 179.0, 0.1, 2);
    spBeamHeight = makeDouble(0.01, 179.0, 0.1, 2);
    spAngleResolution = makeDouble(0.01, 30.0, 0.01, 3);
    spUpdateStride = new QSpinBox(advancedPanel);
    spUpdateStride->setRange(1, 30);

    advancedForm->addRow("Range (m)", spRange);
    advancedForm->addRow("Gain", spGain);
    advancedForm->addRow("Center Frequency (kHz)", spCenterFrequency);
    advancedForm->addRow("Bandwidth (kHz)", spBandwidth);
    advancedForm->addRow("Beam Width (deg)", spBeamWidth);
    advancedForm->addRow("Beam Height (deg)", spBeamHeight);
    advancedForm->addRow("Angle Resolution (deg)", spAngleResolution);
    advancedForm->addRow("Update Stride", spUpdateStride);

    connect(spRange, SIGNAL(valueChanged(double)), this, SLOT(onAdvancedControlChanged(double)));
    connect(spGain, SIGNAL(valueChanged(double)), this, SLOT(onAdvancedControlChanged(double)));
    connect(spCenterFrequency, SIGNAL(valueChanged(double)), this, SLOT(onAdvancedControlChanged(double)));
    connect(spBandwidth, SIGNAL(valueChanged(double)), this, SLOT(onAdvancedControlChanged(double)));
    connect(spBeamWidth, SIGNAL(valueChanged(double)), this, SLOT(onAdvancedControlChanged(double)));
    connect(spBeamHeight, SIGNAL(valueChanged(double)), this, SLOT(onAdvancedControlChanged(double)));
    connect(spAngleResolution, SIGNAL(valueChanged(double)), this, SLOT(onAdvancedControlChanged(double)));
    connect(spUpdateStride, SIGNAL(valueChanged(int)), this, SLOT(onUpdateStrideChanged(int)));

    advancedPanel->setVisible(false);
}

void SideScanControlPanel::setAdvancedPanelEnabled(bool enabled) {
    advancedPanelEnabled = enabled;
    advancedDrawerVisible = false;
    if (advancedPanelEnabled) {
        ensureAdvancedFormExtraControls();
    }
    if (advancedDrawerToggleButton) {
        advancedDrawerToggleButton->setText("Show Settings");
    }
    if (advancedPanel) {
        advancedPanel->setVisible(false);
    }
    layoutChildren();
}

void SideScanControlPanel::ensureAdvancedFormExtraControls() {
    if (!advancedForm || !advancedPanel || !comboPalette || !boxGrid) {
        return;
    }
    if (!advancedFormExtraControlsAttached) {
        comboPalette->setParent(advancedPanel);
        boxGrid->setParent(advancedPanel);
        advancedForm->addRow("Palette", comboPalette);
        advancedForm->addRow("Grid", boxGrid);
        advancedFormExtraControlsAttached = true;
    }
    if (advancedFormOutputControlsAttached) {
        return;
    }
    boxTcpOutput = new QCheckBox("Enable TCP Output", advancedPanel);
    boxFileOutput = new QCheckBox("Enable File Output", advancedPanel);
    edTcpHost = new QLineEdit(advancedPanel);
    edTcpHost->setPlaceholderText("0.0.0.0");
    spTcpPort = new QSpinBox(advancedPanel);
    spTcpPort->setRange(1, 65535);
    spTcpPort->setValue(30002);
    advancedForm->addRow("", boxTcpOutput);
    advancedForm->addRow("", boxFileOutput);
    advancedForm->addRow("TCP Host", edTcpHost);
    advancedForm->addRow("TCP Port", spTcpPort);
    connect(boxTcpOutput, &QCheckBox::toggled, this, &SideScanControlPanel::onOutputControlChanged);
    connect(boxFileOutput, &QCheckBox::toggled, this, &SideScanControlPanel::onOutputControlChanged);
    connect(edTcpHost, &QLineEdit::textChanged, this, &SideScanControlPanel::onOutputControlChanged);
    connect(spTcpPort, qOverload<int>(&QSpinBox::valueChanged), this, &SideScanControlPanel::onOutputControlChanged);
    advancedFormOutputControlsAttached = true;
}

void SideScanControlPanel::updateBandwidthUpperBound() {
    if (!spCenterFrequency || !spBandwidth) {
        return;
    }
    const double max_bw = std::max(0.1, spCenterFrequency->value() - 0.1);
    spBandwidth->setMaximum(max_bw);
    if (spBandwidth->value() > max_bw) {
        const QSignalBlocker blocker(spBandwidth);
        spBandwidth->setValue(max_bw);
    }
}

void SideScanControlPanel::setAdvancedSssConfig(double range_m,
                                                double gain,
                                                double center_frequency_khz,
                                                double bandwidth_khz,
                                                double beam_width_deg,
                                                double beam_height_deg,
                                                double angle_resolution_deg,
                                                int update_stride,
                                                bool tcp_output_enabled,
                                                bool file_output_enabled,
                                                const QString& tcp_host,
                                                int tcp_port) {
    if (!spRange || !spGain || !spCenterFrequency || !spBandwidth || !spBeamWidth || !spBeamHeight ||
        !spAngleResolution || !spUpdateStride) {
        return;
    }
    if (advancedPanelEnabled) {
        ensureAdvancedFormExtraControls();
    }
    syncingAdvancedControls = true;
    {
        const QSignalBlocker b0(spRange);
        const QSignalBlocker b1(spGain);
        const QSignalBlocker b2(spCenterFrequency);
        const QSignalBlocker b3(spBandwidth);
        const QSignalBlocker b4(spBeamWidth);
        const QSignalBlocker b5(spBeamHeight);
        const QSignalBlocker b6(spAngleResolution);
        const QSignalBlocker b7(spUpdateStride);
        spRange->setValue(std::clamp(range_m, 0.1, spRange->maximum()));
        spGain->setValue(std::clamp(gain, 0.0, 1.0));
        spCenterFrequency->setValue(std::clamp(center_frequency_khz, 1.0, 2000.0));
        updateBandwidthUpperBound();
        spBandwidth->setValue(std::clamp(bandwidth_khz, 0.1, spBandwidth->maximum()));
        spBeamWidth->setValue(std::clamp(beam_width_deg, 0.01, 179.0));
        spBeamHeight->setValue(std::clamp(beam_height_deg, 0.01, 179.0));
        spAngleResolution->setValue(std::clamp(angle_resolution_deg, 0.01, 30.0));
        spUpdateStride->setValue(std::clamp(update_stride, 1, 30));
    }
    if (boxTcpOutput && boxFileOutput && edTcpHost && spTcpPort) {
        const QSignalBlocker b8(boxTcpOutput);
        const QSignalBlocker b9(boxFileOutput);
        const QSignalBlocker b10(edTcpHost);
        const QSignalBlocker b11(spTcpPort);
        boxTcpOutput->setChecked(tcp_output_enabled);
        boxFileOutput->setChecked(file_output_enabled);
        edTcpHost->setText(tcp_host);
        spTcpPort->setValue(std::clamp(tcp_port, 1, 65535));
    }
    if (slRange) {
        slRange->setValue(std::clamp(static_cast<int>(std::lround(spRange->value())), slRange->minimum(), slRange->maximum()));
    }
    if (slGain) {
        slGain->setValue(std::clamp(static_cast<int>(std::lround(spGain->value() * 100.0)), slGain->minimum(), slGain->maximum()));
    }
    syncingAdvancedControls = false;
}

void SideScanControlPanel::emitAdvancedSssConfigChanged() {
    if (!spRange || !spGain || !spCenterFrequency || !spBandwidth || !spBeamWidth || !spBeamHeight ||
        !spAngleResolution || !spUpdateStride) {
        return;
    }
    emit advancedSssConfigChanged(
        spRange->value(),
        spGain->value(),
        spCenterFrequency->value(),
        spBandwidth->value(),
        spBeamWidth->value(),
        spBeamHeight->value(),
        spAngleResolution->value(),
        spUpdateStride->value(),
        boxTcpOutput ? boxTcpOutput->isChecked() : false,
        boxFileOutput ? boxFileOutput->isChecked() : false,
        edTcpHost ? edTcpHost->text() : QString(),
        spTcpPort ? spTcpPort->value() : 30002);
}

void SideScanControlPanel::onOutputControlChanged() {
    if (syncingAdvancedControls) {
        return;
    }
    emitAdvancedSssConfigChanged();
}

void SideScanControlPanel::onAdvancedControlChanged(double) {
    if (syncingAdvancedControls) {
        return;
    }
    updateBandwidthUpperBound();
    if (slRange && spRange) {
        const QSignalBlocker blocker(slRange);
        slRange->setValue(std::clamp(static_cast<int>(std::lround(spRange->value())), slRange->minimum(), slRange->maximum()));
    }
    if (slGain && spGain) {
        const QSignalBlocker blocker(slGain);
        slGain->setValue(std::clamp(static_cast<int>(std::lround(spGain->value() * 100.0)), slGain->minimum(), slGain->maximum()));
    }
    if (edRange && spRange) {
        edRange->setText(QString::number(static_cast<int>(std::lround(spRange->value()))) + " m");
    }
    if (edGain && spGain) {
        edGain->setText(QString::number(static_cast<int>(std::lround(spGain->value() * 100.0))) + " %");
    }
    emit rangeChanged(slRange ? slRange->value() : static_cast<int>(std::lround(spRange->value())));
    emit gainChanged(slGain ? slGain->value() : static_cast<int>(std::lround(spGain->value() * 100.0)));
    emitAdvancedSssConfigChanged();
}

void SideScanControlPanel::onUpdateStrideChanged(int) {
    if (syncingAdvancedControls) {
        return;
    }
    emitAdvancedSssConfigChanged();
}
