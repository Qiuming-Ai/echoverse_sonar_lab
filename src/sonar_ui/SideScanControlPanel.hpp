#ifndef SIDESCANSTRIPWIDGET_H
#define SIDESCANSTRIPWIDGET_H

#include <QtGlobal>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif
#include <sonar_types_v2/echoverse_sonar_types.hpp>

class SideScanWaterfallCanvas;

/// Side-scan waterfall with the same settings drawer as \c SonarControlPanel (FLS).
class SideScanControlPanel : public QWidget {
    Q_OBJECT
protected:
    SideScanWaterfallCanvas* plot = nullptr;
    QFrame* sonarDisplayFrame = nullptr;
    QWidget* sonarColorBar = nullptr;
    void layoutChildren();
    void layoutSonarDisplayArea(int x, int y, int w, int h);
    void resizeEvent(QResizeEvent* event) override;
    void createAdvancedSettingsPanel();
    void ensureAdvancedFormExtraControls();
    void emitAdvancedSssConfigChanged();
    void updateBandwidthUpperBound();
    QLabel* lbGain = nullptr;
    QLabel* lbRange = nullptr;
    QLabel* lbPalette = nullptr;
    QLabel* lbGrid = nullptr;
    QLineEdit* edGain = nullptr;
    QLineEdit* edRange = nullptr;
    QWidget* advancedPanel = nullptr;
    QFormLayout* advancedForm = nullptr;
    QDoubleSpinBox* spRange = nullptr;
    QDoubleSpinBox* spGain = nullptr;
    QDoubleSpinBox* spCenterFrequency = nullptr;
    QDoubleSpinBox* spBandwidth = nullptr;
    QDoubleSpinBox* spBeamWidth = nullptr;
    QDoubleSpinBox* spBeamHeight = nullptr;
    QDoubleSpinBox* spAngleResolution = nullptr;
    QSpinBox* spUpdateStride = nullptr;
    QCheckBox* boxTcpOutput = nullptr;
    QCheckBox* boxFileOutput = nullptr;
    QLineEdit* edTcpHost = nullptr;
    QSpinBox* spTcpPort = nullptr;
    QPushButton* advancedDrawerToggleButton = nullptr;
    bool advancedPanelEnabled = false;
    bool advancedDrawerVisible = false;
    bool advancedFormExtraControlsAttached = false;
    bool advancedFormOutputControlsAttached = false;
    bool syncingAdvancedControls = false;

public:
    explicit SideScanControlPanel(QWidget* parent = nullptr);
    ~SideScanControlPanel() override;

    void createGainComponent();
    void createRangeComponent();
    void createPaletteComponent();
    void createGridComponent();
    void setAdvancedPanelEnabled(bool enabled);
    void setAdvancedSssConfig(double range_m,
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
                              int tcp_port);
    QSlider* slGain = nullptr;
    QSlider* slRange = nullptr;
    QComboBox* comboPalette = nullptr;
    QCheckBox* boxGrid = nullptr;

public slots:
    void setPortStarboardData(const sonar_types_v2::samples::Sonar& port, const sonar_types_v2::samples::Sonar& starboard);
    void setGain(int value);
    void setRange(int value);
    void setMaxRange(int value);
    void setMinRange(int value);
    void setSonarPalette(int value);
    int getRange() const;
    int getGain() const;

protected slots:
    void onSlGainChanged(int value);
    void onSlRangeChanged(int value);
    void onComboPaletteChanged(int value);
    void onCheckboxGridChanged(bool value);
    void onAdvancedControlChanged(double value);
    void onUpdateStrideChanged(int value);
    void onOutputControlChanged();

signals:
    void gainChanged(int);
    void rangeChanged(int);
    void sonarPaletteChanged(int);
    void gridChanged(bool);
    void advancedSssConfigChanged(double range_m,
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
                                  int tcp_port);
};

#endif
