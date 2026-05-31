#ifndef SONARWIDGET_H
#define SONARWIDGET_H

#include <QtGlobal>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif
#include <sonar_types_v2/echoverse_sonar_types.hpp>
#include <sonar_types_v2/echoverse_math_types.hpp>

class SonarCanvas;

class SonarControlPanel : public QWidget
{
    Q_OBJECT
protected:
    SonarCanvas *plot = nullptr;
    QFrame *sonarDisplayFrame = nullptr;
    QWidget *sonarColorBar = nullptr;
    void layoutChildren();
    void layoutSonarDisplayArea(int x, int y, int w, int h);
    void resizeEvent(QResizeEvent *event);
    void createAdvancedSettingsPanel();
    void ensureAdvancedFormExtraControls();
    void emitAdvancedSonarConfigChanged();
    void updateBandwidthUpperBound();
    QLabel *lbGain = nullptr;
    QLabel *lbRange = nullptr;
    QLabel *lbPalette = nullptr;
    QLabel *lbGrid = nullptr;
    QLineEdit *edGain = nullptr;
    QLineEdit *edRange = nullptr;
    QWidget *advancedPanel = nullptr;
    QFormLayout *advancedForm = nullptr;
    QDoubleSpinBox *spRange = nullptr;
    QDoubleSpinBox *spGain = nullptr;
    QDoubleSpinBox *spCenterFrequency = nullptr;
    QDoubleSpinBox *spBandwidth = nullptr;
    QDoubleSpinBox *spBeamWidth = nullptr;
    QDoubleSpinBox *spBeamHeight = nullptr;
    QDoubleSpinBox *spAngleResolution = nullptr;
    QPushButton *advancedDrawerToggleButton = nullptr;
    bool advancedPanelEnabled = false;
    bool advancedDrawerVisible = false;
    bool advancedFormExtraControlsAttached = false;
    bool syncingAdvancedControls = false;

public:
    SonarControlPanel(QWidget *parent = 0);
    virtual ~SonarControlPanel();
    void createGainComponent();
    void createRangeComponent();
    void createPaletteComponent();
    void createGridComponent();
    void setAdvancedPanelEnabled(bool enabled);
    void setAdvancedSonarConfig(double range_m,
                                double gain,
                                double center_frequency_khz,
                                double bandwidth_khz,
                                double beam_width_deg,
                                double beam_height_deg,
                                double angle_resolution_deg);
    QSlider *slGain = nullptr;
    QSlider *slRange = nullptr;
    QComboBox *comboPalette = nullptr;
    QCheckBox *boxGrid = nullptr;

public slots:
    void setData(const sonar_types_v2::samples::SonarScan scan);
    void setData(const sonar_types_v2::samples::Sonar sonar);
    void setMotorStep(const sonar_types_v2::Angle step);
    void setGain(int);
    void setRange(int);
    void setMaxRange(int);
    void setMinRange(int);
    void setSonarPalette(int);
    void enableAutoRanging(bool);
    int getRange() const;
    int getGain() const;

    // only for scanning sonars
    void setSectorScan(bool continuous, sonar_types_v2::Angle left, sonar_types_v2::Angle right);

protected slots:
    void onSlGainChanged(int);
    void onSlRangeChanged(int);
    void onComboPaletteChanged(int);
    void onCheckboxGridChanged(bool);
    void onAdvancedControlChanged(double);

signals:
    void gainChanged(int);
    void rangeChanged(int);
    void sonarPaletteChanged(int);
    void gridChanged(bool);
    void advancedSonarConfigChanged(double range_m,
                                    double gain,
                                    double center_frequency_khz,
                                    double bandwidth_khz,
                                    double beam_width_deg,
                                    double beam_height_deg,
                                    double angle_resolution_deg);
};

#endif /* SONAR_WIDGET_H */
