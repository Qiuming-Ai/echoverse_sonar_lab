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
    void resizeEvent(QResizeEvent *event);
    QLabel *lbGain = nullptr;
    QLabel *lbRange = nullptr;
    QLabel *lbPalette = nullptr;
    QLabel *lbGrid = nullptr;
    QLineEdit *edGain = nullptr;
    QLineEdit *edRange = nullptr;

public:
    SonarControlPanel(QWidget *parent = 0);
    virtual ~SonarControlPanel();
    void createGainComponent();
    void createRangeComponent();
    void createPaletteComponent();
    void createGridComponent();
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

signals:
    void gainChanged(int);
    void rangeChanged(int);
    void sonarPaletteChanged(int);
    void gridChanged(bool);
};

#endif /* SONAR_WIDGET_H */
