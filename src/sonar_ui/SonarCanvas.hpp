#ifndef SONARPLOT_H
#define SONARPLOT_H

#include <QtGlobal>
#if QT_VERSION >= QT_VERSION_CHECK(5, 0, 0)
#include <QtWidgets>
#else
#include <QtGui>
#endif
#include <QFrame>
#include <sonar_types_v2/echoverse_sonar_types.hpp>
#include <sonar_types_v2/echoverse_math_types.hpp>
#include <sonar_palette/PaletteRamp.hpp>

#define BASE_WIDTH      1600
#define BASE_HEIGHT      900
#define BINS_REF_SIZE    500

inline constexpr double kSonarRenderRefWidth = static_cast<double>(BASE_WIDTH);
inline constexpr double kSonarRenderRefHeight = static_cast<double>(BASE_HEIGHT);

using namespace sonar_palette;


class SonarCanvas : public QFrame
{
    Q_OBJECT

public:
    SonarCanvas(QWidget *parent = 0);
    virtual ~SonarCanvas();
    void setData(const sonar_types_v2::samples::Sonar& sonar);
    void setSectorScan(bool continuous, sonar_types_v2::Angle leftLimit, sonar_types_v2::Angle rightLimit);
    void setMotorStep(const sonar_types_v2::Angle& step);

protected:
    void paintEvent(QPaintEvent *event);
    void resizeEvent(QResizeEvent *event);
    void drawOverlay();
    void generateMultibeamTransferTable(const sonar_types_v2::samples::Sonar& sonar);
    void generateScanningTransferTable(const sonar_types_v2::samples::Sonar& sonar);
    void applyColormap(PaletteType type);
    bool isMotorStepChanged(const sonar_types_v2::Angle& bearing);
    void addScanningData(const sonar_types_v2::samples::Sonar& sonar);
    int sonarIndexAtPixel(int x, int y) const;
    bool hasDrawableSonar() const;
    void updateOrigin();
    sonar_types_v2::samples::Sonar lastSonar;
    double scaleX;
    double scaleY;
    int range;
    int numSteps;
    bool changedSize;
    bool changedSectorScan;
    bool changedMotorStep;
    bool autoDetectMotorStep;
    bool isMultibeamSonar;
    bool continuous;
    bool enabledGrid;
    QPoint origin;
    std::vector<int> transfer;
    std::vector<QColor> colorMap;
    PaletteRamp heatMapGradient;
    sonar_types_v2::Angle motorStep, lastDiffStep;
    sonar_types_v2::Angle leftLimit, rightLimit;
    std::vector<float> sonarData;

protected slots:
    void rangeChanged(int);
    void sonarPaletteChanged(int);
    void gridChanged(bool);
};

#endif
