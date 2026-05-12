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

/// Same control strip as \c SonarWidget (gain / range / palette / grid) with a side-scan waterfall plot.
class SideScanControlPanel : public QWidget {
    Q_OBJECT
protected:
    SideScanWaterfallCanvas* plot = nullptr;
    double scaleX = 1.0;
    double scaleY = 1.0;
    void resizeEvent(QResizeEvent* event) override;
    QLabel* lbGain = nullptr;
    QLabel* lbRange = nullptr;
    QLabel* lbPalette = nullptr;
    QLabel* lbGrid = nullptr;
    QLineEdit* edGain = nullptr;
    QLineEdit* edRange = nullptr;

public:
    explicit SideScanControlPanel(QWidget* parent = nullptr);
    ~SideScanControlPanel() override;

    void createGainComponent();
    void createRangeComponent();
    void createPaletteComponent();
    void createGridComponent();
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

signals:
    void gainChanged(int);
    void rangeChanged(int);
    void sonarPaletteChanged(int);
    void gridChanged(bool);
};

#endif
