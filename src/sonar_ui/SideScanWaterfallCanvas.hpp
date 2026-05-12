#ifndef SIDESCANWATERFALLPLOT_H
#define SIDESCANWATERFALLPLOT_H

#include <QFrame>
#include <QImage>
#include <vector>

#include <sonar_types_v2/echoverse_sonar_types.hpp>
#include <sonar_palette/PaletteRamp.hpp>

/// Scrolls side-scan pings vertically: newest row at bottom; horizontal axis is cross-track distance
/// from \c -range_m to \c +range_m (port | starboard), vertical axis is time (history).
class SideScanWaterfallCanvas : public QFrame {
    Q_OBJECT
public:
    explicit SideScanWaterfallCanvas(QWidget* parent = nullptr);

    /// Port = SSS A (left side of vehicle), starboard = SSS B (right). Uses beam 0 of each sample.
    void setPortStarboardPing(const sonar_types_v2::samples::Sonar& port, const sonar_types_v2::samples::Sonar& starboard);

public slots:
    void rangeChanged(int value_m);
    void sonarPaletteChanged(int index);
    void gridChanged(bool value);

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    void applyColormap(sonar_palette::PaletteType type);

    std::vector<QColor> colorMap_;
    sonar_palette::PaletteRamp heatMapGradient_;
    QImage scroll_;
    int rangeM_ = 5;
    bool enabledGrid_ = true;
    unsigned int binCount_ = 0;

    static constexpr int k_history_lines = 600;
};

#endif
