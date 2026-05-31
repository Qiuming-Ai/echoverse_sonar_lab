#include "SideScanControlPanel.hpp"
#include "SideScanWaterfallCanvas.hpp"
#include "SonarCanvas.hpp"

#include <QtGlobal>
#include <algorithm>

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#define ROCK_PALETTE_TEXT_ROLE QPalette::WindowText
#define ROCK_PALETTE_FILL_ROLE QPalette::Window
#else
#define ROCK_PALETTE_TEXT_ROLE QPalette::Foreground
#define ROCK_PALETTE_FILL_ROLE QPalette::Background
#endif

SideScanControlPanel::SideScanControlPanel(QWidget* parent) : QWidget(parent)
{
    // Keep panel shrinkable without dominating splitter size allocation.
    setMinimumSize(120, 80);
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
    plot = new SideScanWaterfallCanvas(this);
    plot->setGeometry(10, 10, BASE_WIDTH, BASE_HEIGHT);
    connect(this, SIGNAL(rangeChanged(int)), plot, SLOT(rangeChanged(int)));
    connect(this, SIGNAL(sonarPaletteChanged(int)), plot, SLOT(sonarPaletteChanged(int)));
    connect(this, SIGNAL(gridChanged(bool)), plot, SLOT(gridChanged(bool)));

    QPalette Pal(palette());
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    Pal.setColor(QPalette::Window, plot->palette().color(QPalette::Window));
#else
    Pal.setColor(QPalette::Background, plot->palette().color(QPalette::Background));
#endif
    setAutoFillBackground(true);
    setPalette(Pal);

    createGainComponent();
    createRangeComponent();
    createPaletteComponent();
    createGridComponent();

}

SideScanControlPanel::~SideScanControlPanel() = default;

void SideScanControlPanel::resizeEvent(QResizeEvent* event)
{
    const int gain_row_y = std::max(8, height() - 70);
    const int range_row_y = std::max(8, height() - 40);
    const int right_col_x = std::max(10, width() - 160);
    const int right_input_x = std::max(70, width() - 100);
    if (plot) {
        plot->setGeometry(10, 10, std::max(80, width() - 20), std::max(80, height() - 70));
    }
    if (lbGain) {
        lbGain->setGeometry(10, gain_row_y, 50, 20);
    }
    if (slGain) {
        slGain->setGeometry(70, gain_row_y, 150, 20);
    }
    if (edGain) {
        edGain->setGeometry(230, gain_row_y, 50, 20);
    }
    if (lbRange) {
        lbRange->setGeometry(10, range_row_y, 50, 20);
    }
    if (slRange) {
        slRange->setGeometry(70, range_row_y, 150, 20);
    }
    if (edRange) {
        edRange->setGeometry(230, range_row_y, 50, 20);
    }
    if (lbPalette) {
        lbPalette->setGeometry(right_col_x, range_row_y, 50, 20);
    }
    if (comboPalette) {
        comboPalette->setGeometry(right_input_x, range_row_y, 80, 20);
    }
    if (lbGrid) {
        lbGrid->setGeometry(right_col_x, gain_row_y, 50, 20);
    }
    if (boxGrid) {
        boxGrid->setGeometry(right_input_x, gain_row_y, 80, 20);
    }
    QWidget::resizeEvent(event);
}

void SideScanControlPanel::setPortStarboardData(const sonar_types_v2::samples::Sonar& port, const sonar_types_v2::samples::Sonar& starboard)
{
    plot->setPortStarboardPing(port, starboard);
}

void SideScanControlPanel::setGain(int value)
{
    slGain->setValue(value);
}

int SideScanControlPanel::getGain() const
{
    return slGain->value();
}

void SideScanControlPanel::setRange(int value)
{
    slRange->setValue(value);
}

int SideScanControlPanel::getRange() const
{
    return slRange->value();
}

void SideScanControlPanel::setMinRange(int value)
{
    slRange->setMinimum(value);
}

void SideScanControlPanel::setMaxRange(int value)
{
    slRange->setMaximum(value);
}

void SideScanControlPanel::setSonarPalette(int value)
{
    comboPalette->setCurrentIndex(value);
}

void SideScanControlPanel::onSlGainChanged(int value)
{
    QString str;
    str.setNum(value);
    edGain->setText(str + " %");
    emit gainChanged(value);
}

void SideScanControlPanel::onSlRangeChanged(int value)
{
    QString str;
    str.setNum(value);
    edRange->setText(str + " m");
    emit rangeChanged(value);
}

void SideScanControlPanel::onComboPaletteChanged(int value)
{
    emit sonarPaletteChanged(value);
}

void SideScanControlPanel::onCheckboxGridChanged(bool value)
{
    emit gridChanged(value);
}

void SideScanControlPanel::createGainComponent()
{
    lbGain = new QLabel(this);
    lbGain->setGeometry(10, height() - 70, 50, 20);
    QPalette Pal = lbGain->palette();
    Pal.setColor(ROCK_PALETTE_TEXT_ROLE, Qt::white);
    lbGain->setPalette(Pal);
    lbGain->setText("Gain:");
    slGain = new QSlider(Qt::Horizontal, this);
    slGain->setGeometry(70, height() - 70, 150, 20);
    slGain->setMinimum(0);
    slGain->setMaximum(100);
    slGain->setValue(50);
    edGain = new QLineEdit(this);
    edGain->setGeometry(230, height() - 70, 50, 20);
    edGain->setAlignment(Qt::AlignRight);
    Pal = edGain->palette();
    Pal.setColor(QPalette::Base, plot->palette().color(ROCK_PALETTE_FILL_ROLE));
    Pal.setColor(QPalette::Text, Qt::white);
    setAutoFillBackground(true);
    edGain->setPalette(Pal);
    edGain->setReadOnly(true);
    edGain->setText("50 %");
    connect(slGain, SIGNAL(valueChanged(int)), this, SLOT(onSlGainChanged(int)));
}

void SideScanControlPanel::createRangeComponent()
{
    lbRange = new QLabel(this);
    lbRange->setGeometry(10, height() - 40, 50, 20);
    QPalette Pal = lbRange->palette();
    Pal.setColor(ROCK_PALETTE_TEXT_ROLE, Qt::white);
    lbRange->setPalette(Pal);
    lbRange->setText("Range:");
    slRange = new QSlider(Qt::Horizontal, this);
    slRange->setGeometry(70, height() - 40, 150, 20);
    slRange->setMinimum(1);
    slRange->setMaximum(150);
    slRange->setValue(5);
    edRange = new QLineEdit(this);
    edRange->setGeometry(230, height() - 40, 50, 20);
    edRange->setAlignment(Qt::AlignRight);
    Pal = edRange->palette();
    Pal.setColor(QPalette::Base, plot->palette().color(ROCK_PALETTE_FILL_ROLE));
    Pal.setColor(QPalette::Text, Qt::white);
    setAutoFillBackground(true);
    edRange->setPalette(Pal);
    edRange->setReadOnly(true);
    edRange->setText("5 m");
    connect(slRange, SIGNAL(valueChanged(int)), this, SLOT(onSlRangeChanged(int)));
}

void SideScanControlPanel::createPaletteComponent()
{
    lbPalette = new QLabel(this);
    lbPalette->setGeometry(width() - 160, height() - 40, 50, 20);
    lbPalette->setText("Palette:");
    lbPalette->setStyleSheet("background-color: #505050; color: white;");
    lbPalette->setAlignment(Qt::AlignRight);

    comboPalette = new QComboBox(this);
    comboPalette->setGeometry(width() - 100, height() - 40, 80, 20);
    QPalette Pal = comboPalette->palette();
    Pal.setColor(comboPalette->backgroundRole(), plot->palette().color(ROCK_PALETTE_FILL_ROLE));
    Pal.setColor(comboPalette->foregroundRole(), Qt::white);
    setAutoFillBackground(true);
    comboPalette->setPalette(Pal);
    comboPalette->insertItem(comboPalette->count() + 1, "Jet");
    comboPalette->insertItem(comboPalette->count() + 1, "Hot");
    comboPalette->insertItem(comboPalette->count() + 1, "Gray");
    comboPalette->insertItem(comboPalette->count() + 1, "Bronze");
    connect(comboPalette, SIGNAL(currentIndexChanged(int)), this, SLOT(onComboPaletteChanged(int)));
}

void SideScanControlPanel::createGridComponent()
{
    lbGrid = new QLabel(this);
    lbGrid->setGeometry(width() - 160, height() - 70, 50, 20);
    lbGrid->setText("Grid:");
    lbGrid->setAlignment(Qt::AlignRight);
    lbGrid->setStyleSheet("background-color: #505050; color: white;");

    boxGrid = new QCheckBox(this);
    boxGrid->setGeometry(width() - 100, height() - 70, 20, 20);
    boxGrid->setStyleSheet("background-color: #505050;");
    boxGrid->setChecked(true);
    connect(boxGrid, SIGNAL(clicked(bool)), this, SLOT(onCheckboxGridChanged(bool)));
}
