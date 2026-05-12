#include "SonarControlPanel.hpp"

#include "SonarCanvas.hpp"

#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#define ROCK_PALETTE_TEXT_ROLE QPalette::WindowText
#define ROCK_PALETTE_FILL_ROLE QPalette::Window
#else
#define ROCK_PALETTE_TEXT_ROLE QPalette::Foreground
#define ROCK_PALETTE_FILL_ROLE QPalette::Background
#endif

namespace {
constexpr int kPanelMargin = 10;
constexpr int kControlRowHeight = 30;
constexpr int kControlRowSpacing = 8;
constexpr int kBottomPanelHeight = 70;
}

SonarControlPanel::SonarControlPanel(QWidget *parent)
    : QWidget(parent) {
    plot = new SonarCanvas(this);
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

    createGainComponent();
    createRangeComponent();
    createPaletteComponent();
    createGridComponent();

    resize(1020, 670);
    show();
}

SonarControlPanel::~SonarControlPanel() = default;

void SonarControlPanel::resizeEvent(QResizeEvent *event) {
    if (plot) {
        plot->setGeometry(kPanelMargin, kPanelMargin, width() - 2 * kPanelMargin, height() - kBottomPanelHeight);
    }

    const int firstRowY = height() - kBottomPanelHeight + 4;
    const int secondRowY = firstRowY + kControlRowHeight + kControlRowSpacing;

    if (lbGain) lbGain->setGeometry(10, firstRowY, 50, 20);
    if (slGain) slGain->setGeometry(70, firstRowY, 150, 20);
    if (edGain) edGain->setGeometry(230, firstRowY, 50, 20);

    if (lbRange) lbRange->setGeometry(10, secondRowY, 50, 20);
    if (slRange) slRange->setGeometry(70, secondRowY, 150, 20);
    if (edRange) edRange->setGeometry(230, secondRowY, 50, 20);

    if (lbGrid) lbGrid->setGeometry(width() - 160, firstRowY, 50, 20);
    if (boxGrid) boxGrid->setGeometry(width() - 100, firstRowY, 80, 20);

    if (lbPalette) lbPalette->setGeometry(width() - 160, secondRowY, 50, 20);
    if (comboPalette) comboPalette->setGeometry(width() - 100, secondRowY, 80, 20);

    QWidget::resizeEvent(event);
}

void SonarControlPanel::setData(const sonar_types_v2::samples::SonarScan scan) {
    sonar_types_v2::samples::Sonar sonar(scan);
    setData(sonar);
}

void SonarControlPanel::setData(const sonar_types_v2::samples::Sonar sonar) {
    if (plot) {
        plot->setData(sonar);
    }
}

void SonarControlPanel::setMotorStep(const sonar_types_v2::Angle step) {
    if (plot) {
        plot->setMotorStep(step);
    }
}

void SonarControlPanel::setGain(int value) {
    if (slGain) {
        slGain->setValue(value);
    }
}

int SonarControlPanel::getGain() const {
    return slGain ? slGain->value() : 0;
}

void SonarControlPanel::setRange(int value) {
    if (slRange) {
        slRange->setValue(value);
    }
}

int SonarControlPanel::getRange() const {
    return slRange ? slRange->value() : 0;
}

void SonarControlPanel::setSectorScan(bool continuous, sonar_types_v2::Angle left, sonar_types_v2::Angle right) {
    if (plot) {
        plot->setSectorScan(continuous, left, right);
    }
}

void SonarControlPanel::setMinRange(int value) {
    if (slRange) {
        slRange->setMinimum(value);
    }
}

void SonarControlPanel::setMaxRange(int value) {
    if (slRange) {
        slRange->setMaximum(value);
    }
}

void SonarControlPanel::setSonarPalette(int value) {
    if (comboPalette) {
        comboPalette->setCurrentIndex(value);
    }
}

void SonarControlPanel::enableAutoRanging(bool value) {
    if (slRange) {
        slRange->setEnabled(!value);
    }
}

void SonarControlPanel::onSlGainChanged(int value) {
    if (edGain) {
        edGain->setText(QString::number(value) + " %");
    }
    gainChanged(value);
}

void SonarControlPanel::onSlRangeChanged(int value) {
    if (edRange) {
        edRange->setText(QString::number(value) + " m");
    }
    rangeChanged(value);
}

void SonarControlPanel::onComboPaletteChanged(int value) {
    sonarPaletteChanged(value);
}

void SonarControlPanel::onCheckboxGridChanged(bool value) {
    gridChanged(value);
}

void SonarControlPanel::createGainComponent() {
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

void SonarControlPanel::createRangeComponent() {
    lbRange = new QLabel(this);
    lbRange->setText("Range:");
    QPalette palette = lbRange->palette();
    palette.setColor(ROCK_PALETTE_TEXT_ROLE, Qt::white);
    lbRange->setPalette(palette);

    slRange = new QSlider(Qt::Horizontal, this);
    slRange->setRange(1, 150);
    slRange->setValue(5);

    edRange = new QLineEdit(this);
    edRange->setReadOnly(true);
    edRange->setAlignment(Qt::AlignRight);
    edRange->setText("5 m");
    palette = edRange->palette();
    palette.setColor(QPalette::Base, plot->palette().color(ROCK_PALETTE_FILL_ROLE));
    palette.setColor(QPalette::Text, Qt::white);
    edRange->setPalette(palette);

    connect(slRange, SIGNAL(valueChanged(int)), this, SLOT(onSlRangeChanged(int)));
}

void SonarControlPanel::createPaletteComponent() {
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

void SonarControlPanel::createGridComponent() {
    lbGrid = new QLabel(this);
    lbGrid->setText("Grid:");
    lbGrid->setAlignment(Qt::AlignRight);
    lbGrid->setStyleSheet("background-color: #505050; color: white;");

    boxGrid = new QCheckBox(this);
    boxGrid->setStyleSheet("background-color: #505050;");
    boxGrid->setChecked(true);

    connect(boxGrid, SIGNAL(clicked(bool)), this, SLOT(onCheckboxGridChanged(bool)));
}
