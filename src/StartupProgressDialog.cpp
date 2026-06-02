#include "StartupProgressDialog.hpp"

#include <algorithm>
#include <QApplication>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QProgressBar>
#include <QScreen>
#include <QVBoxLayout>

StartupProgressDialog::StartupProgressDialog(QWidget* parent)
    : QDialog(parent, Qt::SplashScreen | Qt::FramelessWindowHint) {
    setWindowTitle(QStringLiteral("EchoVerse Sonar Lab"));
    setModal(true);
    setFixedSize(460, 150);
    setStyleSheet(
        "QDialog{background:#0a1424;border:1px solid #2a76c9;border-radius:12px;}"
        "QLabel{color:#dcefff;background:transparent;}"
        "QProgressBar{border:1px solid #1f3a59;border-radius:6px;background:#06101e;text-align:center;color:#dcefff;}"
        "QProgressBar::chunk{background:qlineargradient(x1:0,y1:0,x2:1,y2:0,stop:0 #1a5ca8,stop:1 #3ecf6e);"
        "border-radius:5px;}");

    auto* root = new QVBoxLayout(this);
    root->setContentsMargins(18, 16, 18, 16);
    root->setSpacing(10);

    auto* title = new QLabel(QStringLiteral("EchoVerse Sonar Lab"));
    title->setStyleSheet(QStringLiteral("font-size:18px;font-weight:700;color:#f2f8ff;"));
    root->addWidget(title);

    status_label_ = new QLabel(QStringLiteral("Starting..."));
    status_label_->setWordWrap(true);
    root->addWidget(status_label_);

    progress_bar_ = new QProgressBar();
    progress_bar_->setRange(0, 100);
    progress_bar_->setValue(0);
    progress_bar_->setTextVisible(true);
    root->addWidget(progress_bar_);

    if (QScreen* screen = QApplication::primaryScreen()) {
        const QRect geo = screen->availableGeometry();
        move(geo.center() - rect().center());
    }
}

void StartupProgressDialog::setProgress(const int value, const QString& status_text) {
    progress_bar_->setValue(std::clamp(value, 0, 100));
    status_label_->setText(status_text);
    QApplication::processEvents();
}
