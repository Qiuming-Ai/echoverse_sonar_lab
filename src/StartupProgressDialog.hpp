#pragma once

#include <QDialog>
#include <QString>

class QLabel;
class QProgressBar;

class StartupProgressDialog final : public QDialog {
public:
    explicit StartupProgressDialog(QWidget* parent = nullptr);
    void setProgress(int value, const QString& status_text);

private:
    QLabel* status_label_ = nullptr;
    QProgressBar* progress_bar_ = nullptr;
};
