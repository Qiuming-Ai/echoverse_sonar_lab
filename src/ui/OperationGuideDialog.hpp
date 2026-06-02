#pragma once

#include <QDialog>

namespace standalone_mvp {

class OperationGuideDialog final : public QDialog {
public:
    explicit OperationGuideDialog(QWidget* parent = nullptr);
};

} // namespace standalone_mvp
