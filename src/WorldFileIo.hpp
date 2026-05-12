#pragma once

#include <QString>
#include <QStringList>
#include <QVector>
#include <array>

namespace standalone_mvp {

struct WorldIncludeEntry {
    QString model_name;
    std::array<double, 6> pose{};
};

/// Subdirectories of `uwmodels/sdf` that contain `model.sdf`.
QStringList discoverSdfModelNames();

bool loadWorldIncludes(const QString& world_file_path, QVector<WorldIncludeEntry>* out, QString* error = nullptr);

bool saveWorldIncludes(const QString& world_file_path, const QVector<WorldIncludeEntry>& entries, QString* error = nullptr);

} // namespace standalone_mvp
