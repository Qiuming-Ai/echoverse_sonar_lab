#pragma once

#include "WorldFileIo.hpp"

#include <QLabel>
#include <QPushButton>
#include <QTableWidget>
#include <QVector>
#include <QWidget>

#include <functional>

namespace osg {
class Group;
}

namespace standalone_mvp {

class SceneEditorPanel final : public QWidget {
    Q_OBJECT

public:
    explicit SceneEditorPanel(QWidget* parent = nullptr);

    void setWorldFile(const QString& absolute_world_path, const QString& world_spec_for_osg);
    void setWorldModelsGroup(osg::Group* world_models, float range_m);
    void setPauseSonarCallback(std::function<void(bool)> fn);
    void setProjectRoot(const QString& project_root);

signals:
    void sceneContentChanged();

private slots:
    void onAddModel();
    void onEditPose();
    void onDeleteModel();
    void onImportModels();

private:
    void showEvent(QShowEvent* event) override;
    void hideEvent(QHideEvent* event) override;
    void setEditMode(bool on);
    void applyEnabledState();
    void refreshTable();
    void reloadSceneGraph();
    bool saveEntriesToDisk(QString* err = nullptr);

    QVector<WorldIncludeEntry> entries_;
    QString world_path_;
    QString world_spec_osg_;
    QString project_root_;
    osg::Group* world_models_ = nullptr;
    float range_m_ = 40.f;
    std::function<void(bool)> pause_sonar_;
    bool edit_mode_active_ = false;

    QLabel* hint_ = nullptr;
    QTableWidget* table_ = nullptr;
    QPushButton* add_btn_ = nullptr;
    QPushButton* edit_btn_ = nullptr;
    QPushButton* del_btn_ = nullptr;
    QPushButton* reload_btn_ = nullptr;
};

} // namespace standalone_mvp
