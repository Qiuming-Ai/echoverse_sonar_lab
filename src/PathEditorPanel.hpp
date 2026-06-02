#pragma once

#include "AppConfig.hpp"
#include "sonar_imaging/TopDownDepthMapGenerator.hpp"

#include <QCheckBox>
#include <QPushButton>
#include <QTableWidget>
#include <QToolButton>
#include <QVBoxLayout>
#include <QWidget>
#include <osg/Node>
#include <osg/ref_ptr>

namespace standalone_mvp {

class PathMapCanvas;

class PathEditorPanel final : public QWidget {
    Q_OBJECT

public:
    explicit PathEditorPanel(QWidget* parent = nullptr);

    void setSceneRoot(osg::ref_ptr<osg::Node> scene_root);
    void setProjectContext(const QString& project_root, const QString& world_spec);
    void setPathConfig(const PathModeConfig& cfg);
    void setLivePose(double x, double y, double yaw_rad);
    void notifySceneChanged();
    void setCompactLiveMapMode(bool on);
    PathModeConfig pathConfig() const;

signals:
    void startRequested(const PathModeConfig& cfg);
    void stopRequested();
    void pathEdited(const PathModeConfig& cfg);
    void teleportRequested(double x, double y, double z);

private:
    void ensureMapReady();
    QString cacheFilePath() const;
    QString cacheKey() const;
    qint64 worldMTimeMs() const;
    bool loadMapCache(const QString& file_path, sonar_imaging::TopDownDepthMapResult* out) const;
    bool saveMapCache(const QString& file_path, const sonar_imaging::TopDownDepthMapResult& map) const;
    void refreshTable();
    void syncConfigFromTable();
    void updateCanvasWaypoints();
    void setInteractionModeUi(bool add_mode, bool edit_mode);
    void setDrawerExpanded(bool expanded);
    void updateFloatingDrawerGeometry();
protected:
    void resizeEvent(QResizeEvent* event) override;

    osg::ref_ptr<osg::Node> scene_root_;
    QString project_root_;
    QString world_spec_;
    sonar_imaging::TopDownDepthMapGenerator map_generator_;
    PathModeConfig cfg_;

    QVBoxLayout* root_layout_ = nullptr;
    PathMapCanvas* map_canvas_ = nullptr;
    QWidget* editor_panel_ = nullptr;
    QWidget* drawer_content_ = nullptr;
    QTableWidget* table_ = nullptr;
    QToolButton* drawer_toggle_btn_ = nullptr;
    QCheckBox* loop_check_ = nullptr;
    QCheckBox* add_mode_check_ = nullptr;
    QCheckBox* edit_mode_check_ = nullptr;
    QPushButton* start_btn_ = nullptr;
    QPushButton* stop_btn_ = nullptr;
    QPushButton* clear_btn_ = nullptr;
};

} // namespace standalone_mvp

