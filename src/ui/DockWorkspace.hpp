#pragma once

#include <QWidget>
#include <QPointer>
#include <QStringList>
#include <QPair>
#include <functional>

class QSplitter;
class QTabWidget;
class QMenu;

class DockWorkspace : public QWidget {
    Q_OBJECT
public:
    enum class LayoutPreset {
        Single = 0,
        Horizontal,
        Vertical,
        Quad,
    };

    explicit DockWorkspace(QWidget* parent = nullptr);

    QWidget* addTab(QWidget* page, const QString& title);
    QWidget* addTabToActivePane(QWidget* page, const QString& title);

    void restoreSingle();
    void applyQuadLayout();

    void splitActiveLeft();
    void splitActiveRight();
    void splitActiveTop();
    void splitActiveBottom();

    QTabWidget* primaryTabWidget() const;
    void setExtraContextMenuBuilder(std::function<void(QMenu&)> builder);
    LayoutPreset layoutPreset() const;
    void applyLayoutPreset(LayoutPreset preset);
    QStringList singlePaneTabTitles() const;
    QPair<QStringList, QStringList> horizontalPaneTabTitles() const;
    QPair<QStringList, QStringList> verticalPaneTabTitles() const;
    void quadPaneTabTitles(
        QStringList& top_left, QStringList& top_right,
        QStringList& bottom_left, QStringList& bottom_right) const;
    void restoreSinglePaneTabTitles(const QStringList& single_titles);
    void restoreHorizontalPaneTabTitles(const QStringList& left_titles, const QStringList& right_titles);
    void restoreVerticalPaneTabTitles(const QStringList& top_titles, const QStringList& bottom_titles);
    void restoreQuadPaneTabTitles(
        const QStringList& top_left_titles, const QStringList& top_right_titles,
        const QStringList& bottom_left_titles, const QStringList& bottom_right_titles);

signals:
    void layoutPresetChanged();

private:
    class DockPane;
    struct TabPayload;
    enum class LayoutMode {
        Single = 0,
        Horizontal,
        Vertical,
        Quad,
    };

    enum class SplitSide {
        Left = 0,
        Right,
        Top,
        Bottom,
    };

    DockPane* createPane();
    DockPane* ensureActivePane();
    QList<DockPane*> currentPanes() const;
    void activateMode(LayoutMode mode);
    void setRootWidget(QWidget* root_widget);
    QWidget* buildHorizontalLayout();
    QWidget* buildVerticalLayout();
    QWidget* buildQuadLayout();
    void clearRootWidget();
    void moveAllTabs(DockPane* from, DockPane* to);
    QList<TabPayload> takeAllTabs() const;
    void appendTabs(DockPane* pane, const QList<TabPayload>& tabs);
    DockPane* paneForSplitSide(LayoutMode mode, SplitSide side) const;
    DockPane* fallbackPaneForMode(LayoutMode mode, DockPane* excluded) const;
    LayoutMode modeForDropZone(int drop_zone_value) const;
    DockPane* paneForDropZone(LayoutMode mode, int drop_zone_value) const;
    void cleanupPaneIfEmpty(DockPane* pane);
    void enableDragPassthroughForPages();
    void disableDragPassthroughForPages();

    void startTabDrag(DockPane* source_pane, int tab_index);
    void handleTabDropped(
        const QString& source_pane_id, int tab_index, DockPane* target_pane, int drop_zone_value);
    void showPaneContextMenu(DockPane* pane, const QPoint& global_pos);
    QStringList tabTitlesForPane(const DockPane* pane) const;
    void redistributeTabsByTitles(const QList<DockPane*>& target_panes, const QList<QStringList>& ordered_titles);

    DockPane* findPaneById(const QString& pane_id) const;
    QList<DockPane*> allPanes() const;

    DockPane* active_pane_ = nullptr;
    QWidget* root_widget_ = nullptr;
    LayoutMode layout_mode_ = LayoutMode::Single;
    DockPane* pane_single_ = nullptr;
    DockPane* pane_lr_left_ = nullptr;
    DockPane* pane_lr_right_ = nullptr;
    DockPane* pane_tb_top_ = nullptr;
    DockPane* pane_tb_bottom_ = nullptr;
    DockPane* pane_q_tl_ = nullptr;
    DockPane* pane_q_tr_ = nullptr;
    DockPane* pane_q_bl_ = nullptr;
    DockPane* pane_q_br_ = nullptr;
    QList<QPointer<QWidget>> drag_passthrough_widgets_;
    std::function<void(QMenu&)> extra_context_menu_builder_;
};

