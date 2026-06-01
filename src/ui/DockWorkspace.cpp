#include "ui/DockWorkspace.hpp"

#include "ui/DraggableTabBar.hpp"
#include "ui/TabDropOverlay.hpp"

#include <QAction>
#include <QBoxLayout>
#include <QDrag>
#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QJsonDocument>
#include <QJsonObject>
#include <QDebug>
#include <QMenu>
#include <QMimeData>
#include <QSplitter>
#include <QTabWidget>
#include <QVBoxLayout>

namespace {
constexpr const char* kTabDragMimeType = "application/x-echoverse-tab-drag";

QString ptrToHex(const void* p) {
    return QString("0x%1").arg(reinterpret_cast<quintptr>(p), 0, 16);
}

QString widgetStateString(QWidget* w) {
    if (!w) return QStringLiteral("null");
    const QObject* parent_obj = w->parent();
    return QString("ptr=%1 class=%2 visible=%3 hidden=%4 parent=%5")
        .arg(ptrToHex(w))
        .arg(w->metaObject() ? w->metaObject()->className() : "unknown")
        .arg(w->isVisible() ? "1" : "0")
        .arg(w->isHidden() ? "1" : "0")
        .arg(parent_obj ? ptrToHex(parent_obj) : QStringLiteral("null"));
}

void styleSplitterHandle(QSplitter* splitter) {
    if (!splitter) {
        return;
    }
    splitter->setHandleWidth(2);
    splitter->setStyleSheet(
        "QSplitter::handle{"
        "background:#8fb8e0;"
        "border:1px solid #d5ecff;"
        "}"
        "QSplitter::handle:horizontal{"
        "width:2px;"
        "margin:0;"
        "}"
        "QSplitter::handle:vertical{"
        "height:2px;"
        "margin:0;"
        "}");
}
}

struct DockWorkspace::TabPayload {
    QWidget* page = nullptr;
    QString title;
    QIcon icon;
};

class DockWorkspace::DockPane : public QWidget {
    Q_OBJECT
    class DockTabWidget final : public QTabWidget {
    public:
        explicit DockTabWidget(QWidget* parent = nullptr) : QTabWidget(parent) {}
        void installTabBar(QTabBar* bar) { setTabBar(bar); }
    };

public:
    explicit DockPane(DockWorkspace* workspace, QWidget* parent = nullptr)
        : QWidget(parent), workspace_(workspace) {
        setAcceptDrops(true);
        setObjectName(QStringLiteral("DockPane"));
        setStyleSheet("QWidget#DockPane{background:#08111d;border:1px solid #4e6d90;}");

        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(2, 2, 2, 2);
        layout->setSpacing(0);

        tabs_ = new DockTabWidget(this);
        auto* bar = new DraggableTabBar(tabs_);
        tabs_->installTabBar(bar);
        tabs_->setTabsClosable(true);
        tabs_->setDocumentMode(true);
        tabs_->setMovable(false);
        tabs_->setElideMode(Qt::ElideRight);
        tabs_->setStyleSheet(
            "QTabWidget::pane{border:1px solid #4e6d90;}"
            "QTabBar::tab{background:#20364d;color:#d9e7ff;padding:6px 10px;border:1px solid #4e6d90;}"
            "QTabBar::tab:selected{background:#2b4b69;}");
        layout->addWidget(tabs_, 1);

        overlay_ = new TabDropOverlay(this);
        overlay_->setWindowFlags(Qt::ToolTip | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
        overlay_->setAttribute(Qt::WA_TranslucentBackground, true);
        overlay_->setAttribute(Qt::WA_OpaquePaintEvent, false);
        overlay_->setAutoFillBackground(false);
        overlay_->hide();

        connect(bar, &DraggableTabBar::startDragRequested, this, [this](int idx, const QPoint&) {
            emit startDragRequested(pane_id_, idx);
        });
        connect(tabs_, &QTabWidget::currentChanged, this, [this](int) { emit activated(this); });
        connect(tabs_, &QTabWidget::tabCloseRequested, this, [this](int idx) {
            QWidget* page = tabs_->widget(idx);
            tabs_->removeTab(idx);
            if (page) page->deleteLater();
            emit panePossiblyEmpty(this);
        });
        tabs_->tabBar()->setContextMenuPolicy(Qt::CustomContextMenu);
        connect(tabs_->tabBar(), &QWidget::customContextMenuRequested, this, [this](const QPoint& p) {
            emit contextMenuRequested(this, tabs_->tabBar()->mapToGlobal(p));
        });
        setContextMenuPolicy(Qt::CustomContextMenu);
        connect(this, &QWidget::customContextMenuRequested, this, [this](const QPoint& p) {
            emit contextMenuRequested(this, mapToGlobal(p));
        });

        pane_id_ = QString::number(reinterpret_cast<quintptr>(this), 16);
    }

    QString paneId() const { return pane_id_; }
    QTabWidget* tabs() const { return tabs_; }

protected:
    void resizeEvent(QResizeEvent* event) override {
        QWidget::resizeEvent(event);
        if (!overlay_) return;
        if (overlay_->isVisible()) {
            const QPoint global_top_left = mapToGlobal(QPoint(0, 0));
            overlay_->setGeometry(QRect(global_top_left, size()));
        }
    }

    void dragEnterEvent(QDragEnterEvent* event) override {
        if (!event->mimeData()->hasFormat(kTabDragMimeType)) {
            event->ignore();
            return;
        }
        const bool is_split_layout = qobject_cast<QSplitter*>(parentWidget()) != nullptr;
        overlay_->setDisplayMode(
            is_split_layout ? TabDropOverlay::DisplayMode::SingleTarget
                            : TabDropOverlay::DisplayMode::Detailed);
        const QPoint global_top_left = mapToGlobal(QPoint(0, 0));
        overlay_->setGeometry(QRect(global_top_left, size()));
        overlay_->setHoveredZone(overlay_->zoneAt(event->position().toPoint()));
        overlay_->show();
        overlay_->raise();
        event->acceptProposedAction();
    }

    void dragMoveEvent(QDragMoveEvent* event) override {
        if (!event->mimeData()->hasFormat(kTabDragMimeType)) {
            event->ignore();
            return;
        }
        const QPoint global_top_left = mapToGlobal(QPoint(0, 0));
        overlay_->setGeometry(QRect(global_top_left, size()));
        const bool is_split_layout = qobject_cast<QSplitter*>(parentWidget()) != nullptr;
        overlay_->setDisplayMode(
            is_split_layout ? TabDropOverlay::DisplayMode::SingleTarget
                            : TabDropOverlay::DisplayMode::Detailed);
        overlay_->setHoveredZone(overlay_->zoneAt(event->position().toPoint()));
        event->acceptProposedAction();
    }

    void dragLeaveEvent(QDragLeaveEvent* event) override {
        overlay_->hide();
        overlay_->setDisplayMode(TabDropOverlay::DisplayMode::Detailed);
        overlay_->setHoveredZone(TabDropOverlay::DropZone::None);
        QWidget::dragLeaveEvent(event);
    }

    void dropEvent(QDropEvent* event) override {
        overlay_->hide();
        overlay_->setDisplayMode(TabDropOverlay::DisplayMode::Detailed);
        if (!event->mimeData()->hasFormat(kTabDragMimeType)) {
            event->ignore();
            return;
        }
        const QJsonDocument doc = QJsonDocument::fromJson(event->mimeData()->data(kTabDragMimeType));
        if (!doc.isObject()) {
            event->ignore();
            return;
        }
        const QJsonObject obj = doc.object();
        const QString source_pane_id = obj.value(QStringLiteral("paneId")).toString();
        const int tab_index = obj.value(QStringLiteral("tabIndex")).toInt(-1);
        if (source_pane_id.isEmpty() || tab_index < 0) {
            event->ignore();
            return;
        }
        emit tabDropped(source_pane_id, tab_index, this, static_cast<int>(overlay_->zoneAt(event->position().toPoint())));
        event->acceptProposedAction();
    }

signals:
    void activated(DockPane* pane);
    void startDragRequested(const QString& pane_id, int tab_index);
    void tabDropped(const QString& source_pane_id, int tab_index, DockPane* target_pane, int drop_zone_value);
    void panePossiblyEmpty(DockPane* pane);
    void contextMenuRequested(DockPane* pane, const QPoint& global_pos);

private:
    DockWorkspace* workspace_ = nullptr;
    QString pane_id_;
    DockTabWidget* tabs_ = nullptr;
    TabDropOverlay* overlay_ = nullptr;
};

DockWorkspace::DockWorkspace(QWidget* parent)
    : QWidget(parent) {
    auto* root_layout = new QVBoxLayout(this);
    root_layout->setContentsMargins(0, 0, 0, 0);
    root_layout->setSpacing(0);
    activateMode(LayoutMode::Single);
}

DockWorkspace::DockPane* DockWorkspace::createPane() {
    auto* pane = new DockPane(this);
    connect(pane, &DockPane::activated, this, [this](DockPane* p) { active_pane_ = p; });
    connect(pane, &DockPane::startDragRequested, this, [this](const QString& pane_id, int tab_index) {
        if (DockPane* source = findPaneById(pane_id)) {
            startTabDrag(source, tab_index);
        }
    });
    connect(pane, &DockPane::tabDropped, this, [this](const QString& source_pane_id, int tab_index, DockPane* target_pane, int drop_zone_value) {
        handleTabDropped(source_pane_id, tab_index, target_pane, drop_zone_value);
    });
    connect(pane, &DockPane::panePossiblyEmpty, this, [this](DockPane* p) { cleanupPaneIfEmpty(p); });
    connect(pane, &DockPane::contextMenuRequested, this, [this](DockPane* p, const QPoint& pos) { showPaneContextMenu(p, pos); });
    return pane;
}

QWidget* DockWorkspace::addTab(QWidget* page, const QString& title) {
    return addTabToActivePane(page, title);
}

QWidget* DockWorkspace::addTabToActivePane(QWidget* page, const QString& title) {
    DockPane* pane = ensureActivePane();
    if (page && !page->property("_dock_destroy_watch").toBool()) {
        page->setProperty("_dock_destroy_watch", true);
        const QString page_title = title;
        QObject::connect(page, &QObject::destroyed, this, [page_title](QObject* obj) {
            qInfo().noquote()
                << QString("[dock] page destroyed title=%1 obj=%2")
                       .arg(page_title)
                       .arg(ptrToHex(obj));
        });
    }
    qInfo().noquote() << QString("[dock] addTabToActivePane pane=%1 title=%2 page={%3}")
                             .arg(pane ? pane->paneId() : "null")
                             .arg(title)
                             .arg(widgetStateString(page));
    pane->tabs()->addTab(page, title);
    pane->tabs()->setCurrentWidget(page);
    active_pane_ = pane;
    qInfo().noquote() << QString("[dock] addTabToActivePane done pane=%1 count=%2 current=%3")
                             .arg(pane ? pane->paneId() : "null")
                             .arg(pane ? pane->tabs()->count() : -1)
                             .arg(pane ? pane->tabs()->currentIndex() : -1);
    return page;
}

QTabWidget* DockWorkspace::primaryTabWidget() const {
    return active_pane_ ? active_pane_->tabs() : (pane_single_ ? pane_single_->tabs() : nullptr);
}

void DockWorkspace::setExtraContextMenuBuilder(std::function<void(QMenu&)> builder) {
    extra_context_menu_builder_ = std::move(builder);
}

DockWorkspace::LayoutPreset DockWorkspace::layoutPreset() const {
    switch (layout_mode_) {
    case LayoutMode::Horizontal:
        return LayoutPreset::Horizontal;
    case LayoutMode::Vertical:
        return LayoutPreset::Vertical;
    case LayoutMode::Quad:
        return LayoutPreset::Quad;
    case LayoutMode::Single:
    default:
        return LayoutPreset::Single;
    }
}

void DockWorkspace::applyLayoutPreset(LayoutPreset preset) {
    const LayoutMode target_mode = [&]() {
        switch (preset) {
        case LayoutPreset::Horizontal: return LayoutMode::Horizontal;
        case LayoutPreset::Vertical: return LayoutMode::Vertical;
        case LayoutPreset::Quad: return LayoutMode::Quad;
        case LayoutPreset::Single:
        default: return LayoutMode::Single;
        }
    }();
    if (layout_mode_ == target_mode) {
        return;
    }
    const QList<TabPayload> tabs = takeAllTabs();
    activateMode(target_mode);
    DockPane* target = ensureActivePane();
    appendTabs(target, tabs);
    if (target && target->tabs()->count() > 0) {
        target->tabs()->setCurrentIndex(0);
    }
    active_pane_ = target;
}

QStringList DockWorkspace::tabTitlesForPane(const DockPane* pane) const {
    QStringList out;
    if (!pane || !pane->tabs()) {
        return out;
    }
    QTabWidget* tabs = pane->tabs();
    for (int i = 0; i < tabs->count(); ++i) {
        out.push_back(tabs->tabText(i));
    }
    return out;
}

QStringList DockWorkspace::singlePaneTabTitles() const {
    if (layout_mode_ != LayoutMode::Single) {
        return {};
    }
    return tabTitlesForPane(pane_single_);
}

QPair<QStringList, QStringList> DockWorkspace::horizontalPaneTabTitles() const {
    if (layout_mode_ != LayoutMode::Horizontal) {
        return {};
    }
    return {tabTitlesForPane(pane_lr_left_), tabTitlesForPane(pane_lr_right_)};
}

QPair<QStringList, QStringList> DockWorkspace::verticalPaneTabTitles() const {
    if (layout_mode_ != LayoutMode::Vertical) {
        return {};
    }
    return {tabTitlesForPane(pane_tb_top_), tabTitlesForPane(pane_tb_bottom_)};
}

void DockWorkspace::quadPaneTabTitles(
    QStringList& top_left, QStringList& top_right,
    QStringList& bottom_left, QStringList& bottom_right) const {
    top_left.clear();
    top_right.clear();
    bottom_left.clear();
    bottom_right.clear();
    if (layout_mode_ != LayoutMode::Quad) {
        return;
    }
    top_left = tabTitlesForPane(pane_q_tl_);
    top_right = tabTitlesForPane(pane_q_tr_);
    bottom_left = tabTitlesForPane(pane_q_bl_);
    bottom_right = tabTitlesForPane(pane_q_br_);
}

void DockWorkspace::redistributeTabsByTitles(
    const QList<DockPane*>& target_panes, const QList<QStringList>& ordered_titles) {
    if (target_panes.isEmpty() || ordered_titles.size() != target_panes.size()) {
        return;
    }
    QList<TabPayload> pool = takeAllTabs();
    auto take_first_by_title = [&](const QString& title, TabPayload& out_payload) -> bool {
        for (int i = 0; i < pool.size(); ++i) {
            if (pool[i].page && pool[i].title == title) {
                out_payload = pool.takeAt(i);
                return true;
            }
        }
        return false;
    };

    for (int i = 0; i < target_panes.size(); ++i) {
        DockPane* pane = target_panes[i];
        if (!pane) {
            continue;
        }
        QList<TabPayload> picked;
        for (const QString& title : ordered_titles[i]) {
            TabPayload payload;
            if (take_first_by_title(title, payload)) {
                picked.push_back(payload);
            }
        }
        appendTabs(pane, picked);
    }

    if (!pool.isEmpty()) {
        DockPane* fallback = target_panes.first();
        if (!fallback) {
            fallback = ensureActivePane();
        }
        appendTabs(fallback, pool);
    }

    DockPane* first_non_empty = nullptr;
    for (DockPane* pane : target_panes) {
        if (pane && pane->tabs() && pane->tabs()->count() > 0) {
            first_non_empty = pane;
            break;
        }
    }
    active_pane_ = first_non_empty ? first_non_empty : ensureActivePane();
}

void DockWorkspace::restoreSinglePaneTabTitles(const QStringList& single_titles) {
    if (layout_mode_ != LayoutMode::Single || !pane_single_) {
        return;
    }
    redistributeTabsByTitles({pane_single_}, {single_titles});
}

void DockWorkspace::restoreHorizontalPaneTabTitles(
    const QStringList& left_titles, const QStringList& right_titles) {
    if (layout_mode_ != LayoutMode::Horizontal || !pane_lr_left_ || !pane_lr_right_) {
        return;
    }
    redistributeTabsByTitles({pane_lr_left_, pane_lr_right_}, {left_titles, right_titles});
}

void DockWorkspace::restoreVerticalPaneTabTitles(
    const QStringList& top_titles, const QStringList& bottom_titles) {
    if (layout_mode_ != LayoutMode::Vertical || !pane_tb_top_ || !pane_tb_bottom_) {
        return;
    }
    redistributeTabsByTitles({pane_tb_top_, pane_tb_bottom_}, {top_titles, bottom_titles});
}

void DockWorkspace::restoreQuadPaneTabTitles(
    const QStringList& top_left_titles, const QStringList& top_right_titles,
    const QStringList& bottom_left_titles, const QStringList& bottom_right_titles) {
    if (layout_mode_ != LayoutMode::Quad || !pane_q_tl_ || !pane_q_tr_ || !pane_q_bl_ || !pane_q_br_) {
        return;
    }
    redistributeTabsByTitles(
        {pane_q_tl_, pane_q_tr_, pane_q_bl_, pane_q_br_},
        {top_left_titles, top_right_titles, bottom_left_titles, bottom_right_titles});
}

void DockWorkspace::splitActiveLeft() {
    if (layout_mode_ != LayoutMode::Single) return;
    QList<TabPayload> tabs = takeAllTabs();
    qInfo().noquote() << QString("[dock] splitActiveLeft tabs=%1").arg(tabs.size());
    activateMode(LayoutMode::Horizontal);
    appendTabs(pane_lr_right_, tabs);
    if (pane_lr_right_ && pane_lr_right_->tabs()->count() > 0) {
        pane_lr_right_->tabs()->setCurrentIndex(0);
    }
    active_pane_ = pane_lr_right_;
}

void DockWorkspace::splitActiveRight() {
    if (layout_mode_ != LayoutMode::Single) return;
    QList<TabPayload> tabs = takeAllTabs();
    qInfo().noquote() << QString("[dock] splitActiveRight tabs=%1").arg(tabs.size());
    activateMode(LayoutMode::Horizontal);
    appendTabs(pane_lr_left_, tabs);
    if (pane_lr_left_ && pane_lr_left_->tabs()->count() > 0) {
        pane_lr_left_->tabs()->setCurrentIndex(0);
    }
    active_pane_ = pane_lr_left_;
}

void DockWorkspace::splitActiveTop() {
    if (layout_mode_ != LayoutMode::Single) return;
    QList<TabPayload> tabs = takeAllTabs();
    qInfo().noquote() << QString("[dock] splitActiveTop tabs=%1").arg(tabs.size());
    activateMode(LayoutMode::Vertical);
    appendTabs(pane_tb_bottom_, tabs);
    if (pane_tb_bottom_ && pane_tb_bottom_->tabs()->count() > 0) {
        pane_tb_bottom_->tabs()->setCurrentIndex(0);
    }
    active_pane_ = pane_tb_bottom_;
}

void DockWorkspace::splitActiveBottom() {
    if (layout_mode_ != LayoutMode::Single) return;
    QList<TabPayload> tabs = takeAllTabs();
    qInfo().noquote() << QString("[dock] splitActiveBottom tabs=%1").arg(tabs.size());
    activateMode(LayoutMode::Vertical);
    appendTabs(pane_tb_top_, tabs);
    if (pane_tb_top_ && pane_tb_top_->tabs()->count() > 0) {
        pane_tb_top_->tabs()->setCurrentIndex(0);
    }
    active_pane_ = pane_tb_top_;
}

DockWorkspace::DockPane* DockWorkspace::ensureActivePane() {
    if (active_pane_) return active_pane_;
    const QList<DockPane*> panes = currentPanes();
    active_pane_ = panes.isEmpty() ? nullptr : panes.first();
    return active_pane_;
}

QList<DockWorkspace::DockPane*> DockWorkspace::currentPanes() const {
    switch (layout_mode_) {
    case LayoutMode::Single:
        return pane_single_ ? QList<DockPane*>{pane_single_} : QList<DockPane*>{};
    case LayoutMode::Horizontal:
        return {pane_lr_left_, pane_lr_right_};
    case LayoutMode::Vertical:
        return {pane_tb_top_, pane_tb_bottom_};
    case LayoutMode::Quad:
        return {pane_q_tl_, pane_q_tr_, pane_q_bl_, pane_q_br_};
    }
    return {};
}

void DockWorkspace::clearRootWidget() {
    if (!root_widget_) return;
    qInfo().noquote() << QString("[dock] clearRootWidget root=%1").arg(ptrToHex(root_widget_));
    if (auto* box = qobject_cast<QBoxLayout*>(layout())) {
        box->removeWidget(root_widget_);
    }
    root_widget_->setParent(nullptr);
    root_widget_->deleteLater();
    root_widget_ = nullptr;
    pane_single_ = nullptr;
    pane_lr_left_ = nullptr;
    pane_lr_right_ = nullptr;
    pane_tb_top_ = nullptr;
    pane_tb_bottom_ = nullptr;
    pane_q_tl_ = nullptr;
    pane_q_tr_ = nullptr;
    pane_q_bl_ = nullptr;
    pane_q_br_ = nullptr;
}

void DockWorkspace::setRootWidget(QWidget* root_widget) {
    root_widget_ = root_widget;
    qInfo().noquote() << QString("[dock] setRootWidget root=%1").arg(ptrToHex(root_widget_));
    root_widget_->setParent(this);
    if (auto* box = qobject_cast<QBoxLayout*>(layout())) {
        box->addWidget(root_widget_, 1);
    } else if (layout()) {
        layout()->addWidget(root_widget_);
    }
}

QWidget* DockWorkspace::buildHorizontalLayout() {
    auto* split = new QSplitter(Qt::Horizontal);
    split->setChildrenCollapsible(false);
    styleSplitterHandle(split);
    pane_lr_left_ = createPane();
    pane_lr_right_ = createPane();
    split->addWidget(pane_lr_left_);
    split->addWidget(pane_lr_right_);
    split->setSizes({1, 1});
    return split;
}

QWidget* DockWorkspace::buildVerticalLayout() {
    auto* split = new QSplitter(Qt::Vertical);
    split->setChildrenCollapsible(false);
    styleSplitterHandle(split);
    pane_tb_top_ = createPane();
    pane_tb_bottom_ = createPane();
    split->addWidget(pane_tb_top_);
    split->addWidget(pane_tb_bottom_);
    split->setSizes({1, 1});
    return split;
}

QWidget* DockWorkspace::buildQuadLayout() {
    auto* root = new QSplitter(Qt::Vertical);
    root->setChildrenCollapsible(false);
    styleSplitterHandle(root);
    auto* top = new QSplitter(Qt::Horizontal, root);
    auto* bottom = new QSplitter(Qt::Horizontal, root);
    top->setChildrenCollapsible(false);
    bottom->setChildrenCollapsible(false);
    styleSplitterHandle(top);
    styleSplitterHandle(bottom);
    pane_q_tl_ = createPane();
    pane_q_tr_ = createPane();
    pane_q_bl_ = createPane();
    pane_q_br_ = createPane();
    top->addWidget(pane_q_tl_);
    top->addWidget(pane_q_tr_);
    bottom->addWidget(pane_q_bl_);
    bottom->addWidget(pane_q_br_);
    root->addWidget(top);
    root->addWidget(bottom);
    top->setSizes({1, 1});
    bottom->setSizes({1, 1});
    root->setSizes({1, 1});
    return root;
}

void DockWorkspace::activateMode(LayoutMode mode) {
    clearRootWidget();
    qInfo().noquote() << QString("[dock] activateMode from=%1 to=%2")
                             .arg(static_cast<int>(layout_mode_))
                             .arg(static_cast<int>(mode));
    layout_mode_ = mode;
    switch (layout_mode_) {
    case LayoutMode::Single: {
        pane_single_ = createPane();
        setRootWidget(pane_single_);
        active_pane_ = pane_single_;
        break;
    }
    case LayoutMode::Horizontal:
        setRootWidget(buildHorizontalLayout());
        active_pane_ = pane_lr_left_;
        break;
    case LayoutMode::Vertical:
        setRootWidget(buildVerticalLayout());
        active_pane_ = pane_tb_top_;
        break;
    case LayoutMode::Quad:
        setRootWidget(buildQuadLayout());
        active_pane_ = pane_q_tl_;
        break;
    }
    emit layoutPresetChanged();
}

QList<DockWorkspace::TabPayload> DockWorkspace::takeAllTabs() const {
    QList<TabPayload> out;
    const QList<DockPane*> panes = currentPanes();
    qInfo().noquote() << QString("[dock] takeAllTabs panes=%1 mode=%2")
                             .arg(panes.size())
                             .arg(static_cast<int>(layout_mode_));
    for (DockPane* pane : panes) {
        if (!pane) continue;
        QTabWidget* tabs = pane->tabs();
        qInfo().noquote() << QString("[dock] takeAllTabs pane=%1 count_before=%2")
                                 .arg(pane->paneId())
                                 .arg(tabs->count());
        while (tabs->count() > 0) {
            QWidget* page = tabs->widget(0);
            TabPayload payload{page, tabs->tabText(0), tabs->tabIcon(0)};
            tabs->removeTab(0);
            if (payload.page) {
                // Keep pages owned by workspace while old panes get destroyed.
                payload.page->setParent(const_cast<DockWorkspace*>(this));
            }
            qInfo().noquote() << QString("[dock] takeAllTabs moved title=%1 page={%2}")
                                     .arg(payload.title)
                                     .arg(widgetStateString(payload.page));
            out.push_back(payload);
        }
    }
    qInfo().noquote() << QString("[dock] takeAllTabs done total=%1").arg(out.size());
    return out;
}

void DockWorkspace::appendTabs(DockPane* pane, const QList<TabPayload>& tabs) {
    if (!pane) return;
    int first_added_index = -1;
    qInfo().noquote() << QString("[dock] appendTabs start pane=%1 input=%2")
                             .arg(pane->paneId())
                             .arg(tabs.size());
    for (const TabPayload& payload : tabs) {
        if (!payload.page) continue;
        qInfo().noquote() << QString("[dock] appendTabs add title=%1 page_before={%2}")
                                 .arg(payload.title)
                                 .arg(widgetStateString(payload.page));
        const int inserted = pane->tabs()->addTab(payload.page, payload.icon, payload.title);
        qInfo().noquote() << QString("[dock] appendTabs added title=%1 idx=%2 page_after={%3}")
                                 .arg(payload.title)
                                 .arg(inserted)
                                 .arg(widgetStateString(payload.page));
        if (first_added_index < 0) {
            first_added_index = inserted;
        }
    }
    if (pane->tabs()->count() > 0 && pane->tabs()->currentIndex() < 0) {
        pane->tabs()->setCurrentIndex(first_added_index >= 0 ? first_added_index : 0);
    }
    if (pane->tabs()->currentWidget()) {
        pane->tabs()->currentWidget()->setVisible(true);
    }
    qInfo().noquote() << QString("[dock] appendTabs pane=%1 count=%2 current=%3")
                             .arg(pane->paneId())
                             .arg(pane->tabs()->count())
                             .arg(pane->tabs()->currentIndex());
}

DockWorkspace::LayoutMode DockWorkspace::modeForDropZone(int drop_zone_value) const {
    const auto zone = static_cast<TabDropOverlay::DropZone>(drop_zone_value);
    switch (zone) {
    case TabDropOverlay::DropZone::Left:
    case TabDropOverlay::DropZone::Right:
        return LayoutMode::Horizontal;
    case TabDropOverlay::DropZone::Top:
    case TabDropOverlay::DropZone::Bottom:
        return LayoutMode::Vertical;
    case TabDropOverlay::DropZone::TopLeft:
    case TabDropOverlay::DropZone::TopRight:
    case TabDropOverlay::DropZone::BottomLeft:
    case TabDropOverlay::DropZone::BottomRight:
        return LayoutMode::Quad;
    case TabDropOverlay::DropZone::Center:
    case TabDropOverlay::DropZone::None:
    default:
        return LayoutMode::Single;
    }
}

DockWorkspace::DockPane* DockWorkspace::paneForSplitSide(LayoutMode mode, SplitSide side) const {
    if (mode == LayoutMode::Horizontal) {
        return side == SplitSide::Left ? pane_lr_left_ : pane_lr_right_;
    }
    if (mode == LayoutMode::Vertical) {
        return side == SplitSide::Top ? pane_tb_top_ : pane_tb_bottom_;
    }
    return nullptr;
}

DockWorkspace::DockPane* DockWorkspace::paneForDropZone(LayoutMode mode, int drop_zone_value) const {
    const auto zone = static_cast<TabDropOverlay::DropZone>(drop_zone_value);
    if (mode == LayoutMode::Horizontal) {
        if (zone == TabDropOverlay::DropZone::Left) return pane_lr_left_;
        if (zone == TabDropOverlay::DropZone::Right) return pane_lr_right_;
        return pane_lr_right_;
    }
    if (mode == LayoutMode::Vertical) {
        if (zone == TabDropOverlay::DropZone::Top) return pane_tb_top_;
        if (zone == TabDropOverlay::DropZone::Bottom) return pane_tb_bottom_;
        return pane_tb_bottom_;
    }
    if (mode == LayoutMode::Quad) {
        switch (zone) {
        case TabDropOverlay::DropZone::TopLeft: return pane_q_tl_;
        case TabDropOverlay::DropZone::TopRight: return pane_q_tr_;
        case TabDropOverlay::DropZone::BottomLeft: return pane_q_bl_;
        case TabDropOverlay::DropZone::BottomRight: return pane_q_br_;
        default: return pane_q_tl_;
        }
    }
    return pane_single_;
}

DockWorkspace::DockPane* DockWorkspace::fallbackPaneForMode(LayoutMode mode, DockPane* excluded) const {
    const QList<DockPane*> panes = (mode == LayoutMode::Single) ? QList<DockPane*>{pane_single_} : currentPanes();
    for (DockPane* pane : panes) {
        if (pane && pane != excluded) return pane;
    }
    return excluded;
}

void DockWorkspace::moveAllTabs(DockPane* from, DockPane* to) {
    if (!from || !to || from == to) return;
    QTabWidget* src = from->tabs();
    while (src->count() > 0) {
        QWidget* page = src->widget(0);
        const QString title = src->tabText(0);
        const QIcon icon = src->tabIcon(0);
        src->removeTab(0);
        to->tabs()->addTab(page, icon, title);
    }
}

void DockWorkspace::enableDragPassthroughForPages() {
    if (!drag_passthrough_widgets_.isEmpty()) {
        return;
    }
    const QList<DockPane*> panes = currentPanes();
    for (DockPane* pane : panes) {
        if (!pane || !pane->tabs()) {
            continue;
        }
        QTabWidget* tabs = pane->tabs();
        for (int i = 0; i < tabs->count(); ++i) {
            QWidget* page = tabs->widget(i);
            if (!page) {
                continue;
            }
            QList<QWidget*> candidates{page};
            const QList<QWidget*> descendants = page->findChildren<QWidget*>();
            for (QWidget* w : descendants) {
                candidates.push_back(w);
            }
            for (QWidget* w : candidates) {
                if (!w || w->testAttribute(Qt::WA_TransparentForMouseEvents)) {
                    continue;
                }
                w->setAttribute(Qt::WA_TransparentForMouseEvents, true);
                drag_passthrough_widgets_.push_back(w);
            }
        }
    }
}

void DockWorkspace::disableDragPassthroughForPages() {
    for (const QPointer<QWidget>& w : drag_passthrough_widgets_) {
        if (!w) {
            continue;
        }
        w->setAttribute(Qt::WA_TransparentForMouseEvents, false);
    }
    drag_passthrough_widgets_.clear();
}

void DockWorkspace::startTabDrag(DockPane* source_pane, int tab_index) {
    if (!source_pane || tab_index < 0 || tab_index >= source_pane->tabs()->count()) return;
    auto* drag = new QDrag(this);
    auto* mime = new QMimeData();
    QJsonObject obj;
    obj.insert(QStringLiteral("paneId"), source_pane->paneId());
    obj.insert(QStringLiteral("tabIndex"), tab_index);
    mime->setData(kTabDragMimeType, QJsonDocument(obj).toJson(QJsonDocument::Compact));
    drag->setMimeData(mime);
    enableDragPassthroughForPages();
    drag->exec(Qt::MoveAction);
    disableDragPassthroughForPages();
}

void DockWorkspace::handleTabDropped(
    const QString& source_pane_id, int tab_index, DockPane* target_pane, int drop_zone_value) {
    DockPane* source = findPaneById(source_pane_id);
    qInfo().noquote() << QString("[dock] handleTabDropped source=%1 target=%2 tab_index=%3 zone=%4 mode=%5")
                             .arg(source_pane_id)
                             .arg(target_pane ? target_pane->paneId() : "null")
                             .arg(tab_index)
                             .arg(drop_zone_value)
                             .arg(static_cast<int>(layout_mode_));
    if (!source || !target_pane || tab_index < 0 || tab_index >= source->tabs()->count()) return;

    TabPayload dragged{
        source->tabs()->widget(tab_index),
        source->tabs()->tabText(tab_index),
        source->tabs()->tabIcon(tab_index)
    };
    source->tabs()->removeTab(tab_index);
    if (dragged.page) {
        // Prevent dragged page from being deleted with source pane.
        dragged.page->setParent(this);
    }
    qInfo().noquote() << QString("[dock] handleTabDropped dragged title=%1 page={%2}")
                             .arg(dragged.title)
                             .arg(widgetStateString(dragged.page));
    if (!dragged.page) {
        cleanupPaneIfEmpty(source);
        return;
    }

    if (layout_mode_ == LayoutMode::Single) {
        const LayoutMode requested_mode = modeForDropZone(drop_zone_value);
        if (requested_mode != LayoutMode::Single) {
            const QList<TabPayload> remaining = takeAllTabs();
            activateMode(requested_mode);
            DockPane* destination = paneForDropZone(requested_mode, drop_zone_value);
            if (!destination) {
                destination = ensureActivePane();
            }
            DockPane* fallback = fallbackPaneForMode(requested_mode, destination);
            if (!fallback) {
                fallback = ensureActivePane();
            }
            appendTabs(fallback, remaining);
            appendTabs(destination, {dragged});
            if (destination && destination->tabs()->indexOf(dragged.page) >= 0) {
                destination->tabs()->setCurrentWidget(dragged.page);
            }
            active_pane_ = destination;
            return;
        }
    }

    // In split modes, only tab grouping/moving is allowed; no mode transitions.
    DockPane* destination = target_pane;
    if (!destination) destination = ensureActivePane();
    if (!destination) {
        appendTabs(source, {dragged});
        return;
    }
    appendTabs(destination, {dragged});
    if (destination->tabs()->indexOf(dragged.page) >= 0) {
        destination->tabs()->setCurrentWidget(dragged.page);
    }
    active_pane_ = destination;
    cleanupPaneIfEmpty(source);
}

DockWorkspace::DockPane* DockWorkspace::findPaneById(const QString& pane_id) const {
    const QList<DockPane*> panes = currentPanes();
    for (DockPane* p : panes) {
        if (p && p->paneId() == pane_id) return p;
    }
    return nullptr;
}

QList<DockWorkspace::DockPane*> DockWorkspace::allPanes() const {
    return currentPanes();
}

void DockWorkspace::cleanupPaneIfEmpty(DockPane* pane) {
    if (!pane) return;
    // Keep split structure stable in fixed modes. Empty panes are allowed.
    if (layout_mode_ != LayoutMode::Single) return;
    if (pane->tabs()->count() > 0) return;
}

void DockWorkspace::restoreSingle() {
    const QList<TabPayload> tabs = takeAllTabs();
    activateMode(LayoutMode::Single);
    appendTabs(pane_single_, tabs);
    if (pane_single_ && pane_single_->tabs()->count() > 0) {
        pane_single_->tabs()->setCurrentIndex(0);
    }
    active_pane_ = pane_single_;
}

void DockWorkspace::applyQuadLayout() {
    if (layout_mode_ != LayoutMode::Single) return;
    const QList<TabPayload> tabs = takeAllTabs();
    activateMode(LayoutMode::Quad);
    appendTabs(pane_q_tl_, tabs);
    if (pane_q_tl_ && pane_q_tl_->tabs()->count() > 0) {
        pane_q_tl_->tabs()->setCurrentIndex(0);
    }
    active_pane_ = pane_q_tl_;
}

void DockWorkspace::showPaneContextMenu(DockPane* pane, const QPoint& global_pos) {
    if (!pane) return;
    active_pane_ = pane;
    QMenu menu;
    QAction* split_left = menu.addAction("Split Left/Right");
    QAction* split_top = menu.addAction("Split Top/Bottom");
    QAction* quad = menu.addAction("Apply 2x2 Layout");
    menu.addSeparator();
    QAction* restore = menu.addAction("Restore Single");

    const bool can_split = (layout_mode_ == LayoutMode::Single);
    split_left->setEnabled(can_split);
    split_top->setEnabled(can_split);
    quad->setEnabled(can_split);
    restore->setEnabled(layout_mode_ != LayoutMode::Single);
    if (extra_context_menu_builder_) {
        menu.addSeparator();
        extra_context_menu_builder_(menu);
    }

    QAction* chosen = menu.exec(global_pos);
    if (!chosen) return;
    if (chosen == split_left) splitActiveRight();
    else if (chosen == split_top) splitActiveBottom();
    else if (chosen == quad) applyQuadLayout();
    else if (chosen == restore) restoreSingle();
}

#include "DockWorkspace.moc"

