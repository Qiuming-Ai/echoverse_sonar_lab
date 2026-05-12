#include "SceneEditorPanel.hpp"

#include "SharedScene.hpp"

#include <osg/Group>

#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFormLayout>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QTableWidgetItem>
#include <QVBoxLayout>

namespace standalone_mvp {

namespace {

bool runPoseEditorDialog(QWidget* parent, const QString& title, std::array<double, 6>* pose) {
    if (!pose) {
        return false;
    }
    QDialog dlg(parent);
    dlg.setWindowTitle(title);
    dlg.resize(380, 300);
    auto* layout = new QVBoxLayout(&dlg);
    auto* form = new QFormLayout();
    QDoubleSpinBox* s[6];
    const QString labels[] = {QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z"),
                              QStringLiteral("Roll"), QStringLiteral("Pitch"), QStringLiteral("Yaw")};
    for (int i = 0; i < 6; ++i) {
        s[i] = new QDoubleSpinBox(&dlg);
        s[i]->setRange(-1.0e7, 1.0e7);
        s[i]->setDecimals(6);
        s[i]->setSingleStep(0.1);
        s[i]->setValue((*pose)[static_cast<std::size_t>(i)]);
        form->addRow(labels[i], s[i]);
    }
    layout->addLayout(form);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    layout->addWidget(buttons);
    if (dlg.exec() != QDialog::Accepted) {
        return false;
    }
    for (int i = 0; i < 6; ++i) {
        (*pose)[static_cast<std::size_t>(i)] = s[i]->value();
    }
    return true;
}

} // namespace

SceneEditorPanel::SceneEditorPanel(QWidget* parent)
    : QWidget(parent) {
    auto* main = new QVBoxLayout(this);
    main->setContentsMargins(6, 6, 6, 6);
    main->setSpacing(6);
    enable_edit_ = new QCheckBox(QStringLiteral("Scene Edit Mode (pause sonar updates, preview geometry only)"), this);
    hint_ = new QLabel(
        QStringLiteral("Scan models from uwmodels/sdf, write to the project world file, then refresh the main viewer."),
        this);
    hint_->setWordWrap(true);
    hint_->setStyleSheet(QStringLiteral("QLabel{color:#9fb6cc;font-size:11px;}"));
    table_ = new QTableWidget(this);
    table_->setMinimumWidth(640);
    table_->setColumnCount(7);
    table_->setHorizontalHeaderLabels({QStringLiteral("Model"), QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z"),
                                        QStringLiteral("Roll"), QStringLiteral("Pitch"), QStringLiteral("Yaw")});
    table_->setSelectionBehavior(QAbstractItemView::SelectRows);
    table_->setSelectionMode(QAbstractItemView::SingleSelection);
    table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
    table_->verticalHeader()->setVisible(false);
    table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
    for (int i = 1; i < 7; ++i) {
        table_->horizontalHeader()->setSectionResizeMode(i, QHeaderView::ResizeToContents);
    }
    add_btn_ = new QPushButton(QStringLiteral("Add Model..."), this);
    edit_btn_ = new QPushButton(QStringLiteral("Edit Pose..."), this);
    del_btn_ = new QPushButton(QStringLiteral("Delete"), this);
    reload_btn_ = new QPushButton(QStringLiteral("Reload From Disk"), this);
    auto* row = new QHBoxLayout();
    row->addWidget(add_btn_);
    row->addWidget(edit_btn_);
    row->addWidget(del_btn_);
    main->addWidget(enable_edit_);
    main->addWidget(hint_);
    main->addWidget(table_, 1);
    main->addLayout(row);
    main->addWidget(reload_btn_);

    const auto apply_enabled = [this](bool editing) {
        const bool ok = editing && !world_path_.isEmpty() && world_models_;
        table_->setEnabled(ok);
        add_btn_->setEnabled(ok);
        edit_btn_->setEnabled(ok);
        del_btn_->setEnabled(ok);
        reload_btn_->setEnabled(ok);
    };
    apply_enabled(false);

    QObject::connect(enable_edit_, &QCheckBox::toggled, this, [this, apply_enabled](bool on) {
        if (pause_sonar_) {
            pause_sonar_(on);
        }
        apply_enabled(on);
    });
    QObject::connect(add_btn_, &QPushButton::clicked, this, &SceneEditorPanel::onAddModel);
    QObject::connect(edit_btn_, &QPushButton::clicked, this, &SceneEditorPanel::onEditPose);
    QObject::connect(del_btn_, &QPushButton::clicked, this, &SceneEditorPanel::onDeleteModel);
    QObject::connect(reload_btn_, &QPushButton::clicked, this, &SceneEditorPanel::onReloadPreview);
}

void SceneEditorPanel::setWorldFile(const QString& absolute_world_path, const QString& world_spec_for_osg) {
    world_path_ = absolute_world_path;
    world_spec_osg_ = world_spec_for_osg;
    entries_.clear();
    if (!world_path_.isEmpty() && QFile::exists(world_path_)) {
        QString err;
        loadWorldIncludes(world_path_, &entries_, &err);
    }
    refreshTable();
    const auto apply_enabled = [this](bool editing) {
        const bool ok = editing && !world_path_.isEmpty() && world_models_;
        table_->setEnabled(ok);
        add_btn_->setEnabled(ok);
        edit_btn_->setEnabled(ok);
        del_btn_->setEnabled(ok);
        reload_btn_->setEnabled(ok);
    };
    apply_enabled(enable_edit_->isChecked());
}

void SceneEditorPanel::setWorldModelsGroup(osg::Group* world_models, float range_m) {
    world_models_ = world_models;
    range_m_ = range_m;
    const auto apply_enabled = [this](bool editing) {
        const bool ok = editing && !world_path_.isEmpty() && world_models_;
        table_->setEnabled(ok);
        add_btn_->setEnabled(ok);
        edit_btn_->setEnabled(ok);
        del_btn_->setEnabled(ok);
        reload_btn_->setEnabled(ok);
    };
    apply_enabled(enable_edit_->isChecked());
}

void SceneEditorPanel::setPauseSonarCallback(std::function<void(bool)> fn) {
    pause_sonar_ = std::move(fn);
}

void SceneEditorPanel::refreshTable() {
    table_->setRowCount(0);
    for (int i = 0; i < entries_.size(); ++i) {
        const WorldIncludeEntry& e = entries_[i];
        const int row = table_->rowCount();
        table_->insertRow(row);
        table_->setItem(row, 0, new QTableWidgetItem(e.model_name));
        for (int c = 0; c < 6; ++c) {
            table_->setItem(row, c + 1, new QTableWidgetItem(QString::number(e.pose[static_cast<std::size_t>(c)], 'g', 6)));
        }
    }
}

bool SceneEditorPanel::saveEntriesToDisk(QString* err) {
    if (world_path_.isEmpty()) {
        if (err) {
            *err = QStringLiteral("World file path is not set.");
        }
        return false;
    }
    return saveWorldIncludes(world_path_, entries_, err);
}

void SceneEditorPanel::reloadSceneGraph() {
    if (!world_models_ || world_spec_osg_.isEmpty()) {
        return;
    }
    rebuildWorldModelsFromWorldSpec(world_models_, range_m_, world_spec_osg_.toUtf8().constData());
}

void SceneEditorPanel::onAddModel() {
    if (!enable_edit_->isChecked()) {
        return;
    }
    const QStringList models = discoverSdfModelNames();
    if (models.isEmpty()) {
        QMessageBox::warning(this, QStringLiteral("Warning"), QStringLiteral("No models were found under uwmodels/sdf."));
        return;
    }
    QDialog dlg(this);
    dlg.setWindowTitle(QStringLiteral("Add Model"));
    auto* layout = new QVBoxLayout(&dlg);
    auto* form = new QFormLayout();
    auto* combo = new QComboBox(&dlg);
    for (const QString& m : models) {
        combo->addItem(m);
    }
    form->addRow(QStringLiteral("Model"), combo);
    QDoubleSpinBox* sp[6];
    const QString labels[] = {QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Z"),
                              QStringLiteral("Roll"), QStringLiteral("Pitch"), QStringLiteral("Yaw")};
    for (int i = 0; i < 6; ++i) {
        sp[i] = new QDoubleSpinBox(&dlg);
        sp[i]->setRange(-1.0e7, 1.0e7);
        sp[i]->setDecimals(6);
        sp[i]->setSingleStep(0.1);
        sp[i]->setValue(0.0);
        form->addRow(labels[i], sp[i]);
    }
    layout->addLayout(form);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dlg);
    QObject::connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);
    layout->addWidget(buttons);
    if (dlg.exec() != QDialog::Accepted) {
        return;
    }
    WorldIncludeEntry ent;
    ent.model_name = combo->currentText().trimmed();
    for (int i = 0; i < 6; ++i) {
        ent.pose[static_cast<std::size_t>(i)] = sp[i]->value();
    }
    if (ent.model_name.isEmpty()) {
        return;
    }
    entries_.push_back(ent);
    QString err;
    if (!saveEntriesToDisk(&err)) {
        entries_.pop_back();
        QMessageBox::critical(this, QStringLiteral("Error"), err);
        return;
    }
    reloadSceneGraph();
    refreshTable();
}

void SceneEditorPanel::onEditPose() {
    if (!enable_edit_->isChecked()) {
        return;
    }
    const int row = table_->currentRow();
    if (row < 0 || row >= entries_.size()) {
        QMessageBox::information(this, QStringLiteral("Notice"), QStringLiteral("Please select one row first."));
        return;
    }
    std::array<double, 6> pose = entries_[row].pose;
    if (!runPoseEditorDialog(this, QStringLiteral("Pose — %1").arg(entries_[row].model_name), &pose)) {
        return;
    }
    entries_[row].pose = pose;
    QString err;
    if (!saveEntriesToDisk(&err)) {
        QMessageBox::critical(this, QStringLiteral("Error"), err);
        return;
    }
    reloadSceneGraph();
    refreshTable();
}

void SceneEditorPanel::onDeleteModel() {
    if (!enable_edit_->isChecked()) {
        return;
    }
    const int row = table_->currentRow();
    if (row < 0 || row >= entries_.size()) {
        QMessageBox::information(this, QStringLiteral("Notice"), QStringLiteral("Please select a row to delete."));
        return;
    }
    const auto r = QMessageBox::question(this, QStringLiteral("Confirm"), QStringLiteral("Remove this model from the world file?"));
    if (r != QMessageBox::Yes) {
        return;
    }
    entries_.removeAt(row);
    QString err;
    if (!saveEntriesToDisk(&err)) {
        QMessageBox::critical(this, QStringLiteral("Error"), err);
        return;
    }
    reloadSceneGraph();
    refreshTable();
}

void SceneEditorPanel::onReloadPreview() {
    if (!enable_edit_->isChecked()) {
        return;
    }
    QString err;
    if (!loadWorldIncludes(world_path_, &entries_, &err)) {
        QMessageBox::warning(this, QStringLiteral("Warning"), err.isEmpty() ? QStringLiteral("Failed to load world file.") : err);
        return;
    }
    refreshTable();
    reloadSceneGraph();
}

} // namespace standalone_mvp
