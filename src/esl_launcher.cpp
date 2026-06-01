#include "AppConfig.hpp"

#include <QApplication>
#include <QCoreApplication>
#include <QCheckBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QProcess>
#include <QPushButton>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QRegularExpression>
#include <QSettings>
#include <QStackedWidget>
#include <QVBoxLayout>

#include <filesystem>

namespace {
namespace fs = std::filesystem;

QString worldKeyFromWizardText(const QString& raw) {
    QString t = raw.trimmed();
    if (t.isEmpty()) {
        return QStringLiteral("ssiv_bahia");
    }
    QFileInfo fi(t);
    if (fi.suffix().compare(QStringLiteral("world"), Qt::CaseInsensitive) == 0) {
        return fi.completeBaseName();
    }
    return t;
}

QString findRepoScenesRootPath() {
#ifdef STANDALONE_SIMULATION_DIR
    {
        const QString p = QStringLiteral(STANDALONE_SIMULATION_DIR) + QStringLiteral("/uwmodels/scenes");
        if (QDir(p).exists()) {
            return QDir(p).absolutePath();
        }
    }
#endif
    const QString cur = QDir::cleanPath(QDir::currentPath() + QStringLiteral("/uwmodels/scenes"));
    if (QDir(cur).exists()) {
        return cur;
    }
    const QString app =
        QDir::cleanPath(QCoreApplication::applicationDirPath() + QStringLiteral("/uwmodels/scenes"));
    if (QDir(app).exists()) {
        return app;
    }
    return {};
}

bool copyWorldIntoProjectScene(const QString& world_key, const QString& project_dir, QString* error_out) {
    const QString scenes_root = findRepoScenesRootPath();
    if (scenes_root.isEmpty()) {
        if (error_out) {
            *error_out = QStringLiteral("Built-in scene directory uwmodels/scenes was not found.");
        }
        return false;
    }
    const QString src_dir = QDir(scenes_root).filePath(world_key);
    if (!QDir(src_dir).exists()) {
        if (error_out) {
            *error_out = QStringLiteral("Source scene directory was not found:\n%1").arg(src_dir);
        }
        return false;
    }
    const QString dst_parent = QDir(project_dir).filePath(QStringLiteral("uwmodels/scenes"));
    const QString dst_dir = QDir(dst_parent).filePath(world_key);
    std::error_code ec;
    const fs::path dst_path(QString(dst_dir).toStdWString());
    if (fs::exists(dst_path)) {
        fs::remove_all(dst_path, ec);
        if (ec) {
            if (error_out) {
                *error_out = QStringLiteral("Failed to clear destination directory:\n%1\n%2")
                                 .arg(dst_dir, QString::fromStdString(ec.message()));
            }
            return false;
        }
    }
    if (!QDir().mkpath(dst_parent)) {
        if (error_out) {
            *error_out = QStringLiteral("Failed to create project uwmodels/scenes directory.");
        }
        return false;
    }
    const fs::path src_path(QString(src_dir).toStdWString());
    fs::copy(src_path, dst_path, fs::copy_options::recursive | fs::copy_options::overwrite_existing, ec);
    if (ec) {
        if (error_out) {
            *error_out =
                QStringLiteral("Failed to copy scene:\n%1").arg(QString::fromStdString(ec.message()));
        }
        return false;
    }
    const QString world_file = QDir(dst_dir).filePath(world_key + QStringLiteral(".world"));
    if (!QFile::exists(world_file)) {
        if (error_out) {
            *error_out = QStringLiteral("Copied world file was not found:\n%1").arg(world_file);
        }
        return false;
    }
    return true;
}

constexpr char kSettingsOrg[] = "EchoVerse";
constexpr char kSettingsApp[] = "EchoVerseSonarLab";
constexpr char kRecentKey[] = "recent_eslproj_paths";

QString sanitizedProjectBaseName(QString name) {
    name = name.trimmed();
    name.replace(QRegularExpression(QStringLiteral("[\\\\/:*?\"<>|]+")), QStringLiteral("_"));
    if (name.isEmpty()) {
        name = QStringLiteral("Project");
    }
    return name;
}

QStringList loadRecent() {
    QSettings s(kSettingsOrg, kSettingsApp);
    return s.value(kRecentKey).toStringList();
}

void saveRecent(const QStringList& paths) {
    QSettings s(kSettingsOrg, kSettingsApp);
    s.setValue(kRecentKey, paths);
}

void addRecentPath(const QString& eslproj_absolute) {
    const QString norm = QDir::cleanPath(QFileInfo(eslproj_absolute).absoluteFilePath());
    if (norm.isEmpty() || !QFile::exists(norm)) {
        return;
    }
    QStringList list = loadRecent();
    list.removeAll(norm);
    list.prepend(norm);
    while (list.size() > 20) {
        list.removeLast();
    }
    saveRecent(list);
}

QString echoverseSonarLabExecutablePath() {
    const QString dir = QCoreApplication::applicationDirPath();
#ifdef _WIN32
    return dir + QStringLiteral("/echoverse_sonar_lab.exe");
#else
    return dir + QStringLiteral("/echoverse_sonar_lab");
#endif
}

bool launchEchoverseSonarLab(const QString& eslproj_path) {
    const QString exe = echoverseSonarLabExecutablePath();
    if (!QFile::exists(exe)) {
        QMessageBox::critical(nullptr, QStringLiteral("EchoVerse Sonar Lab Launcher"),
                              QStringLiteral("The EchoVerse Sonar Lab executable was not found:\n%1").arg(exe));
        return false;
    }
    addRecentPath(eslproj_path);
    const QString native = QDir::toNativeSeparators(eslproj_path);
    if (!QProcess::startDetached(exe, {QStringLiteral("--project"), native})) {
        QMessageBox::critical(nullptr, QStringLiteral("EchoVerse Sonar Lab Launcher"),
                              QStringLiteral("Unable to launch EchoVerse Sonar Lab."));
        return false;
    }
    return true;
}

class NewProjectDialog final : public QDialog {
public:
    explicit NewProjectDialog(QWidget* parent = nullptr)
        : QDialog(parent) {
        setWindowTitle(QStringLiteral("EchoVerse Sonar Lab Launcher - Create Project"));
        resize(560, 480);

        auto* root = new QVBoxLayout(this);
        stack_ = new QStackedWidget();
        root->addWidget(stack_, 1);

        // Page 0: base info
        auto* base_page = new QWidget();
        auto* base_layout = new QVBoxLayout(base_page);
        auto* base_form = new QFormLayout();
        path_edit_ = new QLineEdit();
        auto* browse = new QPushButton(QStringLiteral("Browse..."));
        auto* path_row = new QHBoxLayout();
        path_row->addWidget(path_edit_, 1);
        path_row->addWidget(browse);
        base_form->addRow(QStringLiteral("Workspace Folder:"), path_row);
        name_edit_ = new QLineEdit();
        base_form->addRow(QStringLiteral("Project Title:"), name_edit_);
        base_layout->addLayout(base_form);
        auto* base_buttons = new QDialogButtonBox(QDialogButtonBox::Cancel);
        auto* base_next = base_buttons->addButton(QStringLiteral("Next"), QDialogButtonBox::ActionRole);
        base_layout->addWidget(base_buttons);
        stack_->addWidget(base_page);

        // Page 1: scene config
        auto* scene_page = new QWidget();
        auto* scene_layout = new QVBoxLayout(scene_page);
        auto* scene_form = new QFormLayout();
        world_combo_ = new QComboBox();
        world_combo_->setEditable(true);
        world_combo_->addItems({QStringLiteral("mangalia"), QStringLiteral("ssiv_bahia"), QStringLiteral("tank")});
        world_combo_->setCurrentText(QStringLiteral("mangalia"));
        scene_form->addRow(QStringLiteral("Scene World:"), world_combo_);
        pos_x_ = createSpin(-100000.0, 100000.0, -609.11);
        pos_y_ = createSpin(-100000.0, 100000.0, 77.27);
        pos_z_ = createSpin(-100000.0, 100000.0, 1.5);
        step_xy_ = createSpin(0.01, 1000.0, 1.0);
        step_z_ = createSpin(0.01, 1000.0, 1.0);
        step_yaw_deg_ = createSpin(0.1, 90.0, 5.0);
        main_yaw_deg_ = createSpin(-360.0, 360.0, -360.0);
        main_pitch_deg_ = createSpin(-89.0, 89.0, 10.0);
        scene_form->addRow(QStringLiteral("Initial X:"), pos_x_);
        scene_form->addRow(QStringLiteral("Initial Y:"), pos_y_);
        scene_form->addRow(QStringLiteral("Initial Z:"), pos_z_);
        scene_form->addRow(QStringLiteral("Step XY:"), step_xy_);
        scene_form->addRow(QStringLiteral("Step Z:"), step_z_);
        scene_form->addRow(QStringLiteral("Step Yaw (deg):"), step_yaw_deg_);
        scene_form->addRow(QStringLiteral("Main Camera Yaw (deg):"), main_yaw_deg_);
        scene_form->addRow(QStringLiteral("Main Camera Pitch (deg):"), main_pitch_deg_);
        scene_layout->addLayout(scene_form);
        auto* scene_buttons = new QDialogButtonBox(QDialogButtonBox::Cancel);
        auto* scene_back = scene_buttons->addButton(QStringLiteral("Back"), QDialogButtonBox::ActionRole);
        auto* scene_next = scene_buttons->addButton(QStringLiteral("Next"), QDialogButtonBox::ActionRole);
        scene_layout->addWidget(scene_buttons);
        stack_->addWidget(scene_page);

        // Page 2: sonar type selection
        auto* sonar_page = new QWidget();
        auto* sonar_layout = new QVBoxLayout(sonar_page);
        auto* sonar_form = new QFormLayout();
        fls_check_ = new QCheckBox(QStringLiteral("FLS"));
        mbes_check_ = new QCheckBox(QStringLiteral("MBES"));
        sss_check_ = new QCheckBox(QStringLiteral("Side-Scan Sonar (SSS)"));
        fls_check_->setChecked(true);
        mbes_check_->setChecked(true);
        sss_check_->setChecked(true);
        sonar_form->addRow(QStringLiteral("Sonar Types (max one each):"), fls_check_);
        sonar_form->addRow(QString(), mbes_check_);
        sonar_form->addRow(QString(), sss_check_);
        sonar_layout->addLayout(sonar_form);
        auto* sonar_buttons = new QDialogButtonBox(QDialogButtonBox::Cancel);
        auto* sonar_back = sonar_buttons->addButton(QStringLiteral("Back"), QDialogButtonBox::ActionRole);
        auto* sonar_next = sonar_buttons->addButton(QStringLiteral("Next"), QDialogButtonBox::ActionRole);
        sonar_layout->addWidget(sonar_buttons);
        stack_->addWidget(sonar_page);

        // Page 3: final create
        auto* final_page = new QWidget();
        auto* final_layout = new QVBoxLayout(final_page);
        final_layout->addWidget(new QLabel(QStringLiteral("Configuration is complete. Click \"Create and Open\" to generate your EchoVerse Sonar Lab project and launch the application.")));
        auto* final_buttons = new QDialogButtonBox(QDialogButtonBox::Cancel);
        auto* final_back = final_buttons->addButton(QStringLiteral("Back"), QDialogButtonBox::ActionRole);
        auto* create_btn = final_buttons->addButton(QStringLiteral("Create and Open"), QDialogButtonBox::AcceptRole);
        final_layout->addWidget(final_buttons);
        stack_->addWidget(final_page);

        connect(browse, &QPushButton::clicked, this, [this] {
            const QString d = QFileDialog::getExistingDirectory(this, QStringLiteral("Select Project Directory"), path_edit_->text());
            if (!d.isEmpty()) {
                path_edit_->setText(QDir::toNativeSeparators(d));
            }
        });
        connect(base_buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        connect(scene_buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        connect(sonar_buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        connect(final_buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
        connect(base_next, &QPushButton::clicked, this, [this]() {
            const QString root = path_edit_->text().trimmed();
            const QString base_name = sanitizedProjectBaseName(name_edit_->text());
            if (root.isEmpty() || !QDir(root).exists()) {
                QMessageBox::warning(this, QStringLiteral("EchoVerse Sonar Lab Launcher"),
                                     QStringLiteral("Select a valid workspace folder to continue."));
                return;
            }
            if (base_name.isEmpty()) {
                QMessageBox::warning(this, QStringLiteral("EchoVerse Sonar Lab Launcher"),
                                     QStringLiteral("Enter a project title to continue."));
                return;
            }
            stack_->setCurrentIndex(1);
        });
        connect(scene_back, &QPushButton::clicked, this, [this]() { stack_->setCurrentIndex(0); });
        connect(scene_next, &QPushButton::clicked, this, [this]() { stack_->setCurrentIndex(2); });
        connect(sonar_back, &QPushButton::clicked, this, [this]() { stack_->setCurrentIndex(1); });
        connect(sonar_next, &QPushButton::clicked, this, [this]() {
            if (!fls_check_->isChecked() && !mbes_check_->isChecked() && !sss_check_->isChecked()) {
                QMessageBox::warning(this, QStringLiteral("EchoVerse Sonar Lab Launcher"),
                                     QStringLiteral("Select at least one sonar module."));
                return;
            }
            stack_->setCurrentIndex(3);
        });
        connect(final_back, &QPushButton::clicked, this, [this]() { stack_->setCurrentIndex(2); });
        connect(create_btn, &QPushButton::clicked, this, &NewProjectDialog::onCreate);

        stack_->setCurrentIndex(0);
    }

    QString createdPath() const { return created_path_; }

private:
    void onCreate() {
        const QString root = path_edit_->text().trimmed();
        const QString base_name = sanitizedProjectBaseName(name_edit_->text());
        if (root.isEmpty() || !QDir(root).exists()) {
            QMessageBox::warning(this, QStringLiteral("EchoVerse Sonar Lab Launcher"),
                                 QStringLiteral("Select a valid workspace folder to continue."));
            return;
        }
        if (!fls_check_->isChecked() && !mbes_check_->isChecked() && !sss_check_->isChecked()) {
            QMessageBox::warning(this, QStringLiteral("EchoVerse Sonar Lab Launcher"),
                                 QStringLiteral("Select at least one sonar module."));
            return;
        }
        const QString project_dir = QDir(root).filePath(base_name);
        if (!QDir().mkpath(project_dir)) {
            QMessageBox::critical(this, QStringLiteral("EchoVerse Sonar Lab Launcher"),
                                  QStringLiteral("Unable to create the project folder."));
            return;
        }
        const QString world_key = worldKeyFromWizardText(world_combo_->currentText());
        QString copy_err;
        if (!copyWorldIntoProjectScene(world_key, project_dir, &copy_err)) {
            QMessageBox::critical(this, QStringLiteral("EchoVerse Sonar Lab Launcher"), copy_err);
            return;
        }
        const QString eslproj = QDir(project_dir).filePath(base_name + QString::fromUtf8(standalone_mvp::kEslprojSuffix));
        if (QFile::exists(eslproj)) {
            const auto r =
                QMessageBox::question(this, QStringLiteral("EchoVerse Sonar Lab Launcher"),
                                      QStringLiteral("A project file with the same name already exists.\nDo you want to overwrite it?\n%1").arg(eslproj));
            if (r != QMessageBox::Yes) {
                return;
            }
        }
        standalone_mvp::AppConfigData cfg = standalone_mvp::makeWizardProjectConfig(
            base_name, fls_check_->isChecked(), mbes_check_->isChecked(), sss_check_->isChecked());
        cfg.scene.world = QStringLiteral("uwmodels/scenes/%1/%2.world").arg(world_key, world_key);
        cfg.pose.x = pos_x_->value();
        cfg.pose.y = pos_y_->value();
        cfg.pose.z = pos_z_->value();
        cfg.pose.step_xy = step_xy_->value();
        cfg.pose.step_z = step_z_->value();
        cfg.pose.step_yaw_deg = step_yaw_deg_->value();
        cfg.camera_system.main_camera.yaw_deg = main_yaw_deg_->value();
        cfg.camera_system.main_camera.pitch_deg = main_pitch_deg_->value();
        cfg.camera.yaw_deg = cfg.camera_system.main_camera.yaw_deg;
        cfg.camera.pitch_deg = cfg.camera_system.main_camera.pitch_deg;
        standalone_mvp::ensureSonarParamFilesForProject(cfg, project_dir);
        standalone_mvp::AppConfigStore store(eslproj);
        if (!store.save(cfg)) {
            QMessageBox::critical(this, QStringLiteral("EchoVerse Sonar Lab Launcher"),
                                  QStringLiteral("Unable to save the project file."));
            return;
        }
        created_path_ = eslproj;
        accept();
    }

    static QDoubleSpinBox* createSpin(double min_v, double max_v, double v) {
        auto* s = new QDoubleSpinBox();
        s->setRange(min_v, max_v);
        s->setDecimals(3);
        s->setValue(v);
        return s;
    }

    QStackedWidget* stack_ = nullptr;
    QLineEdit* path_edit_ = nullptr;
    QLineEdit* name_edit_ = nullptr;
    QComboBox* world_combo_ = nullptr;
    QDoubleSpinBox* pos_x_ = nullptr;
    QDoubleSpinBox* pos_y_ = nullptr;
    QDoubleSpinBox* pos_z_ = nullptr;
    QDoubleSpinBox* step_xy_ = nullptr;
    QDoubleSpinBox* step_z_ = nullptr;
    QDoubleSpinBox* step_yaw_deg_ = nullptr;
    QDoubleSpinBox* main_yaw_deg_ = nullptr;
    QDoubleSpinBox* main_pitch_deg_ = nullptr;
    QCheckBox* fls_check_ = nullptr;
    QCheckBox* mbes_check_ = nullptr;
    QCheckBox* sss_check_ = nullptr;
    QString created_path_;
};

class LauncherWindow final : public QWidget {
public:
    explicit LauncherWindow() {
        setWindowTitle(QStringLiteral("EchoVerse Sonar Lab Launcher"));
        resize(700, 520);
        setStyleSheet(
            "QWidget{background:#060c16;color:#e7f1ff;font-family:'Segoe UI';font-size:13px;}"
            "QLabel{color:#d7e9ff;}"
            "QPushButton{background:qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #123c73,stop:1 #1a5ca8);"
            "color:white;border:1px solid #2a76c9;border-radius:10px;padding:8px 14px;font-weight:600;}"
            "QPushButton:hover{background:qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #16508f,stop:1 #2572c8);}"
            "QPushButton:pressed{background:#184a84;}"
            "QListWidget{background:#0a1424;border:1px solid #1f3a59;border-radius:10px;padding:6px;}"
            "QListWidget::item{padding:8px 10px;border-radius:8px;color:#d7e9ff;}"
            "QListWidget::item:selected{background:#174c83;color:#ffffff;}"
            "QListWidget::item:hover{background:#11345a;}");
        auto* layout = new QVBoxLayout(this);
        layout->setContentsMargins(16, 16, 16, 16);
        layout->setSpacing(12);

        auto* hero = new QLabel(
            QStringLiteral(
                "<div style='padding:14px 16px;border:1px solid #1f3a59;border-radius:12px;"
                "background:qlineargradient(x1:0,y1:0,x2:1,y2:1,stop:0 #0c1c33,stop:1 #0b2745);'>"
                "<div style='font-size:22px;font-weight:700;color:#f2f8ff;'>EchoVerse Sonar Lab</div>"
                "<div style='margin-top:4px;color:#99bee8;'>Launcher</div>"
                "<div style='margin-top:10px;color:#c3ddfb;'>Create, open, and manage sonar simulation projects with a streamlined workflow.</div>"
                "</div>"));
        hero->setTextFormat(Qt::RichText);
        hero->setWordWrap(true);
        layout->addWidget(hero);

        auto* row = new QHBoxLayout();
        row->setSpacing(10);
        open_btn_ = new QPushButton(QStringLiteral("Open Existing Project..."));
        new_btn_ = new QPushButton(QStringLiteral("Create New Project..."));
        row->addWidget(open_btn_);
        row->addWidget(new_btn_);
        layout->addLayout(row);

        auto* recent_title = new QLabel(
            QStringLiteral("<span style='font-size:14px;font-weight:600;color:#c7e0ff;'>Recent Projects</span>"));
        recent_title->setTextFormat(Qt::RichText);
        layout->addWidget(recent_title);
        recent_list_ = new QListWidget();
        layout->addWidget(recent_list_, 1);

        refreshRecent();

        connect(open_btn_, &QPushButton::clicked, this, &LauncherWindow::onOpenProject);
        connect(new_btn_, &QPushButton::clicked, this, &LauncherWindow::onNewProject);
        connect(recent_list_, &QListWidget::itemClicked, this, &LauncherWindow::onRecentActivated);
        connect(recent_list_, &QListWidget::itemDoubleClicked, this, &LauncherWindow::onRecentActivated);
    }

private:
    void refreshRecent() {
        recent_list_->clear();
        const QStringList paths = loadRecent();
        for (const QString& p : paths) {
            if (!QFile::exists(p)) {
                continue;
            }
            const QFileInfo fi(p);
            auto* item = new QListWidgetItem(fi.completeBaseName());
            item->setData(Qt::UserRole, p);
            item->setToolTip(p);
            recent_list_->addItem(item);
        }
    }

    void onOpenProject() {
        const QString dir = QFileDialog::getExistingDirectory(this, QStringLiteral("Select Project Folder"));
        if (dir.isEmpty()) {
            return;
        }
        const QString resolved = standalone_mvp::resolveProjectFileArgument(dir);
        if (resolved.isEmpty()) {
            QMessageBox::warning(this, QStringLiteral("EchoVerse Sonar Lab Launcher"),
                                 QStringLiteral("No .eslproj project file was found in this folder."));
            return;
        }
        if (launchEchoverseSonarLab(resolved)) {
            QApplication::quit();
        }
    }

    void onNewProject() {
        NewProjectDialog dlg(this);
        if (dlg.exec() != QDialog::Accepted) {
            return;
        }
        const QString path = dlg.createdPath();
        if (path.isEmpty()) {
            return;
        }
        if (launchEchoverseSonarLab(path)) {
            QApplication::quit();
        }
    }

    void onRecentActivated(QListWidgetItem* item) {
        if (!item) {
            return;
        }
        const QString p = item->data(Qt::UserRole).toString();
        if (p.isEmpty() || !QFile::exists(p)) {
            QMessageBox::warning(this, QStringLiteral("EchoVerse Sonar Lab Launcher"),
                                 QStringLiteral("The selected project file no longer exists."));
            refreshRecent();
            return;
        }
        if (launchEchoverseSonarLab(p)) {
            QApplication::quit();
        }
    }

    QPushButton* open_btn_ = nullptr;
    QPushButton* new_btn_ = nullptr;
    QListWidget* recent_list_ = nullptr;
};

} // namespace

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    QCoreApplication::setOrganizationName(QString::fromUtf8(kSettingsOrg));
    QCoreApplication::setApplicationName(QString::fromUtf8(kSettingsApp));

    LauncherWindow w;
    w.show();
    return app.exec();
}
