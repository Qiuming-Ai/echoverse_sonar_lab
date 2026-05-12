#include "WorldFileIo.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSet>
#include <QStringList>

namespace standalone_mvp {
namespace {

QString trim(const QString& s) {
    return s.trimmed();
}

std::array<double, 6> parsePoseNumbers(const QString& pose_str) {
    std::array<double, 6> pose{};
    const QStringList parts = trim(pose_str).simplified().split(QLatin1Char(' '), Qt::SkipEmptyParts);
    for (int i = 0; i < 6 && i < parts.size(); ++i) {
        bool ok = false;
        const double v = parts[i].toDouble(&ok);
        pose[static_cast<std::size_t>(i)] = ok ? v : 0.0;
    }
    return pose;
}

QString poseToString(const std::array<double, 6>& p) {
    QString s;
    for (int i = 0; i < 6; ++i) {
        if (i) {
            s += QLatin1Char(' ');
        }
        s += QString::number(p[static_cast<std::size_t>(i)], 'g', 12);
    }
    return s;
}

QString getTagContent(const QString& src, const QString& tag) {
    const QString open = QLatin1Char('<') + tag + QLatin1Char('>');
    const QString close = QLatin1String("</") + tag + QLatin1Char('>');
    const int p0 = src.indexOf(open);
    const int p1 = src.indexOf(close);
    if (p0 < 0 || p1 < 0 || p1 <= p0 + open.size()) {
        return {};
    }
    return trim(src.mid(p0 + open.size(), p1 - (p0 + open.size())));
}

QString uriToModelName(const QString& uri) {
    const QString prefix = QStringLiteral("model://");
    if (uri.startsWith(prefix, Qt::CaseInsensitive)) {
        return uri.mid(prefix.size());
    }
    return uri;
}

bool replaceIncludeSection(QString* xml, const QString& new_include_blocks) {
    if (!xml) {
        return false;
    }
    QString& content = *xml;
    const int first = content.indexOf(QStringLiteral("<include>"));
    if (first < 0) {
        int ins = content.indexOf(QStringLiteral("<plugin"));
        if (ins < 0) {
            ins = content.indexOf(QStringLiteral("<gui"));
        }
        if (ins < 0) {
            ins = content.indexOf(QStringLiteral("</world>"));
        }
        if (ins < 0) {
            return false;
        }
        content.insert(ins, new_include_blocks);
        return true;
    }
    int boundary = content.indexOf(QStringLiteral("<plugin"), first);
    const int gui = content.indexOf(QStringLiteral("<gui"), first);
    if (boundary < 0 || (gui >= 0 && gui < boundary)) {
        boundary = gui;
    }
    if (boundary < 0) {
        boundary = content.indexOf(QStringLiteral("</world>"), first);
    }
    if (boundary < 0) {
        return false;
    }
    const QString before_boundary = content.left(boundary);
    const int last_close = before_boundary.lastIndexOf(QStringLiteral("</include>"));
    if (last_close < 0) {
        return false;
    }
    const int after_last = last_close + QStringLiteral("</include>").size();
    content = content.left(first) + new_include_blocks + content.mid(after_last);
    return true;
}

QString formatIncludeBlock(const QString& model_name, const std::array<double, 6>& pose) {
    return QStringLiteral("    <include>\n      <uri>model://%1</uri>\n      <pose>%2</pose>\n    </include>\n")
        .arg(model_name, poseToString(pose));
}

} // namespace

QStringList discoverSdfModelNames() {
    QStringList roots;
#if defined(STANDALONE_SIMULATION_DIR)
    roots << (QStringLiteral(STANDALONE_SIMULATION_DIR) + QStringLiteral("/uwmodels/sdf"));
#endif
    roots << (QDir::currentPath() + QStringLiteral("/uwmodels/sdf"));
    roots << (QCoreApplication::applicationDirPath() + QStringLiteral("/uwmodels/sdf"));
    QSet<QString> names;
    for (const QString& r : roots) {
        QDir d(r);
        if (!d.exists()) {
            continue;
        }
        const QFileInfoList dirs = d.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot);
        for (const QFileInfo& fi : dirs) {
            const QString sdf = fi.absoluteFilePath() + QStringLiteral("/model.sdf");
            if (QFile::exists(sdf)) {
                names.insert(fi.fileName());
            }
        }
    }
    QStringList out = names.values();
    out.sort(Qt::CaseInsensitive);
    return out;
}

bool loadWorldIncludes(const QString& world_file_path, QVector<WorldIncludeEntry>* out, QString* error) {
    if (!out) {
        return false;
    }
    out->clear();
    QFile f(world_file_path);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (error) {
            *error = QStringLiteral("无法读取：%1").arg(world_file_path);
        }
        return false;
    }
    const QString xml = QString::fromUtf8(f.readAll());
    f.close();
    int pos = 0;
    while (true) {
        const int b = xml.indexOf(QStringLiteral("<include>"), pos);
        if (b < 0) {
            break;
        }
        const int e = xml.indexOf(QStringLiteral("</include>"), b);
        if (e < 0) {
            break;
        }
        const int include_close_len = QStringLiteral("</include>").size();
        const QString block = xml.mid(b, e + include_close_len - b);
        const QString uri = getTagContent(block, QStringLiteral("uri"));
        if (uri.isEmpty()) {
            pos = e + 1;
            continue;
        }
        WorldIncludeEntry ent;
        ent.model_name = uriToModelName(uri);
        ent.pose = parsePoseNumbers(getTagContent(block, QStringLiteral("pose")));
        out->push_back(ent);
        pos = e + 1;
    }
    return true;
}

bool saveWorldIncludes(const QString& world_file_path, const QVector<WorldIncludeEntry>& entries, QString* error) {
    QFile f(world_file_path);
    if (!f.open(QIODevice::ReadWrite | QIODevice::Text)) {
        if (error) {
            *error = QStringLiteral("无法读写：%1").arg(world_file_path);
        }
        return false;
    }
    QString xml = QString::fromUtf8(f.readAll());
    f.close();
    QString blocks;
    for (const WorldIncludeEntry& ent : entries) {
        blocks += formatIncludeBlock(ent.model_name, ent.pose);
    }
    if (!replaceIncludeSection(&xml, blocks)) {
        if (error) {
            *error = QStringLiteral("无法定位 include 区域，未写入。");
        }
        return false;
    }
    QFile out(world_file_path);
    if (!out.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        if (error) {
            *error = QStringLiteral("无法写入：%1").arg(world_file_path);
        }
        return false;
    }
    out.write(xml.toUtf8());
    out.close();
    return true;
}

} // namespace standalone_mvp
