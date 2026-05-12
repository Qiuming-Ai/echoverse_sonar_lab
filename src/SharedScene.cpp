#include "SharedScene.hpp"

#include <osg/Camera>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Light>
#include <osg/LightModel>
#include <osg/LightSource>
#include <osg/Material>
#include <osg/MatrixTransform>
#include <osg/NodeVisitor>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osg/TexMat>
#include <osg/Texture2D>
#include <osg/Viewport>
#include <osgDB/Registry>
#include <osgDB/FileUtils>
#include <osgDB/ReadFile>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>

#include <array>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include <filesystem>
#if defined(_WIN32)
#include <windows.h>
#endif

namespace standalone_mvp {

namespace {
#if defined(_WIN32)
// Forward declaration used by resolveRepoPath() to provide relocatable runtime fallback.
std::filesystem::path getExecutableDir();
#endif

struct ParsedMaterial {
    std::optional<osg::Vec4> ambient;
    std::optional<osg::Vec4> diffuse;
    std::optional<osg::Vec4> specular;
    std::optional<osg::Vec4> emissive;
    std::optional<std::string> texture_rel_path;
    std::optional<std::string> texture_filtering;
    std::optional<float> texture_max_anisotropy;
    std::optional<osg::Vec2f> texture_scale;
};

struct SceneModel {
    enum class PrimitiveKind {
        Box,
        Cylinder,
        Sphere
    };
    std::string mesh_path;
    bool skip_render = false; // e.g. light-only helper models
    std::optional<osg::Vec3d> primitive_box_size;
    std::optional<osg::Vec2d> primitive_cylinder_size; // (radius, length)
    std::optional<double> primitive_sphere_radius;
    std::optional<osg::Vec2d> primitive_plane_size; // (x, y)
    ParsedMaterial material;
    std::array<double, 6> pose_xyzrpy;
    osg::Vec4 fallback_color;
    bool fallback_is_plane;
    bool visible = true;
};

struct SceneSpec {
    const char* world_name;
    std::vector<SceneModel> models;
};

struct WorldInclude {
    std::string model_name;
    std::array<double, 6> pose_xyzrpy{0, 0, 0, 0, 0, 0};
};

osg::ref_ptr<osg::Node> createFallbackNode(float range_m, const SceneModel& model) {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    if (model.primitive_box_size.has_value()) {
        const osg::Vec3d size = *model.primitive_box_size;
        geode->addDrawable(new osg::ShapeDrawable(
            new osg::Box(osg::Vec3(0.0f, 0.0f, 0.0f),
                         static_cast<float>(std::abs(size.x())),
                         static_cast<float>(std::abs(size.y())),
                         static_cast<float>(std::abs(size.z())))));
    } else if (model.primitive_cylinder_size.has_value()) {
        const osg::Vec2d cyl = *model.primitive_cylinder_size;
        geode->addDrawable(new osg::ShapeDrawable(
            new osg::Cylinder(
                osg::Vec3(0.0f, 0.0f, 0.0f),
                static_cast<float>(std::max(0.01, std::abs(cyl.x()))),
                static_cast<float>(std::max(0.01, std::abs(cyl.y()))))));
    } else if (model.primitive_sphere_radius.has_value()) {
        geode->addDrawable(new osg::ShapeDrawable(
            new osg::Sphere(
                osg::Vec3(0.0f, 0.0f, 0.0f),
                static_cast<float>(std::max(0.01, std::abs(*model.primitive_sphere_radius))))));
    } else if (model.primitive_plane_size.has_value()) {
        const osg::Vec2d size = *model.primitive_plane_size;
        geode->addDrawable(new osg::ShapeDrawable(
            new osg::Box(
                osg::Vec3(0.0f, 0.0f, 0.0f),
                static_cast<float>(std::max(0.05, std::abs(size.x()))),
                static_cast<float>(std::max(0.05, std::abs(size.y()))),
                0.05f)));
    } else if (model.fallback_is_plane) {
        geode->addDrawable(new osg::ShapeDrawable(
            new osg::Box(osg::Vec3(0.0f, 0.0f, 0.0f), range_m * 4.0f, range_m * 4.0f, 0.8f)));
    } else {
        geode->addDrawable(new osg::ShapeDrawable(
            new osg::Box(osg::Vec3(0.0f, 0.0f, 0.0f), 8.0f)));
    }
    auto* drawable = geode->getDrawable(0);
    if (drawable) {
        auto* shape = dynamic_cast<osg::ShapeDrawable*>(drawable);
        if (shape) {
            shape->setColor(model.fallback_color);
        }
    }
    return geode;
}

std::string resolveRepoPath(const std::string& relative_path) {
#if defined(STANDALONE_SIMULATION_DIR)
    const std::string base = std::string(STANDALONE_SIMULATION_DIR);
    std::string candidate = base + "/" + relative_path;
    std::error_code ec;
    if (std::filesystem::exists(candidate, ec)) {
        return candidate;
    }
#if defined(_WIN32)
    // If the binary was packaged/moved and the original source path no longer exists,
    // fall back to assets adjacent to the executable (e.g. ./uwmodels, ./src/...).
    const std::filesystem::path exe_dir = getExecutableDir();
    if (!exe_dir.empty()) {
        std::filesystem::path p = exe_dir / relative_path;
        if (std::filesystem::exists(p, ec)) {
            return p.generic_string();
        }
    }
#endif
    // Last resort: keep the original candidate (helps diagnosing missing assets).
    return candidate;
#else
    return relative_path;
#endif
}

std::string trim(const std::string& s) {
    const auto b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) {
        return "";
    }
    const auto e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

std::string getTagContent(const std::string& src, const std::string& tag) {
    const std::string open = "<" + tag + ">";
    const std::string close = "</" + tag + ">";
    const std::size_t p0 = src.find(open);
    const std::size_t p1 = src.find(close);
    if (p0 == std::string::npos || p1 == std::string::npos || p1 <= p0 + open.size()) {
        return "";
    }
    return trim(src.substr(p0 + open.size(), p1 - (p0 + open.size())));
}

std::array<double, 6> parsePose(const std::string& pose_str) {
    std::array<double, 6> pose{0, 0, 0, 0, 0, 0};
    if (pose_str.empty()) {
        return pose;
    }
    std::stringstream ss(pose_str);
    for (double& v : pose) {
        ss >> v;
        if (!ss.good() && !ss.eof()) {
            break;
        }
    }
    return pose;
}

std::string uriToModelName(const std::string& uri) {
    const std::string prefix = "model://";
    if (uri.rfind(prefix, 0) == 0) {
        return uri.substr(prefix.size());
    }
    return uri;
}

std::string worldSpecToSceneKey(const std::string& world_spec) {
    if (world_spec.empty()) {
        return "ssiv_bahia";
    }
    std::filesystem::path p(world_spec);
    if (p.has_extension() && p.extension() == ".world") {
        return p.stem().string();
    }
    return world_spec;
}

std::string resolveWorldFilePath(const std::string& world_spec) {
    const std::string key = worldSpecToSceneKey(world_spec);
    const std::filesystem::path direct(world_spec);
    std::error_code ec;
    if (!world_spec.empty() && std::filesystem::exists(direct, ec) && std::filesystem::is_regular_file(direct, ec)) {
        return direct.string();
    }
    const std::string via_repo = resolveRepoPath(world_spec);
    if (!world_spec.empty() && std::filesystem::exists(via_repo, ec) && std::filesystem::is_regular_file(via_repo, ec)) {
        return via_repo;
    }
    return resolveRepoPath("uwmodels/scenes/" + key + "/" + key + ".world");
}

std::optional<std::vector<WorldInclude>> parseWorldIncludes(const std::string& world_spec) {
    const std::string world_path = resolveWorldFilePath(world_spec);
    std::ifstream in(world_path);
    if (!in) {
        return std::nullopt;
    }
    std::stringstream buffer;
    buffer << in.rdbuf();
    const std::string xml = buffer.str();

    std::vector<WorldInclude> includes;
    std::size_t pos = 0;
    while (true) {
        const std::size_t b = xml.find("<include>", pos);
        if (b == std::string::npos) {
            break;
        }
        const std::size_t e = xml.find("</include>", b);
        if (e == std::string::npos) {
            break;
        }
        const std::string block = xml.substr(b, e + std::string("</include>").size() - b);
        WorldInclude inc;
        const std::string uri = getTagContent(block, "uri");
        if (uri.empty()) {
            pos = e + 1;
            continue;
        }
        inc.model_name = uriToModelName(uri);
        inc.pose_xyzrpy = parsePose(getTagContent(block, "pose"));
        includes.push_back(inc);
        pos = e + 1;
    }
    return includes;
}

std::string uriToRelativeMeshPath(const std::string& uri) {
    const std::string prefix = "model://";
    if (uri.rfind(prefix, 0) == 0) {
        std::string rem = uri.substr(prefix.size()); // e.g. seafloor/visual.osgb
        return "uwmodels/sdf/" + rem;
    }
    const std::string media_prefix_upper = "file://Media/";
    if (uri.rfind(media_prefix_upper, 0) == 0) {
        return "uwmodels/Media/" + uri.substr(media_prefix_upper.size());
    }
    const std::string media_prefix_lower = "file://media/";
    if (uri.rfind(media_prefix_lower, 0) == 0) {
        return "uwmodels/media/" + uri.substr(media_prefix_lower.size());
    }
    return uri;
}

std::optional<osg::Vec3d> parseFirstBoxSize(const std::string& xml, const std::string& block_tag) {
    const std::size_t block_pos = xml.find("<" + block_tag);
    if (block_pos == std::string::npos) {
        return std::nullopt;
    }
    const std::size_t block_end = xml.find("</" + block_tag + ">", block_pos);
    const std::string block = xml.substr(block_pos, (block_end == std::string::npos ? xml.size() : block_end) - block_pos);
    const std::size_t box_pos = block.find("<box>");
    if (box_pos == std::string::npos) {
        return std::nullopt;
    }
    const std::size_t box_end = block.find("</box>", box_pos);
    const std::string box_block = block.substr(box_pos, (box_end == std::string::npos ? block.size() : box_end) - box_pos);
    const std::string size_str = getTagContent(box_block, "size");
    if (size_str.empty()) {
        return std::nullopt;
    }
    std::stringstream ss(size_str);
    double sx = 0.0, sy = 0.0, sz = 0.0;
    ss >> sx >> sy >> sz;
    if (!ss.good() && !ss.eof()) {
        return std::nullopt;
    }
    return osg::Vec3d(sx, sy, sz);
}

std::string parseFirstMeshUri(const std::string& xml, const std::string& block_tag) {
    const std::size_t block_pos = xml.find("<" + block_tag);
    if (block_pos == std::string::npos) {
        return "";
    }
    const std::size_t block_end = xml.find("</" + block_tag + ">", block_pos);
    const std::string block = xml.substr(block_pos, (block_end == std::string::npos ? xml.size() : block_end) - block_pos);
    const std::size_t mesh_pos = block.find("<mesh>");
    if (mesh_pos == std::string::npos) {
        return "";
    }
    const std::size_t mesh_end = block.find("</mesh>", mesh_pos);
    const std::string mesh_block = block.substr(mesh_pos, (mesh_end == std::string::npos ? block.size() : mesh_end) - mesh_pos);
    return getTagContent(mesh_block, "uri");
}

struct ModelGeometrySpec {
    struct GeometryEntry {
        std::string mesh_path;
        std::optional<osg::Vec3d> primitive_box_size;
        std::optional<osg::Vec2d> primitive_cylinder_size;
        std::optional<double> primitive_sphere_radius;
        std::optional<osg::Vec2d> primitive_plane_size;
        osg::Vec4 color = osg::Vec4(0.35f, 0.45f, 0.55f, 1.0f);
        ParsedMaterial material;
        std::array<double, 6> local_pose_xyzrpy{0, 0, 0, 0, 0, 0};
        bool visible = true;
    };
    bool is_light_only = false;
    std::array<double, 6> local_pose_xyzrpy{0, 0, 0, 0, 0, 0};
    std::vector<GeometryEntry> geometries;
};

std::string vec4ToString(const osg::Vec4& v) {
    std::ostringstream oss;
    oss << "(" << v.r() << "," << v.g() << "," << v.b() << "," << v.a() << ")";
    return oss.str();
}

std::string parentDir(const std::string& rel_path) {
    const std::size_t pos = rel_path.find_last_of('/');
    if (pos == std::string::npos) {
        return "";
    }
    return rel_path.substr(0, pos);
}

std::string joinPath(const std::string& a, const std::string& b) {
    if (a.empty()) return b;
    if (b.empty()) return a;
    if (a.back() == '/') return a + b;
    return a + "/" + b;
}

std::string normalizeGenericPath(const std::string& in) {
    std::filesystem::path p(in);
    return p.lexically_normal().generic_string();
}

ModelGeometrySpec parseModelGeometrySpec(const std::string& model_name) {
    ModelGeometrySpec spec;
    const std::string model_sdf_path = resolveRepoPath("uwmodels/sdf/" + model_name + "/model.sdf");
    std::ifstream in(model_sdf_path);
    if (!in) {
        return spec;
    }
    std::stringstream buffer;
    buffer << in.rdbuf();
    const std::string xml = buffer.str();
    if (xml.find("<light") != std::string::npos && xml.find("<model") == std::string::npos) {
        spec.is_light_only = true;
        return spec;
    }

    auto parseColorFromName = [](const std::string& material_name) {
        if (material_name.find("Red") != std::string::npos) return osg::Vec4(0.85f, 0.20f, 0.20f, 1.0f);
        if (material_name.find("Green") != std::string::npos) return osg::Vec4(0.20f, 0.85f, 0.20f, 1.0f);
        if (material_name.find("Blue") != std::string::npos) return osg::Vec4(0.20f, 0.35f, 0.90f, 1.0f);
        if (material_name.find("Grey") != std::string::npos || material_name.find("Gray") != std::string::npos) return osg::Vec4(0.55f, 0.55f, 0.58f, 1.0f);
        return osg::Vec4(0.35f, 0.45f, 0.55f, 1.0f);
    };
    auto parseVec4 = [](const std::string& text) -> std::optional<osg::Vec4> {
        if (text.empty()) return std::nullopt;
        std::stringstream ss(text);
        double r = 0.0, g = 0.0, b = 0.0, a = 1.0;
        ss >> r >> g >> b;
        if (!ss.good() && ss.eof()) return std::nullopt;
        if (ss.good()) ss >> a;
        return osg::Vec4(static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), static_cast<float>(a));
    };
    auto parseMaterialScriptColor = [&](const std::string& material_uri, const std::string& material_name) -> ParsedMaterial {
        ParsedMaterial out;
        std::string material_rel = uriToRelativeMeshPath(material_uri);
        if (material_rel.rfind("uwmodels/", 0) != 0) {
            material_rel = "uwmodels/sdf/" + model_name + "/" + material_rel;
        }
        std::cout << "[material] script uri=" << material_uri
                  << " resolved=" << material_rel
                  << " script=" << material_name
                  << " model=" << model_name << std::endl;
        std::ifstream mat_in(resolveRepoPath(material_rel));
        if (!mat_in) {
            std::cout << "[material] file open failed: " << resolveRepoPath(material_rel) << std::endl;
            return out;
        }
        std::stringstream mat_buf;
        mat_buf << mat_in.rdbuf();
        const std::string mat_txt = mat_buf.str();
        const std::string material_dir = parentDir(material_rel);
        std::size_t start = 0;
        if (!material_name.empty()) {
            const std::string needle = "material " + material_name;
            const std::size_t p = mat_txt.find(needle);
            if (p != std::string::npos) start = p;
        }
        auto parseKeyword = [&](const std::string& kw) -> std::optional<osg::Vec4> {
            const std::size_t p = mat_txt.find(kw, start);
            if (p == std::string::npos) return std::nullopt;
            std::size_t eol = mat_txt.find('\n', p);
            if (eol == std::string::npos) eol = mat_txt.size();
            const std::string line = trim(mat_txt.substr(p + kw.size(), eol - (p + kw.size())));
            return parseVec4(line);
        };
        out.ambient = parseKeyword("ambient");
        out.diffuse = parseKeyword("diffuse");
        out.specular = parseKeyword("specular");
        out.emissive = parseKeyword("emissive");

        const std::size_t tex_unit = mat_txt.find("texture_unit", start);
        if (tex_unit != std::string::npos) {
            std::size_t cursor = tex_unit;
            bool opened_block = false;
            int brace_depth = 0;
            while (cursor < mat_txt.size()) {
                std::size_t eol = mat_txt.find('\n', cursor);
                if (eol == std::string::npos) {
                    eol = mat_txt.size();
                }
                const std::string line = trim(mat_txt.substr(cursor, eol - cursor));
                if (!opened_block && line == "{") {
                    opened_block = true;
                    brace_depth = 1;
                    if (eol >= mat_txt.size()) break;
                    cursor = eol + 1;
                    continue;
                }
                if (opened_block && line == "{") {
                    ++brace_depth;
                } else if (opened_block && line == "}") {
                    --brace_depth;
                    if (brace_depth <= 0) {
                        break;
                    }
                } else if (!line.empty() && line.rfind("texture ", 0) == 0) {
                    std::stringstream tex_ss(line.substr(std::string("texture ").size()));
                    std::string tex_token;
                    tex_ss >> tex_token;
                    if (!tex_token.empty()) {
                        std::string texture_rel = uriToRelativeMeshPath(tex_token);
                        if (texture_rel.rfind("uwmodels/", 0) != 0) {
                            texture_rel = normalizeGenericPath(joinPath(material_dir, texture_rel));
                        }
                        out.texture_rel_path = texture_rel;
                    }
                } else if (!line.empty() && line.rfind("filtering ", 0) == 0) {
                    out.texture_filtering = trim(line.substr(std::string("filtering ").size()));
                } else if (!line.empty() && line.rfind("max_anisotropy ", 0) == 0) {
                    std::stringstream aniso_ss(line.substr(std::string("max_anisotropy ").size()));
                    float aniso = 1.0f;
                    aniso_ss >> aniso;
                    if (!aniso_ss.fail()) {
                        out.texture_max_anisotropy = std::max(1.0f, aniso);
                    }
                } else if (!line.empty() && line.rfind("scale ", 0) == 0) {
                    std::stringstream scale_ss(line.substr(std::string("scale ").size()));
                    float sx = 1.0f;
                    float sy = 1.0f;
                    scale_ss >> sx >> sy;
                    if (!scale_ss.fail()) {
                        out.texture_scale = osg::Vec2f(sx, sy);
                    }
                }
                if (line.rfind("material ", 0) == 0 && cursor > tex_unit) {
                    break;
                }
                if (eol >= mat_txt.size()) {
                    break;
                }
                cursor = eol + 1;
            }
        }
        std::cout << "[material] parsed script model=" << model_name
                  << " ambient=" << (out.ambient ? vec4ToString(*out.ambient) : "none")
                  << " diffuse=" << (out.diffuse ? vec4ToString(*out.diffuse) : "none")
                  << " specular=" << (out.specular ? vec4ToString(*out.specular) : "none")
                  << " emissive=" << (out.emissive ? vec4ToString(*out.emissive) : "none")
                  << " texture=" << (out.texture_rel_path ? *out.texture_rel_path : "none")
                  << " filtering=" << (out.texture_filtering ? *out.texture_filtering : "none")
                  << " max_anisotropy=" << (out.texture_max_anisotropy ? std::to_string(*out.texture_max_anisotropy) : "none")
                  << " scale=" << (out.texture_scale ? ("(" + std::to_string(out.texture_scale->x()) + "," + std::to_string(out.texture_scale->y()) + ")") : "none")
                  << std::endl;
        return out;
    };
    auto parseMaterialFromSdf = [&](const std::string& src) {
        ParsedMaterial out;
        const std::string material_block = getTagContent(src, "material");
        const std::string script_block = getTagContent(material_block, "script");
        const std::string uri = getTagContent(script_block, "uri");
        const std::string script_name = getTagContent(script_block, "name");
        if (!uri.empty()) {
            out = parseMaterialScriptColor(uri, script_name);
        }
        const std::string sdf_diffuse = getTagContent(material_block, "diffuse");
        const std::string sdf_ambient = getTagContent(material_block, "ambient");
        const std::string sdf_emissive = getTagContent(material_block, "emissive");
        if (!out.diffuse && !sdf_diffuse.empty()) out.diffuse = parseVec4(sdf_diffuse);
        if (!out.ambient && !sdf_ambient.empty()) out.ambient = parseVec4(sdf_ambient);
        if (!out.emissive && !sdf_emissive.empty()) out.emissive = parseVec4(sdf_emissive);
        // Only infer color by material name when SDF actually provides a material block.
        // If there is no material at all, preserve the mesh's original embedded shading.
        if (!out.diffuse && !material_block.empty()) {
            out.diffuse = parseColorFromName(script_name.empty() ? getTagContent(src, "name") : script_name);
        }
        if (!out.ambient && out.diffuse) {
            out.ambient = osg::Vec4(out.diffuse->r() * 0.6f, out.diffuse->g() * 0.6f, out.diffuse->b() * 0.6f, out.diffuse->a());
        }
        std::cout << "[material] sdf merged model=" << model_name
                  << " diffuse=" << (out.diffuse ? vec4ToString(*out.diffuse) : "none")
                  << " ambient=" << (out.ambient ? vec4ToString(*out.ambient) : "none")
                  << " specular=" << (out.specular ? vec4ToString(*out.specular) : "none")
                  << " emissive=" << (out.emissive ? vec4ToString(*out.emissive) : "none")
                  << " texture=" << (out.texture_rel_path ? *out.texture_rel_path : "none")
                  << " filtering=" << (out.texture_filtering ? *out.texture_filtering : "none")
                  << " max_anisotropy=" << (out.texture_max_anisotropy ? std::to_string(*out.texture_max_anisotropy) : "none")
                  << " scale=" << (out.texture_scale ? ("(" + std::to_string(out.texture_scale->x()) + "," + std::to_string(out.texture_scale->y()) + ")") : "none")
                  << std::endl;
        return out;
    };
    auto parseFirstByTag = [](const std::string& src, const std::string& tag) {
        const std::string open = "<" + tag + ">";
        const std::string close = "</" + tag + ">";
        const std::size_t b = src.find(open);
        if (b == std::string::npos) return std::string();
        const std::size_t e = src.find(close, b);
        if (e == std::string::npos) return std::string();
        return src.substr(b + open.size(), e - (b + open.size()));
    };
    auto extractBlocks = [](const std::string& src, const std::string& tag) {
        std::vector<std::string> blocks;
        std::size_t pos = 0;
        while (true) {
            const std::size_t b = src.find("<" + tag, pos);
            if (b == std::string::npos) break;
            const std::size_t body = src.find(">", b);
            if (body == std::string::npos) break;
            const std::size_t e = src.find("</" + tag + ">", body);
            if (e == std::string::npos) break;
            blocks.push_back(src.substr(b, e + tag.size() + 3 - b));
            pos = e + tag.size() + 3;
        }
        return blocks;
    };
    auto parseFirstCylinder = [&](const std::string& src) -> std::optional<osg::Vec2d> {
        const std::string cyl = parseFirstByTag(src, "cylinder");
        if (cyl.empty()) return std::nullopt;
        const std::string r = getTagContent(cyl, "radius");
        const std::string l = getTagContent(cyl, "length");
        if (r.empty() || l.empty()) return std::nullopt;
        return osg::Vec2d(std::stod(r), std::stod(l));
    };
    auto parseFirstSphere = [&](const std::string& src) -> std::optional<double> {
        const std::string sph = parseFirstByTag(src, "sphere");
        if (sph.empty()) return std::nullopt;
        const std::string r = getTagContent(sph, "radius");
        if (r.empty()) return std::nullopt;
        return std::stod(r);
    };
    auto parseFirstPlane = [&](const std::string& src) -> std::optional<osg::Vec2d> {
        const std::string plane = parseFirstByTag(src, "plane");
        if (plane.empty()) return std::nullopt;
        const std::string size = getTagContent(plane, "size");
        if (size.empty()) return std::nullopt;
        std::stringstream ss(size);
        double sx = 0.0, sy = 0.0;
        ss >> sx >> sy;
        if (!ss.good() && !ss.eof()) return std::nullopt;
        return osg::Vec2d(sx, sy);
    };
    auto parseBoxFromBlock = [&](const std::string& block) -> std::optional<osg::Vec3d> {
        const std::string box = parseFirstByTag(block, "box");
        if (box.empty()) return std::nullopt;
        const std::string size = getTagContent(box, "size");
        if (size.empty()) return std::nullopt;
        std::stringstream ss(size);
        double sx = 0.0, sy = 0.0, sz = 0.0;
        ss >> sx >> sy >> sz;
        if (!ss.good() && !ss.eof()) return std::nullopt;
        return osg::Vec3d(sx, sy, sz);
    };
    auto parseMeshUriFromBlock = [&](const std::string& block) {
        const std::string mesh = parseFirstByTag(block, "mesh");
        if (mesh.empty()) return std::string();
        return getTagContent(mesh, "uri");
    };

    spec.local_pose_xyzrpy = parsePose(getTagContent(xml, "pose"));

    auto meshExists = [](const std::string& rel, const std::string& model_name_for_relative) {
        std::string candidate = uriToRelativeMeshPath(rel);
        if (candidate.rfind("uwmodels/", 0) != 0) {
            candidate = "uwmodels/sdf/" + model_name_for_relative + "/" + candidate;
        }
        std::ifstream test(resolveRepoPath(candidate), std::ios::binary);
        if (test) {
            return std::optional<std::string>(candidate);
        }
        std::ifstream test_raw(resolveRepoPath(rel), std::ios::binary);
        if (test_raw) {
            return std::optional<std::string>(rel);
        }
        return std::optional<std::string>();
    };

    const auto visual_blocks = extractBlocks(xml, "visual");
    const auto collision_blocks = extractBlocks(xml, "collision");
    auto appendGeometry = [&](const std::string& block, const osg::Vec4& default_color, bool visible) {
        ModelGeometrySpec::GeometryEntry entry;
        entry.visible = visible;
        entry.material = parseMaterialFromSdf(block);
        entry.color = entry.material.diffuse.value_or(default_color);
        if (entry.color.a() <= 0.0f) entry.color = default_color;
        entry.local_pose_xyzrpy = parsePose(getTagContent(block, "pose"));
        const std::string uri = parseMeshUriFromBlock(block);
        if (!uri.empty()) {
            auto rel = meshExists(uri, model_name);
            if (rel) {
                entry.mesh_path = *rel;
                std::cout << "[material] geometry mesh=" << entry.mesh_path
                          << " diffuse=" << (entry.material.diffuse ? vec4ToString(*entry.material.diffuse) : "none")
                          << std::endl;
                spec.geometries.push_back(entry);
                return;
            }
        }
        auto b = parseBoxFromBlock(block);
        if (b) entry.primitive_box_size = *b;
        auto c = parseFirstCylinder(block);
        if (c) entry.primitive_cylinder_size = *c;
        auto s = parseFirstSphere(block);
        if (s) entry.primitive_sphere_radius = *s;
        auto p = parseFirstPlane(block);
        if (p) entry.primitive_plane_size = *p;
        if (entry.primitive_box_size || entry.primitive_cylinder_size || entry.primitive_sphere_radius || entry.primitive_plane_size) {
            std::cout << "[material] geometry primitive model=" << model_name
                      << " diffuse=" << (entry.material.diffuse ? vec4ToString(*entry.material.diffuse) : "none")
                      << std::endl;
            spec.geometries.push_back(entry);
        }
    };
    bool has_visual_geometry = false;
    for (const auto& v : visual_blocks) {
        const std::size_t before = spec.geometries.size();
        appendGeometry(v, osg::Vec4(0.35f, 0.45f, 0.55f, 1.0f), true);
        if (spec.geometries.size() > before) {
            has_visual_geometry = true;
        }
    }
    for (const auto& c : collision_blocks) {
        // Keep collision geometry, but hide it when visual geometry exists for the same model.
        appendGeometry(c, osg::Vec4(0.40f, 0.40f, 0.42f, 1.0f), !has_visual_geometry);
    }
    return spec;
}

bool hasCollisionMesh(const std::string& model_name) {
    const std::string collision_rel = "uwmodels/sdf/" + model_name + "/collision.stl";
    std::ifstream test(resolveRepoPath(collision_rel), std::ios::binary);
    return static_cast<bool>(test);
}

bool hasSuffix(const std::string& value, const std::string& suffix) {
    if (value.size() < suffix.size()) {
        return false;
    }
    return std::equal(suffix.rbegin(), suffix.rend(), value.rbegin(),
                      [](char a, char b) { return std::tolower(a) == std::tolower(b); });
}

bool tryParseVec4Line(const std::string& line, osg::Vec4& out) {
    std::stringstream ss(line);
    double x = 0.0, y = 0.0, z = 0.0, w = 1.0;
    ss >> x >> y >> z;
    if (ss.fail()) {
        return false;
    }
    ss >> w;
    if (ss.fail() && !ss.eof()) {
        return false;
    }
    out = osg::Vec4(static_cast<float>(x), static_cast<float>(y), static_cast<float>(z), static_cast<float>(w));
    return true;
}

void applyParsedMaterialRecursive(osg::Node* node, const ParsedMaterial& material) {
    if (!node) {
        return;
    }
    osg::ref_ptr<osg::Texture2D> texture;
    osg::ref_ptr<osg::TexMat> tex_mat;
    if (material.texture_rel_path) {
        const std::string texture_full = resolveRepoPath(*material.texture_rel_path);
        osg::ref_ptr<osg::Image> image = osgDB::readRefImageFile(texture_full);
        if (image.valid()) {
            texture = new osg::Texture2D(image.get());
            texture->setDataVariance(osg::Object::DYNAMIC);
            osg::Texture::FilterMode min_filter = osg::Texture::LINEAR_MIPMAP_LINEAR;
            osg::Texture::FilterMode mag_filter = osg::Texture::LINEAR;
            if (material.texture_filtering) {
                const std::string f = *material.texture_filtering;
                if (f.find("none") != std::string::npos || f.find("point") != std::string::npos) {
                    min_filter = osg::Texture::NEAREST_MIPMAP_NEAREST;
                    mag_filter = osg::Texture::NEAREST;
                } else if (f.find("bilinear") != std::string::npos) {
                    min_filter = osg::Texture::LINEAR_MIPMAP_NEAREST;
                    mag_filter = osg::Texture::LINEAR;
                } else if (f.find("trilinear") != std::string::npos || f.find("anisotropic") != std::string::npos) {
                    min_filter = osg::Texture::LINEAR_MIPMAP_LINEAR;
                    mag_filter = osg::Texture::LINEAR;
                }
            }
            texture->setFilter(osg::Texture::MIN_FILTER, min_filter);
            texture->setFilter(osg::Texture::MAG_FILTER, mag_filter);
            texture->setWrap(osg::Texture::WRAP_S, osg::Texture::REPEAT);
            texture->setWrap(osg::Texture::WRAP_T, osg::Texture::REPEAT);
            if (material.texture_max_anisotropy) {
                texture->setMaxAnisotropy(*material.texture_max_anisotropy);
            }
            if (material.texture_scale) {
                tex_mat = new osg::TexMat();
                tex_mat->setMatrix(osg::Matrix::scale(material.texture_scale->x(), material.texture_scale->y(), 1.0f));
            }
            std::cout << "[material] texture loaded " << *material.texture_rel_path << std::endl;
        } else {
            std::cout << "[material] texture load failed " << *material.texture_rel_path << std::endl;
        }
    }

    std::function<void(osg::Node*)> applyToNode = [&](osg::Node* current) {
        if (!current) {
            return;
        }
        const std::string nodeName = current->getName().empty() ? std::string("<unnamed>") : current->getName();
        osg::StateSet* ss = current->getOrCreateStateSet();
        osg::ref_ptr<osg::Material> mat = new osg::Material();
        mat->setColorMode(osg::Material::OFF);
        if (material.ambient) {
            mat->setAmbient(osg::Material::FRONT_AND_BACK, *material.ambient);
        }
        if (material.diffuse) {
            mat->setDiffuse(osg::Material::FRONT_AND_BACK, *material.diffuse);
        }
        if (material.specular) {
            mat->setSpecular(osg::Material::FRONT_AND_BACK, *material.specular);
            mat->setShininess(osg::Material::FRONT_AND_BACK, 32.0f);
        }
        if (material.emissive) {
            mat->setEmission(osg::Material::FRONT_AND_BACK, *material.emissive);
        }
        ss->setAttributeAndModes(mat.get(), osg::StateAttribute::ON);
        if (texture.valid()) {
            ss->setTextureAttributeAndModes(0, texture.get(), osg::StateAttribute::ON);
            if (tex_mat.valid()) {
                ss->setTextureAttributeAndModes(0, tex_mat.get(), osg::StateAttribute::ON);
            }
        }
        const float alpha = material.diffuse ? material.diffuse->a() : 1.0f;
        if (alpha < 0.999f) {
            ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
            ss->setMode(GL_BLEND, osg::StateAttribute::ON);
        }
        std::cout << "[material] apply node=" << nodeName
                  << " alpha=" << alpha
                  << " ambient=" << (material.ambient ? vec4ToString(*material.ambient) : "none")
                  << " diffuse=" << (material.diffuse ? vec4ToString(*material.diffuse) : "none")
                  << " texture=" << (material.texture_rel_path ? *material.texture_rel_path : "none")
                  << " filtering=" << (material.texture_filtering ? *material.texture_filtering : "none")
                  << " max_anisotropy=" << (material.texture_max_anisotropy ? std::to_string(*material.texture_max_anisotropy) : "none")
                  << " scale=" << (material.texture_scale ? ("(" + std::to_string(material.texture_scale->x()) + "," + std::to_string(material.texture_scale->y()) + ")") : "none")
                  << std::endl;

        osg::Group* group = current->asGroup();
        if (!group) {
            return;
        }
        for (unsigned int i = 0; i < group->getNumChildren(); ++i) {
            applyToNode(group->getChild(i));
        }
    };
    applyToNode(node);
}

bool hasMaterialOverride(const ParsedMaterial& material) {
    return material.ambient.has_value() || material.diffuse.has_value() || material.specular.has_value() ||
           material.emissive.has_value() || material.texture_rel_path.has_value() || material.texture_filtering.has_value() ||
           material.texture_max_anisotropy.has_value() || material.texture_scale.has_value();
}

void appendPathUnique(osgDB::FilePathList& paths, const std::string& path) {
    if (path.empty()) {
        return;
    }
    auto samePath = [&path](const std::string& existing) {
        if (existing.size() != path.size()) {
            return false;
        }
        for (std::size_t i = 0; i < path.size(); ++i) {
            if (std::tolower(static_cast<unsigned char>(existing[i])) !=
                std::tolower(static_cast<unsigned char>(path[i]))) {
                return false;
            }
        }
        return true;
    };
    if (std::find_if(paths.begin(), paths.end(), samePath) == paths.end()) {
        paths.push_back(path);
    }
}

void appendPluginCandidates(osgDB::FilePathList& paths, const std::filesystem::path& root) {
    namespace fs = std::filesystem;
    if (root.empty()) {
        return;
    }
    std::error_code ec;
    if (!fs::exists(root, ec) || !fs::is_directory(root, ec)) {
        return;
    }
    appendPathUnique(paths, root.generic_string());
    for (const auto& entry : fs::directory_iterator(root, ec)) {
        if (ec || !entry.is_directory(ec)) {
            continue;
        }
        const std::string name = entry.path().filename().string();
        if (name.rfind("osgPlugins-", 0) == 0) {
            appendPathUnique(paths, entry.path().generic_string());
        }
    }
}

#if defined(_WIN32)
std::filesystem::path getExecutableDir() {
    std::vector<char> buf(MAX_PATH, '\0');
    DWORD len = 0;
    while (true) {
        len = GetModuleFileNameA(nullptr, buf.data(), static_cast<DWORD>(buf.size()));
        if (len == 0) {
            return {};
        }
        if (len < buf.size() - 1) {
            break;
        }
        buf.resize(buf.size() * 2, '\0');
    }
    return std::filesystem::path(std::string(buf.data(), len)).parent_path();
}
#endif

void decorateCollisionStlGeode(osg::Geode* geode) {
    if (!geode) {
        return;
    }
    osg::ref_ptr<osg::Material> mat = new osg::Material();
    mat->setColorMode(osg::Material::OFF);
    mat->setAmbient(osg::Material::FRONT_AND_BACK, osg::Vec4(0.28f, 0.28f, 0.30f, 1.0f));
    mat->setDiffuse(osg::Material::FRONT_AND_BACK, osg::Vec4(0.62f, 0.64f, 0.68f, 1.0f));
    mat->setSpecular(osg::Material::FRONT_AND_BACK, osg::Vec4(0.35f, 0.35f, 0.38f, 1.0f));
    mat->setShininess(osg::Material::FRONT_AND_BACK, 48.0f);
    osg::ref_ptr<osg::LightModel> lm = new osg::LightModel();
    lm->setTwoSided(true);
    osg::StateSet* ss = geode->getOrCreateStateSet();
    ss->setAttributeAndModes(mat.get(), osg::StateAttribute::ON);
    ss->setAttributeAndModes(lm.get(), osg::StateAttribute::ON);
}

osg::ref_ptr<osg::Node> loadStlMesh(const std::string& full_path) {
    std::ifstream in(full_path, std::ios::binary);
    if (!in) {
        return nullptr;
    }

    in.seekg(0, std::ios::end);
    const std::streamoff file_size = in.tellg();
    in.seekg(0, std::ios::beg);
    if (file_size < 84) {
        return nullptr;
    }

    char header[80] = {0};
    in.read(header, 80);
    std::uint32_t tri_count = 0;
    in.read(reinterpret_cast<char*>(&tri_count), 4);
    const std::streamoff expected = 84 + static_cast<std::streamoff>(tri_count) * 50;
    if (expected == file_size) {
        osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
        vertices->reserve(static_cast<std::size_t>(tri_count) * 3);
        osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array();
        normals->reserve(static_cast<std::size_t>(tri_count) * 3);

        for (std::uint32_t i = 0; i < tri_count; ++i) {
            float n[3];
            float v[9];
            in.read(reinterpret_cast<char*>(n), sizeof(n));
            in.read(reinterpret_cast<char*>(v), sizeof(v));
            std::uint16_t attr = 0;
            in.read(reinterpret_cast<char*>(&attr), sizeof(attr));
            (void)attr;

            const osg::Vec3 normal(n[0], n[1], n[2]);
            for (int k = 0; k < 3; ++k) {
                vertices->push_back(osg::Vec3(v[k * 3 + 0], v[k * 3 + 1], v[k * 3 + 2]));
                normals->push_back(normal);
            }
        }

        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
        geom->setVertexArray(vertices);
        geom->setNormalArray(normals, osg::Array::BIND_PER_VERTEX);
        geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, static_cast<int>(vertices->size())));

        osg::ref_ptr<osg::Geode> geode = new osg::Geode();
        geode->addDrawable(geom);
        decorateCollisionStlGeode(geode.get());
        return geode;
    }
    // ASCII STL fallback
    in.clear();
    in.seekg(0, std::ios::beg);
    std::string line;
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array();
    osg::Vec3 current_normal(0.0f, 0.0f, 1.0f);
    while (std::getline(in, line)) {
        std::stringstream ss(line);
        std::string token;
        ss >> token;
        for (char& c : token) {
            c = static_cast<char>(std::tolower(c));
        }
        if (token == "facet") {
            std::string normal_kw;
            float nx = 0.0f, ny = 0.0f, nz = 1.0f;
            ss >> normal_kw >> nx >> ny >> nz;
            current_normal = osg::Vec3(nx, ny, nz);
        } else if (token == "vertex") {
            float x = 0.0f, y = 0.0f, z = 0.0f;
            ss >> x >> y >> z;
            vertices->push_back(osg::Vec3(x, y, z));
            normals->push_back(current_normal);
        }
    }
    if (vertices->empty()) {
        return nullptr;
    }
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
    geom->setVertexArray(vertices);
    geom->setNormalArray(normals, osg::Array::BIND_PER_VERTEX);
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, static_cast<int>(vertices->size())));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(geom);
    decorateCollisionStlGeode(geode.get());
    return geode;
}

osg::ref_ptr<osg::Node> loadModelOrFallback(float range_m, const SceneModel& model) {
    const std::string full_path = resolveRepoPath(model.mesh_path);
    osg::ref_ptr<osg::Node> loaded;
    if (hasSuffix(full_path, ".stl")) {
        // Prefer built-in STL loader first to avoid plugin dependency and noisy read errors.
        loaded = loadStlMesh(full_path);
    }
    if (!loaded.valid()) {
        loaded = osgDB::readRefNodeFile(full_path);
    }
    if (!loaded.valid() && hasSuffix(full_path, ".osgb")) {
        // If OSG binary plugins are unavailable on this Windows build, fallback to collision STL if present.
        const std::size_t slash = model.mesh_path.find_last_of('/');
        if (slash != std::string::npos) {
            const std::string collision_rel = model.mesh_path.substr(0, slash + 1) + "collision.stl";
            const std::string collision_full = resolveRepoPath(collision_rel);
            loaded = loadStlMesh(collision_full);
            if (loaded.valid()) {
                std::cout << "[scene] load_real " << collision_rel << " (osgb->stl fallback)" << std::endl;
                return loaded;
            }
        }
    }
    if (loaded.valid()) {
        if (hasMaterialOverride(model.material)) {
            applyParsedMaterialRecursive(loaded.get(), model.material);
        } else {
            std::cout << "[material] keep original mesh material (no sdf/script override) mesh=" << model.mesh_path << std::endl;
        }
        std::cout << "[material] applied mesh=" << model.mesh_path
                  << " diffuse=" << (model.material.diffuse ? vec4ToString(*model.material.diffuse) : "none")
                  << " ambient=" << (model.material.ambient ? vec4ToString(*model.material.ambient) : "none")
                  << " specular=" << (model.material.specular ? vec4ToString(*model.material.specular) : "none")
                  << " emissive=" << (model.material.emissive ? vec4ToString(*model.material.emissive) : "none")
                  << " texture=" << (model.material.texture_rel_path ? *model.material.texture_rel_path : "none")
                  << " filtering=" << (model.material.texture_filtering ? *model.material.texture_filtering : "none")
                  << " max_anisotropy=" << (model.material.texture_max_anisotropy ? std::to_string(*model.material.texture_max_anisotropy) : "none")
                  << " scale=" << (model.material.texture_scale ? ("(" + std::to_string(model.material.texture_scale->x()) + "," + std::to_string(model.material.texture_scale->y()) + ")") : "none")
                  << std::endl;
        std::cout << "[scene] load_real " << model.mesh_path << std::endl;
        return loaded;
    }
    std::cout << "[scene] load_fallback " << model.mesh_path << std::endl;
    return createFallbackNode(range_m, model);
}

const SceneSpec& selectScene(const std::string& world_name) {
    static const SceneSpec kSsivBahia{
        "ssiv_bahia",
        {
            // From uwmodels/scenes/ssiv_bahia/ssiv_bahia.world
            {std::string("uwmodels/sdf/seafloor/collision.stl"), false, std::nullopt, std::nullopt, std::nullopt, std::nullopt, ParsedMaterial{}, {0.0, 0.0, -12.3, 0.0, 0.0, 0.0}, osg::Vec4(0.15f, 0.2f, 0.2f, 1.0f), true},
            {std::string("uwmodels/sdf/ssiv_mockup_bahia/visual.osgb"), false, std::nullopt, std::nullopt, std::nullopt, std::nullopt, ParsedMaterial{}, {47.36, 113.10, -12.3, 0.0, 0.0, 4.29}, osg::Vec4(0.35f, 0.45f, 0.55f, 1.0f), false},
        }};

    static const SceneSpec kTank{
        "tank",
        {
            // From uwmodels/scenes/tank/tank.world
            {std::string("uwmodels/sdf/tank/visual.osgb"), false, std::nullopt, std::nullopt, std::nullopt, std::nullopt, ParsedMaterial{}, {0.0, 0.0, 1.0, 0.0, 0.0, 0.0}, osg::Vec4(0.4f, 0.5f, 0.6f, 1.0f), false},
        }};

    if (worldSpecToSceneKey(world_name) == "tank") {
        return kTank;
    }
    return kSsivBahia;
}

SceneSpec loadSceneFromWorld(const std::string& world_name) {
    SceneSpec spec;
    spec.world_name = "dynamic";
    auto includes_opt = parseWorldIncludes(world_name);
    if (!includes_opt) {
        return spec;
    }
    const auto& includes = *includes_opt;
    for (std::size_t i = 0; i < includes.size(); ++i) {
        const auto& inc = includes[i];
        ModelGeometrySpec geometry = parseModelGeometrySpec(inc.model_name);
        SceneModel model;
        model.mesh_path = "";
        model.skip_render = false;
        model.primitive_box_size = std::nullopt;
        model.primitive_cylinder_size = std::nullopt;
        model.primitive_sphere_radius = std::nullopt;
        model.primitive_plane_size = std::nullopt;
        model.material = ParsedMaterial{};
        model.pose_xyzrpy = inc.pose_xyzrpy;
        model.fallback_color = osg::Vec4(0.3f + 0.1f * static_cast<float>(i % 3), 0.45f, 0.55f, 1.0f);
        model.fallback_is_plane = (inc.model_name == "seafloor");
        if (geometry.is_light_only) {
            model.skip_render = true;
            spec.models.push_back(model);
            continue;
        }
        if (geometry.geometries.empty()) {
            spec.models.push_back(model);
            continue;
        }
        for (const auto& g : geometry.geometries) {
            SceneModel item = model;
            item.pose_xyzrpy = {
                inc.pose_xyzrpy[0] + geometry.local_pose_xyzrpy[0] + g.local_pose_xyzrpy[0],
                inc.pose_xyzrpy[1] + geometry.local_pose_xyzrpy[1] + g.local_pose_xyzrpy[1],
                inc.pose_xyzrpy[2] + geometry.local_pose_xyzrpy[2] + g.local_pose_xyzrpy[2],
                inc.pose_xyzrpy[3] + geometry.local_pose_xyzrpy[3] + g.local_pose_xyzrpy[3],
                inc.pose_xyzrpy[4] + geometry.local_pose_xyzrpy[4] + g.local_pose_xyzrpy[4],
                inc.pose_xyzrpy[5] + geometry.local_pose_xyzrpy[5] + g.local_pose_xyzrpy[5]
            };
            item.fallback_color = g.color;
            item.mesh_path = g.mesh_path;
            item.primitive_box_size = g.primitive_box_size;
            item.primitive_cylinder_size = g.primitive_cylinder_size;
            item.primitive_sphere_radius = g.primitive_sphere_radius;
            item.primitive_plane_size = g.primitive_plane_size;
            item.material = g.material;
            item.visible = g.visible;
            spec.models.push_back(item);
        }
    }
    return spec;
}

void appendSceneModelsToGroup(osg::Group* parent, float range_m, const SceneSpec& scene) {
    if (!parent) {
        return;
    }
    for (const SceneModel& model : scene.models) {
        if (model.skip_render) {
            std::cout << "[scene] skip_render light-only model" << std::endl;
            continue;
        }
        if (model.mesh_path.empty()) {
            std::cout << "[scene] load_fallback <missing-mesh-uri>" << std::endl;
            osg::ref_ptr<osg::Node> node = createFallbackNode(range_m, model);
            osg::ref_ptr<osg::MatrixTransform> xform = new osg::MatrixTransform();
            osg::Matrixd matrix =
                osg::Matrixd::translate(model.pose_xyzrpy[0], model.pose_xyzrpy[1], model.pose_xyzrpy[2]) *
                osg::Matrixd::rotate(model.pose_xyzrpy[5], osg::Vec3d(0.0, 0.0, 1.0)) *
                osg::Matrixd::rotate(model.pose_xyzrpy[4], osg::Vec3d(0.0, 1.0, 0.0)) *
                osg::Matrixd::rotate(model.pose_xyzrpy[3], osg::Vec3d(1.0, 0.0, 0.0));
            xform->setMatrix(matrix);
            xform->addChild(node);
            parent->addChild(xform);
            continue;
        }
        osg::ref_ptr<osg::Node> node = loadModelOrFallback(range_m, model);
        osg::ref_ptr<osg::MatrixTransform> xform = new osg::MatrixTransform();
        osg::Matrixd matrix =
            osg::Matrixd::translate(model.pose_xyzrpy[0], model.pose_xyzrpy[1], model.pose_xyzrpy[2]) *
            osg::Matrixd::rotate(model.pose_xyzrpy[5], osg::Vec3d(0.0, 0.0, 1.0)) *
            osg::Matrixd::rotate(model.pose_xyzrpy[4], osg::Vec3d(0.0, 1.0, 0.0)) *
            osg::Matrixd::rotate(model.pose_xyzrpy[3], osg::Vec3d(1.0, 0.0, 0.0));
        xform->setMatrix(matrix);
        xform->addChild(node);
        if (!model.visible) {
            xform->setNodeMask(0u);
            std::cout << "[scene] hidden collision geometry for model mesh=" << model.mesh_path << std::endl;
        }
        parent->addChild(xform);
    }
}

/// Fills `world_models` (cleared by caller when rebuilding) using: unreadable world → fallback map; 0 includes → empty; else → file.
void populateWorldModelsGroup(osg::Group* world_models, float range_m, const std::string& world_spec) {
    if (!world_models) {
        return;
    }
    const auto includes_opt = parseWorldIncludes(world_spec);
    if (!includes_opt) {
        const SceneSpec& fb = selectScene(worldSpecToSceneKey(world_spec));
        std::cout << "[scene] world=" << resolveWorldFilePath(world_spec) << " (fallback: world file unreadable) -> " << fb.world_name
                  << std::endl;
        appendSceneModelsToGroup(world_models, range_m, fb);
        return;
    }
    if (includes_opt->empty()) {
        std::cout << "[scene] world=" << resolveWorldFilePath(world_spec) << " (0 includes — empty preview)" << std::endl;
        SceneSpec empty{};
        appendSceneModelsToGroup(world_models, range_m, empty);
        return;
    }
    SceneSpec dynamic_scene = loadSceneFromWorld(world_spec);
    std::cout << "[scene] world=" << resolveWorldFilePath(world_spec) << " (from world file)" << std::endl;
    appendSceneModelsToGroup(world_models, range_m, dynamic_scene);
}

void rebuildWorldModelsFromWorldSpecImpl(osg::Group* world_models, float range_m, const std::string& world_spec) {
    if (!world_models) {
        return;
    }
    world_models->removeChildren(0, world_models->getNumChildren());
    populateWorldModelsGroup(world_models, range_m, world_spec);
}

} // namespace

const char* kWorldModelsGroupName = "StandaloneWorldModels";

void rebuildWorldModelsFromWorldSpec(osg::Group* world_models, float range_m, const std::string& world_spec) {
    rebuildWorldModelsFromWorldSpecImpl(world_models, range_m, world_spec);
}

osg::Group* findWorldModelsGroup(osg::Group* scene_root) {
    if (!scene_root) {
        return nullptr;
    }
    for (unsigned i = 0; i < scene_root->getNumChildren(); ++i) {
        osg::Node* c = scene_root->getChild(i);
        osg::Group* g = c ? c->asGroup() : nullptr;
        if (g && g->getName() == kWorldModelsGroupName) {
            return g;
        }
    }
    return nullptr;
}

void configureOsgDataPath() {
    auto* registry = osgDB::Registry::instance();
#if defined(STANDALONE_SIMULATION_DIR)
    std::string base_dir = STANDALONE_SIMULATION_DIR;
    {
        std::error_code ec;
        const std::filesystem::path sim_dir(STANDALONE_SIMULATION_DIR);
        const bool have_uwmodels = std::filesystem::is_directory(sim_dir / "uwmodels", ec);
        const bool have_src_resources = std::filesystem::is_directory(sim_dir / "src" / "sonar_imaging" / "resources", ec);
        if (!have_uwmodels && !have_src_resources) {
            base_dir.clear();
#if defined(_WIN32)
            const std::filesystem::path exe_dir = getExecutableDir();
            if (!exe_dir.empty()) {
                base_dir = exe_dir.generic_string();
            }
#endif
        }
    }
    if (!base_dir.empty()) {
        appendPathUnique(registry->getDataFilePathList(), base_dir);
        // If the project was installed into ./install/share, add it too.
        std::error_code ec;
        const std::filesystem::path base_path(base_dir);
        const std::filesystem::path install_share = base_path / ".." / "install" / "share";
        const std::filesystem::path canon = std::filesystem::weakly_canonical(install_share, ec);
        if (!ec && std::filesystem::is_directory(canon)) {
            appendPathUnique(registry->getDataFilePathList(), canon.generic_string());
            std::cout << "[scene] OSG data path += " << canon.generic_string() << " (install share)" << std::endl;
        }
    }
#endif
    auto& lib_paths = registry->getLibraryFilePathList();
#if defined(STANDALONE_OSG_PLUGIN_ROOT)
    appendPluginCandidates(lib_paths, std::filesystem::path(STANDALONE_OSG_PLUGIN_ROOT));
#endif
#if defined(_WIN32)
    const std::filesystem::path exe_dir = getExecutableDir();
    appendPluginCandidates(lib_paths, exe_dir);
    appendPluginCandidates(lib_paths, exe_dir / "bin");
    appendPluginCandidates(lib_paths, exe_dir.parent_path() / "bin");
#endif
    std::error_code ec;
    appendPluginCandidates(lib_paths, std::filesystem::current_path(ec));
}

osg::ref_ptr<osg::Group> createSharedSceneGraph(float range_m, const std::string& world_name) {
    osg::ref_ptr<osg::Group> root = new osg::Group();
    osg::ref_ptr<osg::Group> world_models = new osg::Group();
    world_models->setName(kWorldModelsGroupName);
    populateWorldModelsGroup(world_models.get(), range_m, world_name);
    root->addChild(world_models.get());

    // Ensure fallback geometries are lit and visible.
    osg::ref_ptr<osg::Light> light = new osg::Light();
    light->setLightNum(0);
    light->setPosition(osg::Vec4(0.0f, 0.0f, 200.0f, 1.0f));
    light->setAmbient(osg::Vec4(0.35f, 0.35f, 0.35f, 1.0f));
    light->setDiffuse(osg::Vec4(0.9f, 0.9f, 0.9f, 1.0f));
    osg::ref_ptr<osg::LightSource> light_source = new osg::LightSource();
    light_source->setLight(light);
    root->addChild(light_source);
    std::cout << "[scene] root_children=" << root->getNumChildren() << std::endl;

    return root;
}

bool computeInitialPoseNearCollision(const std::string& world_name,
                                     float range_m,
                                     osg::Vec3d& position,
                                     double& yaw,
                                     double& pitch) {
    auto includes_opt = parseWorldIncludes(world_name);
    if (!includes_opt || includes_opt->empty()) {
        return false;
    }

    const auto& includes = *includes_opt;
    int best_idx = -1;
    int best_score = std::numeric_limits<int>::min();
    for (std::size_t i = 0; i < includes.size(); ++i) {
        const auto& inc = includes[i];
        int score = 0;
        const bool is_seafloor = (inc.model_name == "seafloor");
        const bool has_collision = hasCollisionMesh(inc.model_name);
        if (!is_seafloor) {
            score += 10;
        }
        if (has_collision) {
            score += 5;
        }
        if (score > best_score) {
            best_score = score;
            best_idx = static_cast<int>(i);
        }
    }
    if (best_idx < 0) {
        return false;
    }

    const auto& target = includes[static_cast<std::size_t>(best_idx)];
    const double mx = target.pose_xyzrpy[0];
    const double my = target.pose_xyzrpy[1];
    const double mz = target.pose_xyzrpy[2];

    const double dxy = std::clamp(static_cast<double>(range_m) * 0.35, 3.0, 8.0);
    const double dz = std::clamp(static_cast<double>(range_m) * 0.15, 1.5, 4.0);
    const osg::Vec3d eye(mx - dxy, my - 0.6 * dxy, mz + dz);
    const osg::Vec3d look(mx, my, mz);
    const osg::Vec3d dir = look - eye;
    const double planar = std::sqrt(dir.x() * dir.x() + dir.y() * dir.y());

    position = eye;
    yaw = std::atan2(dir.y(), dir.x());
    pitch = std::atan2(dir.z(), std::max(1.0e-9, planar));
    return true;
}

void applySonarFrustumToCamera(osg::Camera* camera,
                               int viewport_width,
                               int viewport_height,
                               double beam_width_deg,
                               double beam_height_deg) {
    if (!camera || viewport_width <= 0 || viewport_height <= 0) {
        return;
    }
    const double znear = 1.0;
    const double zfar = 1.0e6;
    const double half_w = znear * std::tan(0.5 * osg::DegreesToRadians(beam_width_deg));
    const double half_h = znear * std::tan(0.5 * osg::DegreesToRadians(beam_height_deg));
    // Window aspect is chosen via viewerHeightForSonarAspect so pixels are not stretched vs beam angles.
    camera->setProjectionMatrixAsFrustum(-half_w, half_w, -half_h, half_h, znear, zfar);
}

int viewerHeightForSonarAspect(int window_width, double beam_width_deg, double beam_height_deg) {
    if (window_width <= 0) {
        return 840;
    }
    const double t_hw = std::tan(0.5 * osg::DegreesToRadians(beam_width_deg));
    const double t_vw = std::tan(0.5 * osg::DegreesToRadians(beam_height_deg));
    if (t_hw < 1e-12) {
        return std::max(200, window_width);
    }
    const int h = static_cast<int>(std::lround(static_cast<double>(window_width) * (t_vw / t_hw)));
    return std::max(200, h);
}

namespace {

struct SonarFovResizeHandler final : public osgGA::GUIEventHandler {
    osg::observer_ptr<osg::Camera> camera_;
    double beam_width_deg_;
    double beam_height_deg_;

public:
    SonarFovResizeHandler(osg::Camera* cam, double bw, double bh)
        : camera_(cam), beam_width_deg_(bw), beam_height_deg_(bh) {}

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override {
        if (ea.getEventType() != osgGA::GUIEventAdapter::RESIZE) {
            return false;
        }
        osg::Camera* cam = camera_.get();
        if (!cam) {
            return false;
        }
        const int w = ea.getWindowWidth();
        const int h = ea.getWindowHeight();
        if (w <= 0 || h <= 0) {
            return false;
        }
        applySonarFrustumToCamera(cam, w, h, beam_width_deg_, beam_height_deg_);
        return false;
    }
};

} // namespace

void installSonarFovResizeHandler(osgViewer::Viewer& viewer, double beam_width_deg, double beam_height_deg) {
    osg::Camera* cam = viewer.getCamera();
    if (!cam) {
        return;
    }
    cam->setProjectionResizePolicy(osg::Camera::FIXED);
    const osg::Viewport* vp = cam->getViewport();
    int w = vp ? static_cast<int>(vp->width()) : 0;
    int h = vp ? static_cast<int>(vp->height()) : 0;
    if (w <= 0 || h <= 0) {
        w = 1360;
        h = 840;
    }
    applySonarFrustumToCamera(cam, w, h, beam_width_deg, beam_height_deg);
    viewer.addEventHandler(new SonarFovResizeHandler(cam, beam_width_deg, beam_height_deg));
    std::cout << "[scene] viewer projection = sonar FOV " << beam_width_deg << " x " << beam_height_deg << " deg"
              << std::endl;
}

} // namespace standalone_mvp
