#include "CameraVisualEffects.hpp"

#include <osg/Array>
#include <osg/BlendFunc>
#include <osg/Fog>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Point>
#include <osg/StateSet>

#include <opencv2/imgproc.hpp>

#include <QtGlobal>

#include <algorithm>
#include <cmath>
#include <random>

namespace {

QImage mirroredCopy(const QImage& source) {
    QImage copy = source.copy();
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    return copy.flipped(Qt::Horizontal);
#else
    return copy.mirrored();
#endif
}

float hashSeed(int index, float salt) {
    return std::fmod(std::sin(static_cast<float>(index) * 12.9898f + salt * 78.233f) * 43758.5453f, 1.0f);
}

} // namespace

void CameraVisualEffects::attachToScene(osg::Group* root) {
    if (!root) {
        return;
    }

    for (unsigned i = 0; i < root->getNumChildren(); ++i) {
        if (auto* light_source = dynamic_cast<osg::LightSource*>(root->getChild(i))) {
            light_source_ = light_source;
            if (light_source_->getLight()) {
                base_ambient_ = light_source_->getLight()->getAmbient();
                base_diffuse_ = light_source_->getLight()->getDiffuse();
            }
            break;
        }
    }

    particles_root_ = new osg::Group();
    particles_root_->setName("ocean_particles");
    particles_geode_ = new osg::Geode();
    particles_geometry_ = new osg::Geometry();
    particles_geometry_->setUseDisplayList(false);
    particles_geometry_->setUseVertexBufferObjects(true);
    particles_geode_->addDrawable(particles_geometry_.get());

    osg::StateSet* state = particles_geode_->getOrCreateStateSet();
    state->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    state->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    state->setMode(GL_BLEND, osg::StateAttribute::ON);
    state->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
    state->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
                                osg::StateAttribute::ON);
    state->setAttribute(new osg::Point(particleSizePx(ParticleKind::MarineSnow)), osg::StateAttribute::ON);

    particles_root_->addChild(particles_geode_.get());
    root->addChild(particles_root_.get());
    scene_root_ = root;

    particles_.resize(static_cast<std::size_t>(kMaxParticles));
    ensureParticlePool();
    applyLight();
    applyFog();
}

void CameraVisualEffects::seedParticleVolume(const Eigen::Vector3d& center, double radius_m) {
    volume_center_ = center;
    volume_radius_m_ = std::clamp(radius_m, 20.0, 200.0);
    volume_height_m_ = std::clamp(radius_m * 0.45, 12.0, 60.0);

    std::mt19937 rng(1337);
    std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
    std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * M_PI);

    for (int i = 0; i < kMaxParticles; ++i) {
        ParticleState& particle = particles_[static_cast<std::size_t>(i)];
        const double u = unit_dist(rng);
        const double v = unit_dist(rng);
        const double w = unit_dist(rng);
        const double radius_sample = volume_radius_m_ * std::cbrt(u);
        const double angle = angle_dist(rng);

        const double x = volume_center_.x() + radius_sample * std::cos(angle);
        const double y = volume_center_.y() + radius_sample * std::sin(angle);
        const double z = volume_center_.z() + (v - 0.5) * volume_height_m_;
        particle.position = Eigen::Vector3d(x, y, z);
        particle.phase = hashSeed(i, 0.17f) * 6.283185f;

        const double kind_roll = w;
        if (kind_roll < 0.45) {
            particle.kind = ParticleKind::Bubble;
            particle.velocity =
                Eigen::Vector3d((unit_dist(rng) - 0.5) * 0.04, (unit_dist(rng) - 0.5) * 0.04, 0.06 + unit_dist(rng) * 0.08);
            particle.alpha = 0.12f + static_cast<float>(unit_dist(rng)) * 0.18f;
        } else if (kind_roll < 0.82) {
            particle.kind = ParticleKind::MarineSnow;
            const double drift_angle = angle_dist(rng);
            const double drift_speed = 0.015 + unit_dist(rng) * 0.04;
            particle.velocity = Eigen::Vector3d(std::cos(drift_angle) * drift_speed,
                                                std::sin(drift_angle) * drift_speed,
                                                -0.004 - unit_dist(rng) * 0.012);
            particle.alpha = 0.08f + static_cast<float>(unit_dist(rng)) * 0.14f;
        } else {
            particle.kind = ParticleKind::Debris;
            particle.velocity =
                Eigen::Vector3d((unit_dist(rng) - 0.5) * 0.03, (unit_dist(rng) - 0.5) * 0.03, -0.008 - unit_dist(rng) * 0.02);
            particle.alpha = 0.10f + static_cast<float>(unit_dist(rng)) * 0.16f;
        }
    }

    particles_seeded_ = true;
    syncParticleGeometry(volume_center_);
}

void CameraVisualEffects::setFogAmount(float normalized) {
    fog_amount_ = std::clamp(normalized, 0.0f, 1.0f);
    applyFog();
}

void CameraVisualEffects::setLightAmount(float normalized) {
    light_amount_ = std::clamp(normalized, 0.0f, 1.0f);
    applyLight();
}

void CameraVisualEffects::setParticleAmount(float normalized) {
    particle_amount_ = std::clamp(normalized, 0.0f, 1.0f);
}

void CameraVisualEffects::applyLight() {
    if (!light_source_ || !light_source_->getLight()) {
        return;
    }

    const float scale = 0.25f + light_amount_ * 1.75f;
    osg::Light* light = light_source_->getLight();
    light->setAmbient(base_ambient_ * scale);
    light->setDiffuse(base_diffuse_ * scale);
}

void CameraVisualEffects::applyFog() {
    if (!scene_root_.valid()) {
        return;
    }

    osg::StateSet* state = scene_root_->getOrCreateStateSet();
    if (fog_amount_ <= 0.001f) {
        state->setMode(GL_FOG, osg::StateAttribute::OFF);
        return;
    }

    osg::ref_ptr<osg::Fog> fog = new osg::Fog();
    fog->setMode(osg::Fog::LINEAR);
    fog->setColor(osg::Vec4(0.03f, 0.05f, 0.08f, 1.0f));
    fog->setDensity(0.02f + fog_amount_ * 0.12f);
    fog->setStart(1.0f);
    fog->setEnd(8.0f + (1.0f - fog_amount_) * 92.0f);
    state->setAttributeAndModes(fog.get(), osg::StateAttribute::ON);
}

float CameraVisualEffects::particleSizePx(ParticleKind kind) {
    switch (kind) {
    case ParticleKind::Bubble:
        return 4.5f;
    case ParticleKind::MarineSnow:
        return 2.5f;
    case ParticleKind::Debris:
        return 3.5f;
    }
    return 3.0f;
}

void CameraVisualEffects::ensureParticlePool() {
    if (!particles_geometry_) {
        return;
    }

    auto* vertices = new osg::Vec3Array(static_cast<unsigned int>(kMaxParticles));
    auto* colors = new osg::Vec4Array(static_cast<unsigned int>(kMaxParticles));
    for (int i = 0; i < kMaxParticles; ++i) {
        (*vertices)[static_cast<unsigned int>(i)] = osg::Vec3(0.0f, 0.0f, 0.0f);
        (*colors)[static_cast<unsigned int>(i)] = osg::Vec4(1.0f, 1.0f, 1.0f, 0.0f);
    }

    particles_geometry_->setVertexArray(vertices);
    particles_geometry_->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
    particles_geometry_->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, static_cast<int>(kMaxParticles)));
}

void CameraVisualEffects::wrapParticleIntoVolume(ParticleState& particle) const {
    auto wrap_axis = [](double value, double center, double half_extent) {
        double local = value - center;
        while (local > half_extent) {
            local -= 2.0 * half_extent;
        }
        while (local < -half_extent) {
            local += 2.0 * half_extent;
        }
        return center + local;
    };

    const Eigen::Vector2d offset(particle.position.x() - volume_center_.x(),
                                 particle.position.y() - volume_center_.y());
    const double horizontal_dist = offset.norm();
    if (horizontal_dist > volume_radius_m_) {
        const double scale = (volume_radius_m_ * 0.92) / std::max(1e-6, horizontal_dist);
        particle.position.x() = volume_center_.x() + offset.x() * scale;
        particle.position.y() = volume_center_.y() + offset.y() * scale;
    }

    particle.position.z() =
        wrap_axis(particle.position.z(), volume_center_.z(), volume_height_m_ * 0.5);
}

void CameraVisualEffects::syncParticleGeometry(const Eigen::Vector3d& camera_position) {
    if (!particles_geometry_) {
        return;
    }

    auto* vertices = dynamic_cast<osg::Vec3Array*>(particles_geometry_->getVertexArray());
    auto* colors = dynamic_cast<osg::Vec4Array*>(particles_geometry_->getColorArray());
    if (!vertices || !colors) {
        return;
    }

    const int active_count =
        static_cast<int>(std::lround(particle_amount_ * static_cast<float>(kMaxParticles)));
    particles_root_->setNodeMask(active_count > 0 ? ~0u : 0u);
    if (active_count <= 0) {
        return;
    }

    constexpr double kVisibleDistanceM = 70.0;
    constexpr double kFadeDistanceM = 45.0;

    for (int i = 0; i < kMaxParticles; ++i) {
        const unsigned idx = static_cast<unsigned>(i);
        if (i >= active_count || !particles_seeded_) {
            (*colors)[idx] = osg::Vec4(1.0f, 1.0f, 1.0f, 0.0f);
            continue;
        }

        const ParticleState& particle = particles_[static_cast<std::size_t>(i)];
        (*vertices)[idx] = osg::Vec3(static_cast<float>(particle.position.x()),
                                       static_cast<float>(particle.position.y()),
                                       static_cast<float>(particle.position.z()));

        const double distance = (particle.position - camera_position).norm();
        double visibility = 1.0;
        if (distance > kFadeDistanceM) {
            visibility = 1.0 - (distance - kFadeDistanceM) / std::max(1.0, kVisibleDistanceM - kFadeDistanceM);
        }
        visibility = std::clamp(visibility, 0.0, 1.0);

        float r = 0.82f;
        float g = 0.90f;
        float b = 0.98f;
        switch (particle.kind) {
        case ParticleKind::Bubble:
            r = 0.78f;
            g = 0.92f;
            b = 1.0f;
            break;
        case ParticleKind::MarineSnow:
            r = 0.88f;
            g = 0.90f;
            b = 0.84f;
            break;
        case ParticleKind::Debris:
            r = 0.72f;
            g = 0.78f;
            b = 0.80f;
            break;
        }

        const float alpha =
            static_cast<float>(visibility) * particle.alpha * (0.35f + particle_amount_ * 0.85f);
        (*colors)[idx] = osg::Vec4(r, g, b, alpha);
    }

    vertices->dirty();
    colors->dirty();
    particles_geometry_->dirtyBound();
}

void CameraVisualEffects::updateParticles(const Eigen::Vector3d& camera_position, double delta_s) {
    if (!particles_geometry_ || !particles_root_.valid()) {
        return;
    }

    if (!particles_seeded_) {
        seedParticleVolume(camera_position, volume_radius_m_);
    }

    const double dt = std::clamp(delta_s, 0.0, 0.1);
    simulation_time_s_ += dt;

    const int active_count =
        static_cast<int>(std::lround(particle_amount_ * static_cast<float>(kMaxParticles)));
    if (active_count <= 0) {
        particles_root_->setNodeMask(0u);
        return;
    }

    for (int i = 0; i < active_count; ++i) {
        ParticleState& particle = particles_[static_cast<std::size_t>(i)];
        particle.position += particle.velocity * dt;

        const float bob = std::sin(particle.phase + static_cast<float>(simulation_time_s_) * 0.7f);
        switch (particle.kind) {
        case ParticleKind::Bubble:
            particle.position.z() += static_cast<double>(bob) * 0.004;
            break;
        case ParticleKind::MarineSnow:
            particle.position.x() += static_cast<double>(bob) * 0.0015;
            particle.position.y() += static_cast<double>(std::cos(particle.phase)) * 0.0015;
            break;
        case ParticleKind::Debris:
            particle.position.z() += static_cast<double>(bob) * 0.0008;
            break;
        }

        wrapParticleIntoVolume(particle);

        const double camera_distance = (particle.position - camera_position).norm();
        if (camera_distance > volume_radius_m_ * 0.95) {
            const float seed = hashSeed(i, static_cast<float>(simulation_time_s_) * 0.31f);
            const double angle = static_cast<double>(seed) * 6.28318530718;
            const double radial = volume_radius_m_ * (0.35 + 0.55 * std::fmod(seed * 17.0f, 1.0f));
            const double height = (hashSeed(i, 0.73f) - 0.5) * volume_height_m_;
            particle.position = camera_position +
                                Eigen::Vector3d(std::cos(angle) * radial, std::sin(angle) * radial, height);
            wrapParticleIntoVolume(particle);
        }
    }

    syncParticleGeometry(camera_position);
}

QImage CameraVisualEffects::postProcessImage(const QImage& source) const {
    if (source.isNull()) {
        return {};
    }

    QImage processed = mirroredCopy(source);
    if (fog_amount_ <= 0.001f) {
        return processed;
    }

    cv::Mat rgba(processed.height(), processed.width(), CV_8UC4,
                 const_cast<uchar*>(processed.constBits()), static_cast<size_t>(processed.bytesPerLine()));
    cv::Mat bgr;
    cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);

    const int kernel = std::max(1, static_cast<int>(std::lround(fog_amount_ * 15.0f)) * 2 + 1);
    cv::GaussianBlur(bgr, bgr, cv::Size(kernel, kernel), 0.0);

    cv::Mat out_rgba;
    cv::cvtColor(bgr, out_rgba, cv::COLOR_BGR2RGBA);
    return QImage(out_rgba.data, out_rgba.cols, out_rgba.rows, static_cast<int>(out_rgba.step),
                  QImage::Format_RGBA8888)
        .copy();
}
