#pragma once

#include <Eigen/Geometry>

#include <osg/Group>
#include <osg/LightSource>
#include <osg/ref_ptr>

#include <QImage>

#include <vector>

class CameraVisualEffects {
public:
    void attachToScene(osg::Group* root);
    void seedParticleVolume(const Eigen::Vector3d& center, double radius_m);

    void setFogAmount(float normalized);
    void setLightAmount(float normalized);
    void setParticleAmount(float normalized);

    float fogAmount() const { return fog_amount_; }
    float lightAmount() const { return light_amount_; }
    float particleAmount() const { return particle_amount_; }

    void updateParticles(const Eigen::Vector3d& camera_position, double delta_s);

    QImage postProcessImage(const QImage& source) const;

private:
    enum class ParticleKind { Bubble, MarineSnow, Debris };

    struct ParticleState {
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        float phase = 0.0f;
        float alpha = 0.0f;
        ParticleKind kind = ParticleKind::Bubble;
    };

    void ensureParticlePool();
    void applyLight();
    void applyFog();
    void syncParticleGeometry(const Eigen::Vector3d& camera_position);
    void wrapParticleIntoVolume(ParticleState& particle) const;
    static float particleSizePx(ParticleKind kind);

    osg::ref_ptr<osg::Group> scene_root_;
    osg::ref_ptr<osg::LightSource> light_source_;
    osg::ref_ptr<osg::Group> particles_root_;
    osg::ref_ptr<osg::Geode> particles_geode_;
    osg::ref_ptr<osg::Geometry> particles_geometry_;

    std::vector<ParticleState> particles_;

    float fog_amount_ = 0.0f;
    float light_amount_ = 0.5f;
    float particle_amount_ = 0.25f;

    osg::Vec4 base_ambient_{0.35f, 0.35f, 0.35f, 1.0f};
    osg::Vec4 base_diffuse_{0.9f, 0.9f, 0.9f, 1.0f};

    Eigen::Vector3d volume_center_ = Eigen::Vector3d::Zero();
    double volume_radius_m_ = 80.0;
    double volume_height_m_ = 35.0;
    double simulation_time_s_ = 0.0;
    bool particles_seeded_ = false;

    static constexpr int kMaxParticles = 50000;
};
