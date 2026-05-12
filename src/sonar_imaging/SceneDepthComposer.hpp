#ifndef SONAR_IMAGING_SCENE_DEPTH_COMPOSER_HPP_
#define SONAR_IMAGING_SCENE_DEPTH_COMPOSER_HPP_

#include "GeometryTextureTools.hpp"

// C++ includes
#include <vector>

// OSG includes
#include <osg/Node>
#include <osg/Group>
#include <osg/ref_ptr>

namespace sonar_imaging {

/**
 * Gets the informations of normal and distance from a osg scene, between the objects and the camera.
 */
class SceneDepthComposer {
public:
    /**
     * Build a map from the normal surface and the distance from objects to the viewer camera by applying shaders.
     * BLUE CHANNEL, presents the normal values from the objects to the center camera, where:
     *      1 is the max value, and represents the normal vector of the object surface and the normal vector of camera are in the same directions;
     *      0 is the minimum value, the normal vector of the object surface and the normal vector of camera are in the perpendicular directions.
     * GREEN CHANNEL presents the distance values relative from camera center, where:
     *      0 is the minimum value, and represents the object is near from the camera;
     *      1 is the max value, and represents the object is far from the camera, and it is limited by max range.
    */
    SceneDepthComposer();
    SceneDepthComposer(float maxRange);
    SceneDepthComposer(float maxRange, float attenuationCoeff);

    /**
     * Add the models in the main normal depth map node
     * @param node: osg node to add a main scene
     */
    void attachSceneNode(osg::ref_ptr<osg::Node> node);

    osg::ref_ptr<osg::Group> depthComposerNode() const;

    // ---- Shader output toggles ----
    bool isDrawDepth() const;
    bool isDrawNormal() const;
    bool isDrawReverb() const;
    void setDrawDepth(bool value);
    void setDrawNormal(bool value);
    void setDrawReverb(bool value);

    // ---- Range and attenuation controls ----
    float getAttenuationCoefficient() const;
    float getMaxRange() const;
    void setAttenuationCoefficient(float value);
    void setMaxRange(float value);

  private:
    // Root node owning shader state and uniforms for the sonar depth composer.
    osg::ref_ptr<osg::Group> _depth_composer_root_node_;
    // Child group where the externally managed scene graph is attached/replaced.
    osg::ref_ptr<osg::Group> _sonar_state_host_node_;

    /**
     * Setup the main scene node.
     *
     * @param maxRange: limits the maximum observable distance by viewer camera.
     * @param attenuationCoeff: the underwater signal attenuation value.
     * @param drawDepth: enables the distance calculation on shader.
     * @param drawNormal: enables the normal calculation on shader.
     * @param drawReverb: enables the reverberation effect on shader.
     */
    osg::ref_ptr<osg::Group> createComposerRootWithShaderState(
        float maxRange = 50.0,
        float attenuationCoeff = 0,
        bool drawDepth = true,
        bool drawNormal = true,
        bool drawReverb = true);
};
}

#endif
