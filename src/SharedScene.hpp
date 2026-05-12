#pragma once

#include <osg/Group>
#include <osg/Vec3d>
#include <string>

namespace osg {
class Camera;
}
namespace osgViewer {
class Viewer;
}

namespace standalone_mvp {

// ---- Scene graph setup/reload ----
void configureOsgDataPath();
osg::ref_ptr<osg::Group> createSharedSceneGraph(float range_m, const std::string& world_name = "ssiv_bahia");

/// OSG group name used by \ref createSharedSceneGraph for all world geometry (reloadable).
extern const char* kWorldModelsGroupName;

/// Replaces all children of `world_models` by reparsing the world file / fallback scene (same rules as initial load).
void rebuildWorldModelsFromWorldSpec(osg::Group* worldModelsGroup,
                                     float range_m,
                                     const std::string& worldSpecPath);

/// Finds the group created by \ref createSharedSceneGraph (named \ref kWorldModelsGroupName).
osg::Group* findWorldModelsGroup(osg::Group* sceneRoot);
bool computeInitialPoseNearCollision(const std::string& worldName,
                                     float range_m,
                                     osg::Vec3d& position,
                                     double& yaw,
                                     double& pitch);

// ---- Camera/frustum helpers ----
/** Perspective frustum matching imaging sonar horizontal/vertical beam angles (degrees). */
void applySonarFrustumToCamera(osg::Camera* camera,
                               int viewportWidth,
                               int viewportHeight,
                               double beam_width_deg,
                               double beam_height_deg);

/**
 * Pin camera projection to sonar FOV and keep it on window resize (Camera::FIXED + handler).
 * Call after setUpViewInWindow / realize so viewport size is valid.
 */
void installSonarFovResizeHandler(osgViewer::Viewer& viewer, double beam_width_deg, double beam_height_deg);

/** Window height for a given width so pixel aspect matches sonar angular aspect (no stretch). */
int viewerHeightForSonarAspect(int windowWidth, double beam_width_deg, double beam_height_deg);

} // namespace standalone_mvp
