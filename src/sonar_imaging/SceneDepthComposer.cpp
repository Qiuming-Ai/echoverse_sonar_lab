#include "SceneDepthComposer.hpp"

#include <stdexcept>

#include <osg/Program>
#include <osg/Shader>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <osg/Uniform>

namespace sonar_imaging {
namespace {

const char* kEmbeddedVertexShader = R"GLSL(
#version 130

uniform mat4 osg_ViewMatrixInverse;

out vec3 worldPos;
out vec3 worldNormal;
out vec3 cameraPos;
out vec3 viewPos;
out vec3 viewNormal;
out mat3 TBN;

void main() {
    gl_Position = gl_ModelViewProjectionMatrix * gl_Vertex;
    gl_TexCoord[0] = gl_MultiTexCoord0;

    mat4 modelWorld = osg_ViewMatrixInverse * gl_ModelViewMatrix;
    worldPos = (modelWorld * gl_Vertex).xyz;
    worldNormal = mat3(modelWorld) * gl_Normal;

    viewPos = (gl_ModelViewMatrix * gl_Vertex).xyz;
    viewNormal = gl_NormalMatrix * gl_Normal;

    cameraPos = osg_ViewMatrixInverse[3].xyz / osg_ViewMatrixInverse[3].w;

    vec3 N = normalize(viewNormal);
    vec3 T = cross(N, vec3(1, 1, 1));
    vec3 B = cross(N, T) + cross(T, N);
    TBN = mat3(T, B, N);
}
)GLSL";

const char* kEmbeddedFragmentShader = R"GLSL(
#version 130

in vec3 worldPos;
in vec3 worldNormal;
in vec3 cameraPos;
in vec3 viewPos;
in vec3 viewNormal;
in mat3 TBN;

uniform bool drawNormal;
uniform bool drawDistance;
uniform bool drawReverb;
uniform float farPlane;
uniform float reflectance;
uniform float attenuationCoeff;

uniform sampler2D normalTex;
uniform bool useNormalTex;

uniform sampler2D trianglesTex;
uniform vec4 trianglesTexSize;

struct Ray {
    vec3 origin;
    vec3 direction;
    vec3 invDirection;
    int sign[3];
};

Ray makeRay(vec3 origin, vec3 direction) {
    vec3 invDirection = vec3(1.0) / direction;
    return Ray(
        origin,
        direction,
        invDirection,
        int[3] (int(invDirection.x < 0.0),
                int(invDirection.y < 0.0),
                int(invDirection.z < 0.0)));
}

struct Triangle {
    vec3 v0, v1, v2;
    vec3 center;
    vec3 normal;
};

struct Box {
    vec3 aabb[2];
};

float getTexData(sampler2D tex, int i, int j) {
    return texelFetch(tex, ivec2(i,j), 0).r;
}

Triangle getTriangleData(sampler2D tex, int idx) {
    Triangle triangle;
    triangle.v0     = vec3(getTexData(tex, idx, 0),     getTexData(tex, idx, 1),    getTexData(tex, idx, 2));
    triangle.v1     = vec3(getTexData(tex, idx, 3),     getTexData(tex, idx, 4),    getTexData(tex, idx, 5));
    triangle.v2     = vec3(getTexData(tex, idx, 6),     getTexData(tex, idx, 7),    getTexData(tex, idx, 8));
    triangle.center = vec3(getTexData(tex, idx, 9),     getTexData(tex, idx, 10),   getTexData(tex, idx, 11));
    triangle.normal = vec3(getTexData(tex, idx, 12),    getTexData(tex, idx, 13),   getTexData(tex, idx, 14));
    return triangle;
}

Box getBoxData(sampler2D tex, int idx) {
    Box box;
    box.aabb[0]     = vec3(getTexData(tex, idx, 0),     getTexData(tex, idx, 1),    getTexData(tex, idx, 2));
    box.aabb[1]     = vec3(getTexData(tex, idx, 3),     getTexData(tex, idx, 4),    getTexData(tex, idx, 5));
    return box;
}

bool boxContainsPoint(Box box, vec3 p) {
    if ((box.aabb[0].x <= p.x) && (p.x <= box.aabb[1].x)
        && (box.aabb[0].y <= p.y) && (p.y <= box.aabb[1].y)
        && (box.aabb[0].z <= p.z) && (p.z <= box.aabb[1].z))
        return true;

    return false;
}

bool rayIntersectsBox(Ray ray, Box box)
{
    float tmin  = (box.aabb[ray.sign[0]    ].x - ray.origin.x) * ray.invDirection.x;
    float tmax  = (box.aabb[1 - ray.sign[0]].x - ray.origin.x) * ray.invDirection.x;
    float tymin = (box.aabb[ray.sign[1]    ].y - ray.origin.y) * ray.invDirection.y;
    float tymax = (box.aabb[1 - ray.sign[1]].y - ray.origin.y) * ray.invDirection.y;
    float tzmin = (box.aabb[ray.sign[2]    ].z - ray.origin.z) * ray.invDirection.z;
    float tzmax = (box.aabb[1 - ray.sign[2]].z - ray.origin.z) * ray.invDirection.z;

    tmin = max(max(tmin, tymin), tzmin);
    tmax = min(min(tmax, tymax), tzmax);

    return (tmin < tmax);
}

bool rayIntersectsTriangle(Ray ray, Triangle triangle)
{
    float EPSILON = 0.0000001;

    vec3 edge1, edge2, h, s, q;
    float a, f, u, v;
    edge1 = triangle.v1 - triangle.v0;
    edge2 = triangle.v2 - triangle.v0;

    h = cross(ray.direction, edge2);
    a = dot(edge1, h);
    if (a > -EPSILON && a < EPSILON)
        return false;

    f = 1 / a;
    s = ray.origin - triangle.v0;
    u = f * dot(s, h);
    if (u < 0.0 || u > 1.0)
        return false;

    q = cross(s, edge1);
    v = f * dot(ray.direction, q);
    if (v < 0.0 || (u + v) > 1.0)
        return false;

    float t = f * dot(edge2, q);
    if (t <= EPSILON)
        return false;

    return true;
}

vec4 primaryReflections() {
    vec3 nViewPos = normalize(viewPos);
    vec3 nViewNormal = normalize(viewNormal);

    if (useNormalTex) {
        vec3 normalRGB = texture2D(normalTex, gl_TexCoord[0].xy).rgb;
        vec3 normalMap = (normalRGB * 2.0 - 1.0) * TBN;
        nViewNormal = normalize(normalMap);
    }

    if (reflectance > 0)
        nViewNormal = min(nViewNormal * reflectance, 1.0);

    float viewDistance = length(viewPos);
    float nViewDistance = viewDistance / farPlane;

    vec4 result = vec4(0, 0, 0, 1);
    if (nViewDistance <= 1) {
        if (drawDistance)   result.y = nViewDistance;
        if (drawNormal)     result.z = abs(dot(nViewPos, nViewNormal));
    }

    return result;
}

vec4 secondaryReflections(vec4 firstR) {
    vec3 worldIncident = cameraPos - worldPos;
    vec3 nWorldNormal = normalize(worldNormal);
    vec3 reflectedDir = reflect(-worldIncident, nWorldNormal);
    Ray ray = makeRay(worldPos, reflectedDir);

    vec4 result = vec4(0,0,0,1);

    if (firstR.z > 0) {
        bool triangleIntersected = false;
        for (int i = int(trianglesTexSize.y); triangleIntersected == false, i < int(trianglesTexSize.z); i++)
        {
            Box box = getBoxData(trianglesTex, i);
            bool boxIntersected = !boxContainsPoint(box, ray.origin) && rayIntersectsBox(ray, box);

            if (boxIntersected) {
                int j = (i - int(trianglesTexSize.y) + int(trianglesTexSize.x));

                int idx0 = int(getTexData(trianglesTex, j + 0, 0));
                int idx1 = int(getTexData(trianglesTex, j + 1, 0));

                Triangle tri;
                for (int k = idx0; triangleIntersected == false, k < idx1; k++) {
                    tri = getTriangleData(trianglesTex, k);
                    triangleIntersected = rayIntersectsTriangle(ray, tri);
                }

                if (triangleIntersected) {
                    float reverbDistance = length(ray.origin - tri.center);
                    float nReverbDistance = reverbDistance / farPlane;

                    vec3 nTrianglePos = normalize(cameraPos - tri.center);
                    vec3 nTriangleNormal = normalize(tri.normal);

                    if (nReverbDistance <= 1) {
                        if (drawDistance)   result.y = nReverbDistance;
                        if (drawNormal)     result.z = abs(dot(nTrianglePos, nTriangleNormal));
                    }
                }
            }
        }
    }

    return result;
}

vec4 unifiedReflections (vec4 firstR, vec4 secndR) {
    float nDistance = (firstR.y + secndR.y);
    float nNormal = (firstR.z + secndR.z);

    vec4 result = vec4(0, 0, 0, 1);
    if (nDistance <= 1) {
        result.y = nDistance;
        result.z = nNormal;
    }
    return result;
}

void main() {
    vec4 firstR = primaryReflections();
    vec4 result = firstR;

    if (drawReverb) {
        vec4 secndR = secondaryReflections(firstR);
        result = unifiedReflections(firstR, secndR);
    }

    float value = result.z * exp(-2 * attenuationCoeff * result.y * farPlane);
    result.z = value;
    gl_FragData[0] = result;
}
)GLSL";

osg::Uniform* findDepthToggleUniform(osg::StateSet* stateSet) {
    osg::Uniform* depthToggle = stateSet->getUniform("drawDistance");
    if (!depthToggle) {
        depthToggle = stateSet->getUniform("drawDepth");
    }
    return depthToggle;
}

} // namespace

SceneDepthComposer::SceneDepthComposer(float maxRange) {
    _depth_composer_root_node_ = createComposerRootWithShaderState(maxRange);
}

SceneDepthComposer::SceneDepthComposer(float maxRange, float attenuationCoeff) {
    _depth_composer_root_node_ = createComposerRootWithShaderState(maxRange, attenuationCoeff);
}

SceneDepthComposer::SceneDepthComposer() {
    _depth_composer_root_node_ = createComposerRootWithShaderState();
}

osg::ref_ptr<osg::Group> SceneDepthComposer::depthComposerNode() const {
    return _depth_composer_root_node_;
}

void SceneDepthComposer::setMaxRange(float value) {
    _depth_composer_root_node_->getOrCreateStateSet()->getUniform("farPlane")->set(value);
}

float SceneDepthComposer::getMaxRange() const {
    float value = 0.0f;
    _depth_composer_root_node_->getOrCreateStateSet()->getUniform("farPlane")->get(value);
    return value;
}

void SceneDepthComposer::setAttenuationCoefficient(float value) {
    _depth_composer_root_node_->getOrCreateStateSet()->getUniform("attenuationCoeff")->set(value);
}

float SceneDepthComposer::getAttenuationCoefficient() const {
    float value = 0.0f;
    _depth_composer_root_node_->getOrCreateStateSet()->getUniform("attenuationCoeff")->get(value);
    return value;
}

void SceneDepthComposer::setDrawNormal(bool value) {
    _depth_composer_root_node_->getOrCreateStateSet()->getUniform("drawNormal")->set(value);
}

bool SceneDepthComposer::isDrawNormal() const {
    bool value = false;
    _depth_composer_root_node_->getOrCreateStateSet()->getUniform("drawNormal")->get(value);
    return value;
}

void SceneDepthComposer::setDrawDepth(bool value) {
    osg::StateSet* stateSet = _depth_composer_root_node_->getOrCreateStateSet();
    osg::Uniform* depthToggle = findDepthToggleUniform(stateSet);
    if (depthToggle) {
        depthToggle->set(value);
    }
}

bool SceneDepthComposer::isDrawDepth() const {
    bool value = false;
    osg::StateSet* stateSet = _depth_composer_root_node_->getOrCreateStateSet();
    osg::Uniform* depthToggle = findDepthToggleUniform(stateSet);
    if (depthToggle) {
        depthToggle->get(value);
    }
    return value;
}

void SceneDepthComposer::setDrawReverb(bool value) {
    _depth_composer_root_node_->getOrCreateStateSet()->getUniform("drawReverb")->set(value);
}

bool SceneDepthComposer::isDrawReverb() const {
    bool value = false;
    _depth_composer_root_node_->getOrCreateStateSet()->getUniform("drawReverb")->get(value);
    return value;
}

void SceneDepthComposer::attachSceneNode(osg::ref_ptr<osg::Node> node) {
    if (!_sonar_state_host_node_.valid()) {
        _sonar_state_host_node_ = new osg::Group();
        _depth_composer_root_node_->addChild(_sonar_state_host_node_);
    } else {
        _sonar_state_host_node_->removeChildren(0, _sonar_state_host_node_->getNumChildren());
    }

    _sonar_state_host_node_->addChild(node);

    TrianglesVisitor visitor;
    node->accept(visitor);
    const std::vector<Triangle> triangles = visitor.collectedTriangles();
    const std::vector<uint> trianglesRef = visitor.triangleGroupOffsets();
    const std::vector<BoundingBox> bboxes = visitor.worldBoundingBoxes();

    osg::ref_ptr<osg::Texture2D> trianglesTexture;
    triangles2texture(triangles, trianglesRef, bboxes, trianglesTexture);

    osg::ref_ptr<osg::StateSet> state = _sonar_state_host_node_->getOrCreateStateSet();
    state->addUniform(new osg::Uniform(osg::Uniform::SAMPLER_2D, "trianglesTex"));
    state->getUniform("trianglesTex")->set(0);
    state->setTextureAttributeAndModes(0, trianglesTexture, osg::StateAttribute::ON);

    state->addUniform(new osg::Uniform(osg::Uniform::FLOAT_VEC4, "trianglesTexSize"));
    state->getUniform("trianglesTexSize")->set(
        osg::Vec4(static_cast<float>(triangles.size()),
                  static_cast<float>(triangles.size() + trianglesRef.size()),
                  static_cast<float>(trianglesTexture->getTextureWidth()),
                  static_cast<float>(trianglesTexture->getTextureHeight())));
}

osg::ref_ptr<osg::Group> SceneDepthComposer::createComposerRootWithShaderState(
    float maxRange, float attenuationCoeff, bool drawDepth, bool drawNormal, bool drawReverb) {
    osg::ref_ptr<osg::Group> root = new osg::Group();
    osg::ref_ptr<osg::Program> program = new osg::Program();
    osg::ref_ptr<osg::StateSet> state = root->getOrCreateStateSet();

    osg::ref_ptr<osg::Shader> vertexShader =
        new osg::Shader(osg::Shader::VERTEX, std::string(kEmbeddedVertexShader));
    osg::ref_ptr<osg::Shader> fragmentShader =
        new osg::Shader(osg::Shader::FRAGMENT, std::string(kEmbeddedFragmentShader));
    if (!vertexShader.valid() || !fragmentShader.valid()) {
        throw std::runtime_error("sonar_imaging: failed to create embedded GLSL shaders.");
    }
    program->addShader(vertexShader);
    program->addShader(fragmentShader);
    state->setAttributeAndModes(program, osg::StateAttribute::ON);

    state->addUniform(new osg::Uniform(osg::Uniform::FLOAT, "farPlane"));
    state->addUniform(new osg::Uniform(osg::Uniform::FLOAT, "attenuationCoeff"));
    state->addUniform(new osg::Uniform(osg::Uniform::BOOL, "drawDistance"));
    state->addUniform(new osg::Uniform(osg::Uniform::BOOL, "drawNormal"));
    state->addUniform(new osg::Uniform(osg::Uniform::BOOL, "drawReverb"));
    state->getUniform("farPlane")->set(maxRange);
    state->getUniform("attenuationCoeff")->set(attenuationCoeff);
    state->getUniform("drawDistance")->set(drawDepth);
    state->getUniform("drawNormal")->set(drawNormal);
    state->getUniform("drawReverb")->set(drawReverb);

    _sonar_state_host_node_ = new osg::Group();
    root->addChild(_sonar_state_host_node_);
    return root;
}

} // namespace sonar_imaging
