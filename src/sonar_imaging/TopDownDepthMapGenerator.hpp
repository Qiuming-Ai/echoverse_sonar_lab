#pragma once

#include <QImage>
#include <QPointF>
#include <osg/Node>
#include <osg/ref_ptr>

#include <vector>

namespace sonar_imaging {

struct TopDownDepthMapResult {
    QImage image;
    int width = 0;
    int height = 0;
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
    double far_plane_m = 0.0;
    double eye_z = 0.0;
    double z_min = 0.0;
    double z_max = 0.0;
    std::vector<float> depth_norm;
    std::vector<float> z_values;
    std::vector<unsigned char> hit_mask;

    bool valid() const { return width > 0 && height > 0 && !image.isNull() && !depth_norm.empty(); }
    bool hasHitAtPixel(const QPointF& pixel) const;
    QPointF pixelToWorld(const QPointF& pixel) const;
    QPointF worldToPixel(const QPointF& world_xy) const;
    double sampleZAtPixel(const QPointF& pixel) const;
};

class TopDownDepthMapGenerator {
public:
    TopDownDepthMapResult generate(osg::ref_ptr<osg::Node> scene_root, int width = 900, int height = 900) const;
};

} // namespace sonar_imaging

