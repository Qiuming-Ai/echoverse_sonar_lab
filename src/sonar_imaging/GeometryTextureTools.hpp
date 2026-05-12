#ifndef SIMULATION_NORMAL_DEPTH_MAP_SRC_TOOLS_HPP_
#define SIMULATION_NORMAL_DEPTH_MAP_SRC_TOOLS_HPP_

// C++ includes
#include <vector>
#include <iostream>

// OSG includes
#include <osg/Node>
#include <osg/Geode>
#include <osg/TriangleFunctor>
#include <osg/Texture2D>

namespace sonar_imaging {

    /**
     * Triangle definition.
    */
    struct Triangle
    {
        // Layout contract for GPU upload:
        // [v0, v1, v2, centroid, unit_normal] as contiguous float triplets.
        std::vector<osg::Vec3f> packed_vertices;

        Triangle()
            : packed_vertices(5, osg::Vec3f(0, 0, 0)){};

        Triangle(osg::Vec3f v1, osg::Vec3f v2, osg::Vec3f v3)
            : packed_vertices(5, osg::Vec3f(0, 0, 0))
        {
            assignVerticesAndDerivedData(v1, v2, v3);
        };

        void assignVerticesAndDerivedData(osg::Vec3f v1, osg::Vec3f v2, osg::Vec3f v3)
        {
            packed_vertices[0] = v1;
            packed_vertices[1] = v2;
            packed_vertices[2] = v3;
            packed_vertices[3] = (v1 + v2 + v3) / 3;
            packed_vertices[4] = (v2 - v1)^(v3 - v1);
            packed_vertices[4].normalize();
        };

        std::vector<float> flattenToFloatArray() const
        {
            std::vector<float> flattened;
            flattened.reserve(packed_vertices.size() * 3);
            for (const osg::Vec3f& point : packed_vertices) {
                flattened.push_back(point.x());
                flattened.push_back(point.y());
                flattened.push_back(point.z());
            }
            return flattened;
        }
    };

    /**
     * Bounding box definition.
    */
    struct BoundingBox
    {
        // Layout contract for GPU upload: [min_corner, max_corner].
        std::vector<osg::Vec3f> packed_corners;

        BoundingBox()
            : packed_corners(2, osg::Vec3f(0, 0, 0)){};

        BoundingBox(osg::Vec3f min, osg::Vec3f max)
            : packed_corners(2, osg::Vec3f(0, 0, 0))
        {
            packed_corners[0] = min;
            packed_corners[1] = max;
        };

        std::vector<float> flattenToFloatArray() const
        {
            std::vector<float> flattened;
            flattened.reserve(packed_corners.size() * 3);
            for (const osg::Vec3f& corner : packed_corners) {
                flattened.push_back(corner.x());
                flattened.push_back(corner.y());
                flattened.push_back(corner.z());
            }
            return flattened;
        }
    };

    /**
     * Visit a node and store the triangles and bouding boxes data (in world coordinates)
     * of each 3D model.
     */
    class TrianglesVisitor : public osg::NodeVisitor
    {
      protected:
        struct WorldTriangleCollector
        {
            std::vector<Triangle> collected_triangles;
            osg::Matrixd local_to_world;

            inline void operator()(const osg::Vec3f &v1,
                                   const osg::Vec3f &v2,
                                   const osg::Vec3f &v3,
                                   bool treatVertexDataAsTemporary)
            {
                (void)treatVertexDataAsTemporary;
                osg::Vec3f world_v1 = v1 * local_to_world;
                osg::Vec3f world_v2 = v2 * local_to_world;
                osg::Vec3f world_v3 = v3 * local_to_world;
                collected_triangles.push_back(Triangle(world_v1, world_v2, world_v3));
            };

            inline void operator()(const osg::Vec3f &v1,
                                   const osg::Vec3f &v2,
                                   const osg::Vec3f &v3)
            {
                (*this)(v1, v2, v3, false);
            };
        };
        osg::TriangleFunctor<WorldTriangleCollector> triangle_functor_;
        std::vector<uint> triangle_group_offsets_;
        std::vector<BoundingBox> world_bounding_boxes_;

      public:
        TrianglesVisitor()
        {
            setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
            triangle_group_offsets_.push_back(0);
        };

        void apply(osg::Geode &geode)
        {
            // Every drawable is converted into world-space triangle records.
            triangle_functor_.local_to_world = osg::computeLocalToWorld(this->getNodePath());

            for (size_t idx = 0; idx < geode.getNumDrawables(); ++idx)
            {
                geode.getDrawable(idx)->accept(triangle_functor_);
                triangle_group_offsets_.push_back(triangle_functor_.collected_triangles.size());

                osg::BoundingBox bb = geode.getDrawable(idx)->getBoundingBox();
                if (!bb.valid()) {
                    bb.expandBy(geode.getDrawable(idx)->getBound());
                }
                BoundingBox world_bb(bb._min * triangle_functor_.local_to_world,
                                     bb._max * triangle_functor_.local_to_world);
                world_bounding_boxes_.push_back(world_bb);
            }
        }

        std::vector<Triangle> collectedTriangles() const { return triangle_functor_.collected_triangles; };
        std::vector<uint> triangleGroupOffsets() const { return triangle_group_offsets_; };
        std::vector<BoundingBox> worldBoundingBoxes() const { return world_bounding_boxes_; };
    };

    /**
     * Set the pixel value of an osg image.
     *
     * @param image: source OSG image data.
     * @param x: column index of source image.
     * @param y: row index of source image.
     * @param k: channel index of source image.
     * @param value: the new pixel value.
     */
    template <typename T>
    void writeImageChannelValue(osg::ref_ptr<osg::Image> &image,
                                unsigned int x,
                                unsigned int y,
                                unsigned int k,
                                T value)
    {

        bool valid = (y < (unsigned int)image->s())
                     && (x < (unsigned int)image->t())
                     && (k < (unsigned int)image->r());

        if (!valid) {
            std::cout << "Not valid" << std::endl;
            return;
        }

        uint linearOffset = (x * image->s() + y) * image->r() + k;

        T *rawData = (T *)image->data();
        rawData = rawData + linearOffset;
        *rawData = value;
    }

    /**
     * Convert the triangles/meshes data extracted from 3D models into
     * an OSG texture to be read by shader.
     *
     * @param triangles: vector of triangles.
     * @param trianglesRef: index of triangles per 3D model on texture.
     * @param bboxes: bounding box data (vmin and vmax) of each 3D model.
     * @param texture: final OSG texture with triangles data.
     */
    void triangles2texture(
        std::vector<Triangle> triangles,
        std::vector<uint> trianglesRef,
        std::vector<BoundingBox> bboxes,
        osg::ref_ptr<osg::Texture2D> &texture);
}

#endif
