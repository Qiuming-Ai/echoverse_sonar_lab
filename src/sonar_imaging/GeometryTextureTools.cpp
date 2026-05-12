#include "GeometryTextureTools.hpp"

// C++ includes
#include <cmath>
#include <iostream>

namespace sonar_imaging {
namespace {

osg::ref_ptr<osg::Image> allocateGeometryTextureImage(const std::vector<Triangle>& triangles,
                                                      const std::vector<uint>& triangleOffsets,
                                                      const std::vector<BoundingBox>& worldBboxes) {
    osg::ref_ptr<osg::Image> geometryImage = new osg::Image();
    geometryImage->allocateImage((triangles.size() + triangleOffsets.size() + worldBboxes.size()),
                                 triangles[0].flattenToFloatArray().size(),
                                 1,
                                 GL_RED,
                                 GL_FLOAT);
    geometryImage->setInternalTextureFormat(GL_R32F);
    return geometryImage;
}

void writePackedTriangles(const std::vector<Triangle>& triangles, osg::ref_ptr<osg::Image>& geometryImage) {
    for (size_t triangleRow = 0; triangleRow < triangles.size(); ++triangleRow) {
        const std::vector<float> packedTriangle = triangles[triangleRow].flattenToFloatArray();
        for (size_t scalarColumn = 0; scalarColumn < packedTriangle.size(); ++scalarColumn) {
            writeImageChannelValue(geometryImage, scalarColumn, triangleRow, 0, packedTriangle[scalarColumn]);
        }
    }
}

void writeTriangleOffsets(const std::vector<uint>& triangleOffsets,
                          size_t triangleCount,
                          osg::ref_ptr<osg::Image>& geometryImage) {
    for (size_t offsetRow = 0; offsetRow < triangleOffsets.size(); ++offsetRow) {
        const size_t imageRow = triangleCount + offsetRow;
        writeImageChannelValue(geometryImage, 0, imageRow, 0, static_cast<float>(triangleOffsets[offsetRow]));
    }
}

void writeBoundingBoxes(const std::vector<BoundingBox>& worldBboxes,
                        size_t triangleCount,
                        size_t triangleOffsetCount,
                        osg::ref_ptr<osg::Image>& geometryImage) {
    for (size_t bboxRow = 0; bboxRow < worldBboxes.size(); ++bboxRow) {
        const size_t imageRow = triangleCount + triangleOffsetCount + bboxRow;
        const std::vector<float> packedBounds = worldBboxes[bboxRow].flattenToFloatArray();
        for (size_t scalarColumn = 0; scalarColumn < packedBounds.size(); ++scalarColumn) {
            writeImageChannelValue(geometryImage, scalarColumn, imageRow, 0, packedBounds[scalarColumn]);
        }
    }
}

osg::ref_ptr<osg::Texture2D> buildGeometryTexture(osg::ref_ptr<osg::Image>& geometryImage) {
    osg::ref_ptr<osg::Texture2D> geometryTexture = new osg::Texture2D;
    geometryTexture->setTextureSize(geometryImage->s(), geometryImage->t());
    geometryTexture->setResizeNonPowerOfTwoHint(false);
    geometryTexture->setUnRefImageDataAfterApply(true);
    geometryTexture->setImage(geometryImage);
    return geometryTexture;
}

} // namespace

void triangles2texture(
    std::vector<Triangle> triangles,
    std::vector<uint> trianglesRef,
    std::vector<BoundingBox> bboxes,
    osg::ref_ptr<osg::Texture2D> &texture)
{
    // Pack world geometry into a single float texture:
    // [triangle rows][triangle offset rows][bbox rows].
    osg::ref_ptr<osg::Image> packedGeometryImage = allocateGeometryTextureImage(triangles, trianglesRef, bboxes);
    writePackedTriangles(triangles, packedGeometryImage);
    writeTriangleOffsets(trianglesRef, triangles.size(), packedGeometryImage);
    writeBoundingBoxes(bboxes, triangles.size(), trianglesRef.size(), packedGeometryImage);
    texture = buildGeometryTexture(packedGeometryImage);
}
}
