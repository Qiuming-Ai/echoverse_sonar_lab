#ifndef _UTILS_HPP_
#define _UTILS_HPP_

// Opencv includes
#include <opencv2/core/core.hpp>

// Openscenegraph includes
#include <osg/Image>
#include <cmath>

namespace sonar_core{
    /**
    * Convert an OpenSceneGraph float RGB image into an OpenCV BGR matrix.
    * @param osg_image source OSG image
    * @param cv_image destination OpenCV matrix
    */
    void convertOsgImageToCvMat(osg::ref_ptr<osg::Image>& osg_image, cv::Mat& cv_image);
   
    /**
     * @brief Compute underwater absorption coefficient (Nepers/m)
     *
     *  This follows the widely used decomposition from
     *  "A simplified formula for viscous and chemical absorption in sea water":
     *  boric acid + magnesium sulfate + pure-water contributions.
     *
     *  @param double frequency: sound frequency in kHz.
     *  @param double temperature: water temperature in Celsius degrees.
     *  @param double depth: distance from water surface in meters.
     *  @param double salinity: amount of salt dissolved in a body of water in ppt.
     *  @param double acidity: pH water value.
     *
     *  @return attenuation coefficient in Nepers per meter
     */

    double computeAbsorptionCoefficient( const double frequency,
                                        const double temperature,
                                        const double depth,
                                        const double salinity,
                                        const double acidity);
}
#endif
