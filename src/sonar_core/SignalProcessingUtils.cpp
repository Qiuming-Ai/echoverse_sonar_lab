#include "SignalProcessingUtils.hpp"

// Opencv includes
#include <opencv2/imgproc/imgproc.hpp>

namespace sonar_core {
namespace {

double computeBoricAcidContribution(double frequencySquared,
                                    double salinity,
                                    double temperature,
                                    double acidity) {
    const double boricRelaxationKhz = 0.78 * std::pow(salinity / 35.0, 0.5) * std::exp(temperature / 26.0);
    return 0.106 * ((boricRelaxationKhz * frequencySquared) /
                    (frequencySquared + boricRelaxationKhz * boricRelaxationKhz)) *
           std::exp((acidity - 8.0) / 0.56);
}

double computeMagnesiumSulfateContribution(double frequencySquared,
                                           double salinity,
                                           double temperature,
                                           double depth) {
    const double magnesiumRelaxationKhz = 42.0 * std::exp(temperature / 17.0);
    return 0.52 * (1.0 + temperature / 43.0) * (salinity / 35.0) *
           ((magnesiumRelaxationKhz * frequencySquared) /
            (frequencySquared + magnesiumRelaxationKhz * magnesiumRelaxationKhz)) *
           std::exp(-depth / 6000.0);
}

double computeFreshwaterContribution(double frequencySquared, double temperature, double depth) {
    return 0.00049 * frequencySquared * std::exp(-(temperature / 27.0 + depth / 17000.0));
}

double convertDbPerKmToNepersPerMeter(double attenuationDbPerKm) {
    const double attenuationDbPerM = attenuationDbPerKm / 1000.0;
    const double linearPressureRatio = std::pow(10.0, -attenuationDbPerM / 20.0);
    return -std::log(linearPressureRatio);
}

} // namespace

void convertOsgImageToCvMat(osg::ref_ptr<osg::Image>& osg_image, cv::Mat& cv_image) {
    cv_image = cv::Mat(osg_image->t(), osg_image->s(), CV_32FC3, osg_image->data());
    cv::cvtColor(cv_image, cv_image, cv::COLOR_RGB2BGR);
}

double computeAbsorptionCoefficient(const double frequency,
                                    const double temperature,
                                    const double depth,
                                    const double salinity,
                                    const double acidity) {
    const double frequencySquared = frequency * frequency;

    // Model attenuation as the sum of boric acid, magnesium sulfate, and pure-water terms.
    const double boricTerm = computeBoricAcidContribution(frequencySquared, salinity, temperature, acidity);
    const double magnesiumTerm =
        computeMagnesiumSulfateContribution(frequencySquared, salinity, temperature, depth);
    const double freshwaterTerm = computeFreshwaterContribution(frequencySquared, temperature, depth);

    const double attenuationDbPerKm = boricTerm + magnesiumTerm + freshwaterTerm;
    return convertDbPerKmToNepersPerMeter(attenuationDbPerKm);
}

}
