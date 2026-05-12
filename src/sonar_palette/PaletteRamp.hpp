#ifndef PALETTE_H
#define PALETTE_H

#include <vector>
#include <stdexcept>
#include "PaletteTypes.hpp"

namespace sonar_palette {

class PaletteRamp {

    struct ColorPoint {     // Internal class used to store colors at different points in the gradient.
        float r, g, b;      // Red, green and blue values of our color.
        float val;          // Position of our color along the gradient (between 0 and 1).
        ColorPoint(float red, float green, float blue, float value) :
                r(red), g(green), b(blue), val(value) {
        }
    };

private:
    std::vector<ColorPoint> color;      // Color points in ascending value order.

public:
    PaletteRamp() { }
    void addColorPoint(const float red, const float green, const float blue, const float value);
    void clearGradient();
    void getColorAtValue(const float value, float &red, float &green, float &blue) const;
    void colormapSelector(const PaletteType type);
};

}

#endif
