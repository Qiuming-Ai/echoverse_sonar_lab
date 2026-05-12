#include "PaletteRamp.hpp"
#include <algorithm>
#include <array>
#include <iterator>
#include <limits>

namespace sonar_palette {

namespace {
using PresetPoint = std::array<float, 4>; // r, g, b, value

std::vector<PresetPoint> presetForType(const PaletteType type) {
    switch (type) {
        case PALETTE_JET:
            return {
                PresetPoint{0.0f, 0.0f, 1.0f, 0.00f},
                PresetPoint{0.0f, 1.0f, 1.0f, 0.25f},
                PresetPoint{0.0f, 1.0f, 0.0f, 0.50f},
                PresetPoint{1.0f, 1.0f, 0.0f, 0.75f},
                PresetPoint{1.0f, 0.0f, 0.0f, 1.00f}
            };
        case PALETTE_HOT:
            return {
                PresetPoint{0.0f, 0.0f, 0.0f, 0.000f},
                PresetPoint{1.0f, 0.0f, 0.0f, 0.125f},
                PresetPoint{1.0f, 1.0f, 0.0f, 0.550f},
                PresetPoint{1.0f, 1.0f, 1.0f, 1.000f}
            };
        case PALETTE_GRAYSCALE:
            return {
                PresetPoint{0.0f, 0.0f, 0.0f, 0.0f},
                PresetPoint{1.0f, 1.0f, 1.0f, 1.0f}
            };
        case PALETTE_BRONZE:
            return {
                PresetPoint{0.00f, 0.00f, 0.00f, 0.0f},
                PresetPoint{0.87f, 0.43f, 0.00f, 0.5f},
                PresetPoint{1.00f, 0.97f, 0.48f, 1.0f}
            };
        default:
            throw std::invalid_argument("Color gradient type does not match a known value");
    }
}
} // namespace

void PaletteRamp::addColorPoint(const float red, const float green, const float blue, const float value) {
    const auto it = std::lower_bound(color.begin(), color.end(), value,
        [](const ColorPoint& point, const float currentValue) {
            return point.val < currentValue;
        });
    color.insert(it, ColorPoint(red, green, blue, value));
}

void PaletteRamp::getColorAtValue(const float value, float &red, float &green, float &blue) const {
    if (color.empty()) {
        throw std::out_of_range("ERROR: There is no color in the current palette.");
    }

    if (value <= color.front().val) {
        red = color.front().r;
        green = color.front().g;
        blue = color.front().b;
        return;
    }

    if (value >= color.back().val) {
        red = color.back().r;
        green = color.back().g;
        blue = color.back().b;
        return;
    }

    const auto upper = std::lower_bound(color.begin(), color.end(), value,
        [](const ColorPoint& point, const float currentValue) {
            return point.val < currentValue;
        });

    const auto lower = std::prev(upper);
    const float delta = upper->val - lower->val;
    const float t = (delta > std::numeric_limits<float>::epsilon()) ?
        (value - lower->val) / delta : 0.0f;

    red = lower->r + (upper->r - lower->r) * t;
    green = lower->g + (upper->g - lower->g) * t;
    blue = lower->b + (upper->b - lower->b) * t;
}

void PaletteRamp::clearGradient() {
    color.clear();
}

void PaletteRamp::colormapSelector(const PaletteType type) {
    clearGradient();
    const auto preset = presetForType(type);
    color.reserve(preset.size());
    for (const auto& item : preset) {
        color.emplace_back(item[0], item[1], item[2], item[3]);
    }
}

}
