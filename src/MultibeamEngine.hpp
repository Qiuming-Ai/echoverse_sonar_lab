#ifndef STANDALONE_MVP_MULTIBEAM_ENGINE_HPP
#define STANDALONE_MVP_MULTIBEAM_ENGINE_HPP

#include <cstdint>
#include <memory>
#include <vector>

namespace standalone_mvp {

struct MultibeamConfig {
    float range_m = 18.0f;
    float gain = 0.35f;
    std::uint32_t bin_count = 750;
    std::uint32_t beam_count = 256;
    double beam_width_deg = 120.0;
    double beam_height_deg = 20.0;
    // Acoustic settings follow Rock imaging_sonar_simulation defaults and behavior.
    bool enable_attenuation = true;
    double attenuation_frequency_khz = 300.0;
    double attenuation_temperature_c = 25.0;
    // Rock type default is 0 (freshwater-only term), keep this for compatibility.
    double attenuation_salinity_ppt = 0.0;
    double attenuation_acidity_ph = 8.0;
    bool enable_reverb = true;
    bool enable_speckle = true;
};

struct SonarFrame {
    std::uint32_t bin_count = 0;
    std::uint32_t beam_count = 0;
    std::vector<float> bins;
};

class IMultibeamBackend {
public:
    virtual ~IMultibeamBackend() = default;
    virtual void configure(const MultibeamConfig& cfg) = 0;
    virtual SonarFrame step(double x, double y, double z, double roll, double pitch, double yaw) = 0;
};

class MultibeamEngine {
public:
    MultibeamEngine();
    void configure(const MultibeamConfig& cfg);
    SonarFrame step(double x, double y, double z, double roll, double pitch, double yaw);
    const MultibeamConfig& getConfig() const;

private:
    MultibeamConfig cfg_;
    std::unique_ptr<IMultibeamBackend> backend_;
};

} // namespace standalone_mvp

#endif
