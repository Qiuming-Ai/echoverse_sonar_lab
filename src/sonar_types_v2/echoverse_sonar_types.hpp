#pragma once

#include <cstdint>
#include <limits>
#include <vector>

#include <sonar_types_v2/echoverse_math_types.hpp>
#include <sonar_types_v2/echoverse_time_types.hpp>

namespace sonar_types_v2 { namespace samples {
struct SonarBeam {
    sonar_types_v2::Time time;
    sonar_types_v2::Angle bearing;
    double sampling_interval;
    float speed_of_sound;
    float beamwidth_horizontal;
    float beamwidth_vertical;
    std::vector<uint8_t> beam;

    SonarBeam()
        : sampling_interval(std::numeric_limits<double>::signaling_NaN()),
          speed_of_sound(std::numeric_limits<float>::signaling_NaN()),
          beamwidth_horizontal(std::numeric_limits<float>::signaling_NaN()),
          beamwidth_vertical(std::numeric_limits<float>::signaling_NaN()) {}
};

struct SonarScan {
    SonarScan();
    SonarScan(uint16_t beam_count, uint16_t bin_count, sonar_types_v2::Angle first_bearing,
              sonar_types_v2::Angle bearing_step, bool use_column_major_layout = true);
    SonarScan(const SonarScan& other, bool deep_copy = true);
    SonarScan& operator=(const SonarScan& other);

    void initFrom(const SonarScan& other, bool deep_copy = true);
    void init(uint16_t beam_count, uint16_t bin_count, sonar_types_v2::Angle first_bearing,
              sonar_types_v2::Angle bearing_step, bool use_column_major_layout = true, int fill_value = -1);
    void clear(int fill_value = 0);
    int bearingToBeamIndex(sonar_types_v2::Angle bearing, bool enforce_range = true) const;
    bool containsBeam(const SonarBeam& sonar_beam) const;
    bool containsBeam(sonar_types_v2::Angle bearing) const;
    void insertBeam(const SonarBeam& sonar_beam, bool allow_resize = true);
    void extractBeam(sonar_types_v2::Angle bearing, SonarBeam& sonar_beam) const;
    void transposeMemoryLayout();
    void swap(SonarScan& sonar_scan);
    uint32_t getNumberOfBytes() const;
    uint32_t getBinCount() const;
    const std::vector<uint8_t>& getData() const;
    sonar_types_v2::Angle getEndBearing() const;
    sonar_types_v2::Angle getStartBearing() const;
    sonar_types_v2::Angle getAngularResolution() const;
    double getSpatialResolution() const;
    void setData(const std::vector<uint8_t>& data);
    void setData(const char* data, uint32_t size);
    uint8_t* getDataPtr();
    const uint8_t* getDataConstPtr() const;

    void reset(int const val = 0) { clear(val); }
    int beamIndexForBearing(sonar_types_v2::Angle bearing, bool range_check = true) const {
        return bearingToBeamIndex(bearing, range_check);
    }
    bool hasSonarBeam(const SonarBeam& sonar_beam) const { return containsBeam(sonar_beam); }
    bool hasSonarBeam(sonar_types_v2::Angle bearing) const { return containsBeam(bearing); }
    void addSonarBeam(const SonarBeam& sonar_beam, bool resize = true) { insertBeam(sonar_beam, resize); }
    void getSonarBeam(sonar_types_v2::Angle bearing, SonarBeam& sonar_beam) const { extractBeam(bearing, sonar_beam); }
    void toggleMemoryLayout() { transposeMemoryLayout(); }
    void init(const SonarScan& other, bool bcopy = true) { initFrom(other, bcopy); }

    sonar_types_v2::Time time;
    std::vector<uint8_t> data;
    std::vector<sonar_types_v2::Time> time_beams;
    uint16_t number_of_beams;
    uint16_t number_of_bins;
    sonar_types_v2::Angle start_bearing;
    sonar_types_v2::Angle angular_resolution;
    double sampling_interval;
    float speed_of_sound;
    sonar_types_v2::Angle beamwidth_horizontal;
    sonar_types_v2::Angle beamwidth_vertical;
    bool memory_layout_column;
    bool polar_coordinates;
};

struct Sonar {
    sonar_types_v2::Time time;
    std::vector<sonar_types_v2::Time> timestamps;
    sonar_types_v2::Time bin_duration;
    sonar_types_v2::Angle beam_width;
    sonar_types_v2::Angle beam_height;
    std::vector<sonar_types_v2::Angle> bearings;
    float speed_of_sound;
    uint32_t bin_count;
    uint32_t beam_count;
    std::vector<float> bins;

    static float getSpeedOfSoundInWater() { return 1497.0f; }

    Sonar() : speed_of_sound(getSpeedOfSoundInWater()), bin_count(0), beam_count(0) {}
    Sonar(sonar_types_v2::Time t, sonar_types_v2::Time bd, int bin_count, sonar_types_v2::Angle bw, sonar_types_v2::Angle bh)
        : time(t), bin_duration(bd), beam_width(bw), beam_height(bh), speed_of_sound(getSpeedOfSoundInWater()),
          bin_count(bin_count), beam_count(0) {}

    void configureDimensions(int bins_count, int beams_count, bool per_beam_timestamps);
    static Sonar fromSingleBeam(sonar_types_v2::Time time, sonar_types_v2::Time bin_duration, sonar_types_v2::Angle beam_width,
                                sonar_types_v2::Angle beam_height, std::vector<float> const& bins,
                                sonar_types_v2::Angle bearing = sonar_types_v2::Angle(),
                                float speed_of_sound = getSpeedOfSoundInWater());
    sonar_types_v2::Time getBinRelativeStartTime(unsigned int bin_idx) const;
    sonar_types_v2::Time getBeamAcquisitionStartTime(unsigned int beam) const;
    sonar_types_v2::Time getBinTime(unsigned int bin, unsigned int beam) const;
    float getBinStartDistance(unsigned int bin) const;
    void setUniformBeamBearings(sonar_types_v2::Angle start, sonar_types_v2::Angle interval);
    void appendBeam(std::vector<float> const& bins);
    void appendBeam(std::vector<float> const& bins, sonar_types_v2::Angle bearing);
    void appendBeam(sonar_types_v2::Time const& beam_time, std::vector<float> const& beam_bins);
    void appendBeam(sonar_types_v2::Time const& beam_time, std::vector<float> const& beam_bins, sonar_types_v2::Angle bearing);
    void appendBeamBins(std::vector<float> const& beam_bins);
    void setBeam(unsigned int beam, std::vector<float> const& bins);
    void setBeam(unsigned int beam, std::vector<float> const& bins, sonar_types_v2::Angle bearing);
    void setBeam(unsigned int beam, sonar_types_v2::Time const& beam_time, std::vector<float> const& beam_bins);
    void setBeam(unsigned int beam, sonar_types_v2::Time const& beam_time, std::vector<float> const& beam_bins,
                 sonar_types_v2::Angle bearing);
    void setBeamBins(int beam, std::vector<float> const& beam_bins);
    sonar_types_v2::Angle getBeamBearing(unsigned int beam) const;
    std::vector<float> getBeamBins(unsigned int beam) const;
    void getBeamBins(unsigned int beam, std::vector<float>& beam_bins) const;
    Sonar getBeam(unsigned int beam) const;
    void validateConsistency();

    explicit Sonar(SonarScan const& old, float gain = 1);
    explicit Sonar(SonarBeam const& old, float gain = 1);
    SonarBeam toSonarBeam(float gain = 1);
    SonarScan toSonarScan(float gain = 1);

    void resize(int bins_count, int beams_count, bool per_beam_timestamps) {
        configureDimensions(bins_count, beams_count, per_beam_timestamps);
    }
    void setRegularBeamBearings(sonar_types_v2::Angle start, sonar_types_v2::Angle interval) {
        setUniformBeamBearings(start, interval);
    }
    void pushBeam(std::vector<float> const& beam_bins) { appendBeam(beam_bins); }
    void pushBeam(std::vector<float> const& beam_bins, sonar_types_v2::Angle bearing) { appendBeam(beam_bins, bearing); }
    void pushBeam(sonar_types_v2::Time const& beam_time, std::vector<float> const& beam_bins) {
        appendBeam(beam_time, beam_bins);
    }
    void pushBeam(sonar_types_v2::Time const& beam_time, std::vector<float> const& beam_bins, sonar_types_v2::Angle bearing) {
        appendBeam(beam_time, beam_bins, bearing);
    }
    void pushBeamBins(std::vector<float> const& beam_bins) { appendBeamBins(beam_bins); }
    void validate() { validateConsistency(); }
};
}} // namespace sonar_types_v2::samples
