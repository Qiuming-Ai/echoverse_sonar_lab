#include "echoverse_sonar_types.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>

namespace sonar_types_v2 { namespace samples {

namespace {
uint8_t toUint8(float value) {
    if (!std::isfinite(value)) return 0;
    if (value <= 0.0f) return 0;
    if (value >= 255.0f) return 255;
    return static_cast<uint8_t>(std::lround(value));
}
} // namespace

SonarScan::SonarScan()
    : number_of_beams(0),
      number_of_bins(0),
      sampling_interval(0),
      speed_of_sound(0),
      beamwidth_horizontal(sonar_types_v2::Angle::fromRad(0)),
      beamwidth_vertical(sonar_types_v2::Angle::fromRad(0)),
      memory_layout_column(true),
      polar_coordinates(true) {
    clear();
}

SonarScan::SonarScan(uint16_t beam_count, uint16_t bin_count, sonar_types_v2::Angle first_bearing,
                     sonar_types_v2::Angle bearing_step, bool use_column_major_layout) {
    init(beam_count, bin_count, first_bearing, bearing_step, use_column_major_layout);
}

SonarScan::SonarScan(const SonarScan& other, bool deep_copy) { initFrom(other, deep_copy); }

SonarScan& SonarScan::operator=(const SonarScan& other) {
    initFrom(other, true);
    return *this;
}

void SonarScan::initFrom(const SonarScan& other, bool deep_copy) {
    init(other.number_of_beams, other.number_of_bins, other.start_bearing, other.angular_resolution, other.memory_layout_column);
    time = other.time;
    beamwidth_vertical = other.beamwidth_vertical;
    beamwidth_horizontal = other.beamwidth_horizontal;
    sampling_interval = other.sampling_interval;
    speed_of_sound = other.speed_of_sound;
    polar_coordinates = other.polar_coordinates;
    if (deep_copy) {
        setData(other.getData());
        time_beams = other.time_beams;
    }
}

void SonarScan::init(uint16_t beam_count, uint16_t bin_count, sonar_types_v2::Angle first_bearing,
                     sonar_types_v2::Angle bearing_step, bool use_column_major_layout, int fill_value) {
    if (number_of_beams != beam_count || number_of_bins != bin_count) {
        number_of_beams = beam_count;
        number_of_bins = bin_count;
        data.resize(static_cast<size_t>(beam_count) * bin_count);
    }
    start_bearing = first_bearing;
    angular_resolution = bearing_step;
    memory_layout_column = use_column_major_layout;
    speed_of_sound = 0;
    beamwidth_horizontal = sonar_types_v2::Angle::fromRad(0);
    beamwidth_vertical = sonar_types_v2::Angle::fromRad(0);
    clear(fill_value);
}

void SonarScan::clear(const int fill_value) {
    time = sonar_types_v2::Time();
    if (!data.empty() && fill_value >= 0) {
        std::memset(&data[0], fill_value % 256, data.size());
    }
    time_beams.clear();
}

int SonarScan::bearingToBeamIndex(const sonar_types_v2::Angle bearing, bool enforce_range) const {
    double bearing_offset_rad = (start_bearing - bearing).rad;
    int beam_index = static_cast<int>(std::round(bearing_offset_rad / angular_resolution.rad));
    if (enforce_range && (beam_index < 0 || beam_index >= number_of_beams)) return -1;
    return beam_index;
}

bool SonarScan::containsBeam(const SonarBeam& sonar_beam) const { return containsBeam(sonar_beam.bearing); }

bool SonarScan::containsBeam(const sonar_types_v2::Angle bearing) const {
    int beam_index = bearingToBeamIndex(bearing);
    if (beam_index < 0) return false;
    if (time_beams.empty()) return true;
    return time_beams[beam_index].microseconds != 0;
}

void SonarScan::insertBeam(const SonarBeam& sonar_beam, bool allow_resize) {
    if (memory_layout_column) throw std::runtime_error("insertBeam: unsupported memory layout");
    if (number_of_bins < sonar_beam.beam.size()) throw std::runtime_error("insertBeam: too many bins");

    int beam_index = bearingToBeamIndex(sonar_beam.bearing, false);
    if (beam_index < 0) throw std::runtime_error("insertBeam: negative index");
    if (beam_index >= number_of_beams) {
        if (!allow_resize) throw std::runtime_error("insertBeam: bearing out of range");
        number_of_beams = beam_index + 1;
        data.resize(static_cast<size_t>(number_of_beams) * number_of_bins);
    }
    if (time_beams.size() != number_of_beams) time_beams.resize(number_of_beams);

    time_beams[beam_index] = sonar_beam.time;
    sampling_interval = sonar_beam.sampling_interval;
    beamwidth_vertical = sonar_types_v2::Angle::fromRad(sonar_beam.beamwidth_vertical);
    beamwidth_horizontal = sonar_types_v2::Angle::fromRad(sonar_beam.beamwidth_horizontal);
    speed_of_sound = sonar_beam.speed_of_sound;
    std::memcpy(&data[beam_index * number_of_bins], &sonar_beam.beam[0], sonar_beam.beam.size());
}

void SonarScan::extractBeam(const sonar_types_v2::Angle bearing, SonarBeam& sonar_beam) const {
    if (memory_layout_column) throw std::runtime_error("extractBeam: wrong memory layout");
    int beam_index = bearingToBeamIndex(bearing);
    if (beam_index < 0) throw std::runtime_error("extractBeam: no data for bearing");

    sonar_beam.beam.resize(number_of_bins);
    std::memcpy(&sonar_beam.beam[0], &data[number_of_bins * beam_index], number_of_bins);
    sonar_beam.time = (static_cast<int>(time_beams.size()) > beam_index) ? time_beams[beam_index] : time;
    sonar_beam.speed_of_sound = speed_of_sound;
    sonar_beam.beamwidth_horizontal = static_cast<float>(beamwidth_horizontal.rad);
    sonar_beam.beamwidth_vertical = static_cast<float>(beamwidth_vertical.rad);
    sonar_beam.sampling_interval = sampling_interval;
    sonar_beam.bearing = bearing;
}

void SonarScan::transposeMemoryLayout() {
    std::vector<uint8_t> transposed_data(data.size());
    if (memory_layout_column) {
        for (int row = 0; row < number_of_beams; ++row)
            for (int col = 0; col < number_of_bins; ++col)
                transposed_data[row * number_of_bins + col] = data[col * number_of_beams + row];
    } else {
        for (int row = 0; row < number_of_beams; ++row)
            for (int col = 0; col < number_of_bins; ++col)
                transposed_data[col * number_of_beams + row] = data[row * number_of_bins + col];
    }
    memory_layout_column = !memory_layout_column;
    data.swap(transposed_data);
}

void SonarScan::swap(SonarScan& sonar_scan) {
    data.swap(sonar_scan.data);
    std::swap(time, sonar_scan.time);
    std::swap(beamwidth_vertical, sonar_scan.beamwidth_vertical);
    std::swap(beamwidth_horizontal, sonar_scan.beamwidth_horizontal);
    std::swap(sampling_interval, sonar_scan.sampling_interval);
    std::swap(number_of_beams, sonar_scan.number_of_beams);
    std::swap(number_of_bins, sonar_scan.number_of_bins);
    std::swap(start_bearing, sonar_scan.start_bearing);
    std::swap(angular_resolution, sonar_scan.angular_resolution);
    std::swap(memory_layout_column, sonar_scan.memory_layout_column);
    std::swap(polar_coordinates, sonar_scan.polar_coordinates);
    std::swap(speed_of_sound, sonar_scan.speed_of_sound);
}

uint32_t SonarScan::getNumberOfBytes() const { return static_cast<uint32_t>(data.size()); }
uint32_t SonarScan::getBinCount() const { return number_of_beams * number_of_bins; }
const std::vector<uint8_t>& SonarScan::getData() const { return data; }
sonar_types_v2::Angle SonarScan::getEndBearing() const { return start_bearing - angular_resolution * (number_of_beams - 1); }
sonar_types_v2::Angle SonarScan::getStartBearing() const { return start_bearing; }
sonar_types_v2::Angle SonarScan::getAngularResolution() const { return angular_resolution; }
double SonarScan::getSpatialResolution() const { return sampling_interval * 0.5 * speed_of_sound; }
void SonarScan::setData(const std::vector<uint8_t>& scan_data) { data = scan_data; }

void SonarScan::setData(const char* scan_data, uint32_t size) {
    if (size != data.size()) {
        std::cerr << "SonarScan::setData size mismatch (" << size << " != " << data.size() << ")" << std::endl;
        return;
    }
    std::memcpy(&data[0], scan_data, size);
}

uint8_t* SonarScan::getDataPtr() { return static_cast<uint8_t*>(&data[0]); }
const uint8_t* SonarScan::getDataConstPtr() const { return static_cast<const uint8_t*>(&data[0]); }

void Sonar::configureDimensions(int bins_count, int beams_count, bool per_beam_timestamps) {
    if (per_beam_timestamps) timestamps.resize(beams_count);
    else timestamps.clear();
    bearings.resize(beams_count, sonar_types_v2::Angle::unknown());
    bins.resize(static_cast<size_t>(beams_count) * bins_count, sonar_types_v2::unknown<float>());
    bin_count = bins_count;
    beam_count = beams_count;
}

Sonar Sonar::fromSingleBeam(sonar_types_v2::Time time, sonar_types_v2::Time bin_duration, sonar_types_v2::Angle beam_width,
                            sonar_types_v2::Angle beam_height, const std::vector<float>& bins,
                            sonar_types_v2::Angle bearing, float speed_of_sound) {
    Sonar sample(time, bin_duration, static_cast<int>(bins.size()), beam_width, beam_height);
    sample.speed_of_sound = speed_of_sound;
    sample.appendBeam(bins, bearing);
    return sample;
}

sonar_types_v2::Time Sonar::getBinRelativeStartTime(unsigned int bin_idx) const { return bin_duration * static_cast<double>(bin_idx); }
sonar_types_v2::Time Sonar::getBeamAcquisitionStartTime(unsigned int beam) const { return timestamps.empty() ? time : timestamps[beam]; }
sonar_types_v2::Time Sonar::getBinTime(unsigned int bin, unsigned int beam) const { return getBeamAcquisitionStartTime(beam) + getBinRelativeStartTime(bin); }
float Sonar::getBinStartDistance(unsigned int bin) const { return static_cast<float>(getBinRelativeStartTime(bin).toSeconds() * speed_of_sound); }

void Sonar::setUniformBeamBearings(sonar_types_v2::Angle start, sonar_types_v2::Angle interval) {
    sonar_types_v2::Angle angle_cursor(start);
    bearings.resize(beam_count);
    for (uint32_t i = 0; i < beam_count; ++i, angle_cursor += interval) bearings[i] = angle_cursor;
}

void Sonar::appendBeam(const std::vector<float>& beam_bins) {
    if (!timestamps.empty()) throw std::invalid_argument("appendBeam(bins) invalid for per-beam timestamps");
    appendBeamBins(beam_bins);
}

void Sonar::appendBeam(const std::vector<float>& beam_bins, sonar_types_v2::Angle bearing) {
    appendBeam(beam_bins);
    bearings.push_back(bearing);
}

void Sonar::appendBeam(const sonar_types_v2::Time& beam_time, const std::vector<float>& beam_bins) {
    appendBeamBins(beam_bins);
    timestamps.push_back(beam_time);
}

void Sonar::appendBeam(const sonar_types_v2::Time& beam_time, const std::vector<float>& beam_bins, sonar_types_v2::Angle bearing) {
    appendBeam(beam_time, beam_bins);
    bearings.push_back(bearing);
}

void Sonar::appendBeamBins(const std::vector<float>& beam_bins) {
    if (beam_bins.size() != bin_count) throw std::invalid_argument("appendBeam: beam bins size mismatch");
    bins.insert(bins.end(), beam_bins.begin(), beam_bins.end());
    beam_count++;
}

void Sonar::setBeam(unsigned int beam, const std::vector<float>& beam_bins) {
    if (!timestamps.empty()) throw std::invalid_argument("setBeam(bins) invalid for per-beam timestamps");
    setBeamBins(static_cast<int>(beam), beam_bins);
}

void Sonar::setBeam(unsigned int beam, const std::vector<float>& beam_bins, sonar_types_v2::Angle bearing) {
    setBeam(beam, beam_bins);
    bearings[beam] = bearing;
}

void Sonar::setBeam(unsigned int beam, const sonar_types_v2::Time& beam_time, const std::vector<float>& beam_bins) {
    setBeamBins(static_cast<int>(beam), beam_bins);
    timestamps[beam] = beam_time;
}

void Sonar::setBeam(unsigned int beam, const sonar_types_v2::Time& beam_time, const std::vector<float>& beam_bins,
                    sonar_types_v2::Angle bearing) {
    setBeam(beam, beam_time, beam_bins);
    bearings[beam] = bearing;
}

void Sonar::setBeamBins(int beam, const std::vector<float>& beam_bins) {
    if (beam_bins.size() != bin_count) throw std::invalid_argument("setBeamBins: beam bins size mismatch");
    std::copy(beam_bins.begin(), beam_bins.end(), bins.begin() + beam * bin_count);
}

sonar_types_v2::Angle Sonar::getBeamBearing(unsigned int beam) const { return bearings[beam]; }

std::vector<float> Sonar::getBeamBins(unsigned int beam) const {
    std::vector<float> out;
    getBeamBins(beam, out);
    return out;
}

void Sonar::getBeamBins(unsigned int beam, std::vector<float>& beam_bins) const {
    beam_bins.resize(bin_count);
    auto begin_ptr = bins.begin() + beam * bin_count;
    std::copy(begin_ptr, begin_ptr + bin_count, beam_bins.begin());
}

Sonar Sonar::getBeam(unsigned int beam) const {
    return fromSingleBeam(getBeamAcquisitionStartTime(beam), bin_duration, beam_width, beam_height, getBeamBins(beam),
                          getBeamBearing(beam), speed_of_sound);
}

void Sonar::validateConsistency() {
    if (bin_count * beam_count != bins.size()) throw std::logic_error("bins size mismatch");
    if (!timestamps.empty() && timestamps.size() != beam_count) throw std::logic_error("timestamps size mismatch");
    if (bearings.size() != beam_count) throw std::logic_error("bearings size mismatch");
}

Sonar::Sonar(SonarScan const& old, float gain)
    : time(old.time),
      timestamps(old.time_beams),
      bin_duration(sonar_types_v2::Time::fromSeconds(old.getSpatialResolution() / old.speed_of_sound)),
      beam_width(old.beamwidth_horizontal),
      beam_height(old.beamwidth_vertical),
      speed_of_sound(old.speed_of_sound),
      bin_count(old.number_of_bins),
      beam_count(old.number_of_beams) {
    if (!old.polar_coordinates) throw std::invalid_argument("non-polar sonar device is invalid");
    bins.resize(static_cast<size_t>(bin_count) * beam_count);
    SonarScan scan(old);
    if (old.memory_layout_column) scan.transposeMemoryLayout();
    for (unsigned int i = 0; i < bins.size(); ++i) bins[i] = static_cast<float>(scan.data[i] / 255.0f) * gain;
    setUniformBeamBearings(old.getStartBearing(), old.getAngularResolution());
    validateConsistency();
}

Sonar::Sonar(SonarBeam const& old, float gain)
    : time(old.time),
      timestamps(),
      bin_duration(sonar_types_v2::Time::fromSeconds(old.sampling_interval / 2.0)),
      beam_width(sonar_types_v2::Angle::fromRad(old.beamwidth_horizontal)),
      beam_height(sonar_types_v2::Angle::fromRad(old.beamwidth_vertical)),
      speed_of_sound(old.speed_of_sound),
      bin_count(static_cast<uint32_t>(old.beam.size())),
      beam_count(0) {
    std::vector<float> beam_bins(bin_count);
    for (unsigned int i = 0; i < bin_count; ++i) beam_bins[i] = static_cast<float>(old.beam[i] / 255.0f) * gain;
    appendBeam(beam_bins, old.bearing);
}

SonarBeam Sonar::toSonarBeam(float gain) {
    SonarBeam sonar_beam;
    sonar_beam.time = time;
    sonar_beam.speed_of_sound = speed_of_sound;
    sonar_beam.beamwidth_horizontal = static_cast<float>(beam_width.rad);
    sonar_beam.beamwidth_vertical = static_cast<float>(beam_height.rad);
    sonar_beam.bearing = bearings[0];
    sonar_beam.sampling_interval = bin_duration.toSeconds() * 2.0;

    std::vector<float> raw_data(bins.begin(), bins.end());
    auto max_it = std::max_element(raw_data.begin(), raw_data.end());
    if (max_it != raw_data.end() && *max_it > 1) {
        const float max_v = *max_it;
        std::transform(raw_data.begin(), raw_data.end(), raw_data.begin(), [max_v](float x) { return x / max_v; });
    }
    const float scale = 255.f * gain;
    std::transform(raw_data.begin(), raw_data.end(), raw_data.begin(), [scale](float x) { return x * scale; });
    sonar_beam.beam.resize(raw_data.size());
    std::transform(raw_data.begin(), raw_data.end(), sonar_beam.beam.begin(), toUint8);
    return sonar_beam;
}

SonarScan Sonar::toSonarScan(float gain) {
    SonarScan sonar_scan;
    sonar_scan.time = time;
    sonar_scan.time_beams = timestamps;
    sonar_scan.speed_of_sound = speed_of_sound;
    sonar_scan.number_of_bins = static_cast<uint16_t>(bin_count);
    sonar_scan.number_of_beams = static_cast<uint16_t>(beam_count);
    sonar_scan.beamwidth_horizontal = beam_width;
    sonar_scan.beamwidth_vertical = beam_height;
    sonar_scan.start_bearing = bearings[0];
    sonar_scan.angular_resolution = sonar_types_v2::Angle::fromRad(beam_width.rad / beam_count);
    sonar_scan.memory_layout_column = false;
    sonar_scan.polar_coordinates = true;

    std::vector<float> raw_data(bins.begin(), bins.end());
    auto max_it = std::max_element(raw_data.begin(), raw_data.end());
    if (max_it != raw_data.end() && *max_it > 1) {
        const float max_v = *max_it;
        std::transform(raw_data.begin(), raw_data.end(), raw_data.begin(), [max_v](float x) { return x / max_v; });
    }
    const float scale = 255.f * gain;
    std::transform(raw_data.begin(), raw_data.end(), raw_data.begin(), [scale](float x) { return x * scale; });
    sonar_scan.data.resize(raw_data.size());
    std::transform(raw_data.begin(), raw_data.end(), sonar_scan.data.begin(), toUint8);
    return sonar_scan;
}

}} // namespace sonar_types_v2::samples
