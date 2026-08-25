#include "io/esl3d_reader.h"

#include <cmath>
#include <cstring>
#include <fstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "util/log.h"

namespace sonar::io {

namespace {
constexpr uint32_t kMagic = 0x5033534E;
constexpr int kHeaderSize = 56;
}  // namespace

static uint16_t read_u16(const unsigned char* p) {
    return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}
static uint32_t read_u32(const unsigned char* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
static uint64_t read_u64(const unsigned char* p) {
    uint64_t v = 0;
    for (int i = 7; i >= 0; --i) v = (v << 8) | p[static_cast<size_t>(i)];
    return v;
}

static double get_json_double(const nlohmann::json& j, const char* key, double def) {
    if (j.contains(key) && j[key].is_number()) return j[key].get<double>();
    return def;
}

void Esl3dReader::load(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("Cannot open file: " + path);

    frames_.clear();

    std::vector<unsigned char> header(static_cast<size_t>(kHeaderSize));
    uint64_t frameIdx = 0;
    while (true) {
        f.read(reinterpret_cast<char*>(header.data()), kHeaderSize);
        if (f.gcount() == 0) break;
        if (f.gcount() != kHeaderSize)
            throw std::runtime_error("Incomplete packet header at frame " + std::to_string(frameIdx + 1));

        const uint32_t magic = read_u32(header.data());
        const uint16_t version = read_u16(header.data() + 4);
        const uint16_t header_bytes = read_u16(header.data() + 6);
        const uint64_t seq = read_u64(header.data() + 8);
        const uint64_t ts_us = read_u64(header.data() + 16);
        const uint32_t width = read_u32(header.data() + 24);
        const uint32_t height = read_u32(header.data() + 28);
        const uint32_t point_count = read_u32(header.data() + 32);
        const uint32_t metadata_bytes = read_u32(header.data() + 36);
        const uint32_t range_bytes = read_u32(header.data() + 40);
        const uint32_t intensity_bytes = read_u32(header.data() + 44);
        const uint32_t payload_bytes = read_u32(header.data() + 48);
        // reserved at +52 (4 bytes)

        if (magic != kMagic) throw std::runtime_error("Bad magic: 0x" + [&] {
            char b[16];
            std::snprintf(b, sizeof(b), "%08X", magic);
            return std::string(b);
        }());
        if (version != 1) throw std::runtime_error("Unsupported version: " + std::to_string(version));
        if (header_bytes != kHeaderSize) throw std::runtime_error("Unexpected header size: " + std::to_string(header_bytes));
        if (payload_bytes != metadata_bytes + range_bytes + intensity_bytes)
            throw std::runtime_error("Payload length mismatch.");

        std::vector<unsigned char> payload(static_cast<size_t>(payload_bytes));
        f.read(reinterpret_cast<char*>(payload.data()), static_cast<std::streamsize>(payload_bytes));
        if (f.gcount() != static_cast<std::streamsize>(payload_bytes))
            throw std::runtime_error("Unexpected EOF while reading payload at frame " + std::to_string(frameIdx + 1));

        Esl3dFrame fr;
        fr.seq = seq;
        fr.ts_us = ts_us;
        fr.width = static_cast<int>(width);
        fr.height = static_cast<int>(height);
        fr.metadata_json.assign(reinterpret_cast<const char*>(payload.data()),
                                static_cast<size_t>(metadata_bytes));

        const size_t range_start = metadata_bytes;
        const size_t range_end = range_start + range_bytes;
        const size_t inten_start = range_end;

        // MATLAB: reshape(raw,[width,height])'  =>  img(i,j) = raw[i*width + j]
        fr.range.resize(static_cast<int>(height), static_cast<int>(width));
        fr.intensity.resize(static_cast<int>(height), static_cast<int>(width));
        for (size_t i = 0; i < static_cast<size_t>(height); ++i) {
            for (size_t j = 0; j < static_cast<size_t>(width); ++j) {
                const size_t src = i * width + j;
                float rv, iv;
                std::memcpy(&rv, payload.data() + range_start + src * 4, 4);
                std::memcpy(&iv, payload.data() + inten_start + src * 4, 4);
                fr.range(static_cast<int>(i), static_cast<int>(j)) = static_cast<double>(rv);
                fr.intensity(static_cast<int>(i), static_cast<int>(j)) = static_cast<double>(iv);
            }
        }
        (void)point_count;
        frames_.push_back(std::move(fr));
        ++frameIdx;
    }

    if (frames_.empty()) throw std::runtime_error("No frames found in file: " + path);
    SONAR_LOG_INFO("esl3d loaded %d frame(s) from %s", frameCount(), path.c_str());
}

const Esl3dFrame& Esl3dReader::frame(int idx) const {
    if (idx < 0 || idx >= frameCount())
        throw std::out_of_range("esl3d frame index out of range: " + std::to_string(idx));
    return frames_[static_cast<size_t>(idx)];
}

void Esl3dReader::getPointCloud(int idx, FramePointCloud& out) const {
    const Esl3dFrame& fr = frame(idx);
    const int h = fr.height;
    const int w = fr.width;

    nlohmann::json meta;
    try {
        meta = nlohmann::json::parse(fr.metadata_json);
    } catch (...) {
        meta = nlohmann::json::object();
    }
    double hfov = 90.0, vfov = 30.0;
    if (meta.contains("sonar_config")) {
        const auto& sc = meta["sonar_config"];
        hfov = get_json_double(sc, "horizontal_fov_deg", hfov);
        vfov = get_json_double(sc, "vertical_fov_deg", vfov);
    }
    const double hfov_r = hfov * M_PI / 180.0;
    const double vfov_r = vfov * M_PI / 180.0;

    // az/el grids (MATLAB linspace + meshgrid)
    std::vector<double> az(static_cast<size_t>(w)), el(static_cast<size_t>(h));
    for (int j = 0; j < w; ++j)
        az[static_cast<size_t>(j)] = (w == 1) ? 0.0 : -hfov_r * 0.5 + hfov_r * j / (w - 1);
    for (int i = 0; i < h; ++i)
        el[static_cast<size_t>(i)] = (h == 1) ? 0.0 : -vfov_r * 0.5 + vfov_r * i / (h - 1);

    // collect valid points in column-major order (MATLAB linear indexing)
    std::vector<double> xs, ys, zs, amps;
    xs.reserve(static_cast<size_t>(h) * w);
    for (int j = 0; j < w; ++j) {
        for (int i = 0; i < h; ++i) {
            const double r = fr.range(i, j);
            if (!std::isfinite(r) || r <= 0.0) continue;
            const double azv = az[static_cast<size_t>(j)];
            const double elv = el[static_cast<size_t>(i)];
            const double cosel = std::cos(elv);
            const double z = r * cosel * std::cos(azv);
            const double x = r * cosel * std::sin(azv);
            const double y = r * std::sin(elv);
            xs.push_back(x);
            ys.push_back(y);
            zs.push_back(z);
            amps.push_back(fr.intensity(i, j));
        }
    }

    const int K = static_cast<int>(xs.size());
    out.seq = fr.seq;
    out.ts_us = fr.ts_us;
    out.points.resize(K, 3);
    out.amplitudes.resize(K, 1);
    for (int k = 0; k < K; ++k) {
        out.points(k, 0) = xs[static_cast<size_t>(k)];
        out.points(k, 1) = ys[static_cast<size_t>(k)];
        out.points(k, 2) = zs[static_cast<size_t>(k)];
        out.amplitudes(k, 0) = amps[static_cast<size_t>(k)];
    }
}

size_t Esl3dReader::totalPointCount() const {
    size_t total = 0;
    for (const auto& fr : frames_) total += static_cast<size_t>(fr.width) * fr.height;
    return total;
}

}  // namespace sonar::io

