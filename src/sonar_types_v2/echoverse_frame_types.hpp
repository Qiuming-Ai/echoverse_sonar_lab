#pragma once

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <sonar_types_v2/echoverse_time_types.hpp>

namespace sonar_types_v2 { namespace samples { namespace frame {
struct frame_attrib_t {
    std::string data_;
    std::string name_;
    void set(const std::string& name, const std::string& data) {
        name_ = name;
        data_ = data;
    }
};

struct frame_size_t {
    frame_size_t() : width(0), height(0) {}
    frame_size_t(uint16_t w, uint16_t h) : width(w), height(h) {}
    bool operator==(const frame_size_t& other) const { return width == other.width && height == other.height; }
    bool operator!=(const frame_size_t& other) const { return !(*this == other); }
    uint16_t width;
    uint16_t height;
};

enum frame_mode_t {
    MODE_UNDEFINED = 0,
    MODE_GRAYSCALE = 1,
    MODE_RGB = 2,
    MODE_UYVY = 3,
    MODE_BGR = 4,
    MODE_RGB32 = 5,
    RAW_MODES = 128,
    MODE_BAYER = RAW_MODES + 0,
    MODE_BAYER_RGGB = RAW_MODES + 1,
    MODE_BAYER_GRBG = RAW_MODES + 2,
    MODE_BAYER_BGGR = RAW_MODES + 3,
    MODE_BAYER_GBRG = RAW_MODES + 4,
    COMPRESSED_MODES = 256,
    MODE_PJPG = COMPRESSED_MODES + 1,
    MODE_JPEG = COMPRESSED_MODES + 2,
    MODE_PNG = COMPRESSED_MODES + 3
};

enum frame_status_t { STATUS_EMPTY, STATUS_VALID, STATUS_INVALID };

struct Frame {
    Frame() : data_depth(8), pixel_size(1), row_size(0), frame_mode(MODE_GRAYSCALE), frame_status(STATUS_EMPTY) {}
    Frame(uint16_t width, uint16_t height, uint8_t depth = 8, frame_mode_t mode = MODE_GRAYSCALE, int16_t val = 0,
          size_t sizeInBytes = 0) {
        init(width, height, depth, mode, val, sizeInBytes);
    }
    Frame(const Frame& other, bool bcopy = true) { init(other, bcopy); }

    void copyImageIndependantAttributes(const Frame& other) {
        time = other.time;
        received_time = other.received_time;
        attributes = other.attributes;
    }

    void init(const Frame& other, bool bcopy = true) {
        time = other.time;
        received_time = other.received_time;
        attributes = other.attributes;
        size = other.size;
        data_depth = other.data_depth;
        pixel_size = other.pixel_size;
        row_size = other.row_size;
        frame_mode = other.frame_mode;
        frame_status = other.frame_status;
        if (bcopy) {
            image = other.image;
        } else {
            image.resize(other.image.size());
        }
    }

    void init(uint16_t width, uint16_t height, uint8_t depth = 8, frame_mode_t mode = MODE_GRAYSCALE, int16_t val = 0,
              size_t sizeInBytes = 0) {
        size = frame_size_t(width, height);
        data_depth = depth;
        frame_mode = mode;
        pixel_size = getPixelSize();
        row_size = getRowSize();
        if (sizeInBytes == 0) {
            sizeInBytes = isCompressed() ? image.size() : static_cast<size_t>(row_size) * size.height;
        }
        image.resize(sizeInBytes);
        reset(val);
        frame_status = STATUS_VALID;
    }

    void reset(int val = 0) {
        if (val >= 0 && !image.empty()) {
            std::memset(image.data(), val % 256, image.size());
        }
    }

    void swap(Frame& frame) {
        std::swap(time, frame.time);
        std::swap(received_time, frame.received_time);
        image.swap(frame.image);
        attributes.swap(frame.attributes);
        std::swap(size, frame.size);
        std::swap(data_depth, frame.data_depth);
        std::swap(pixel_size, frame.pixel_size);
        std::swap(row_size, frame.row_size);
        std::swap(frame_mode, frame.frame_mode);
        std::swap(frame_status, frame.frame_status);
    }

    bool isHDR() const { return data_depth > 8; }
    void setHDR(bool value) { data_depth = value ? std::max<uint32_t>(data_depth, 16) : std::min<uint32_t>(data_depth, 8); }
    bool isCompressed() const { return frame_mode >= COMPRESSED_MODES; }
    bool isGrayscale() const { return getChannelCount() == 1; }
    bool isRGB() const { return getChannelCount() == 3 || frame_mode == MODE_RGB32; }
    bool isBayer() const { return frame_mode >= RAW_MODES && frame_mode < COMPRESSED_MODES; }

    void setStatus(frame_status_t value) { frame_status = value; }
    frame_status_t getStatus() const { return frame_status; }

    uint32_t getChannelCount() const { return getChannelCount(frame_mode); }
    static uint32_t getChannelCount(frame_mode_t mode) {
        switch (mode) {
            case MODE_GRAYSCALE:
            case MODE_BAYER:
            case MODE_BAYER_RGGB:
            case MODE_BAYER_GRBG:
            case MODE_BAYER_BGGR:
            case MODE_BAYER_GBRG:
                return 1;
            case MODE_RGB:
            case MODE_BGR:
                return 3;
            case MODE_UYVY:
            case MODE_RGB32:
                return 4;
            default:
                return 1;
        }
    }

    static frame_mode_t toFrameMode(const std::string& str) {
        if (str == "MODE_GRAYSCALE" || str == "GRAYSCALE") return MODE_GRAYSCALE;
        if (str == "MODE_RGB" || str == "RGB") return MODE_RGB;
        if (str == "MODE_UYVY" || str == "UYVY") return MODE_UYVY;
        if (str == "MODE_BGR" || str == "BGR") return MODE_BGR;
        if (str == "MODE_RGB32" || str == "RGB32") return MODE_RGB32;
        if (str == "MODE_BAYER" || str == "BAYER") return MODE_BAYER;
        if (str == "MODE_BAYER_RGGB" || str == "BAYER_RGGB") return MODE_BAYER_RGGB;
        if (str == "MODE_BAYER_GRBG" || str == "BAYER_GRBG") return MODE_BAYER_GRBG;
        if (str == "MODE_BAYER_BGGR" || str == "BAYER_BGGR") return MODE_BAYER_BGGR;
        if (str == "MODE_BAYER_GBRG" || str == "BAYER_GBRG") return MODE_BAYER_GBRG;
        if (str == "MODE_PJPG" || str == "PJPG") return MODE_PJPG;
        if (str == "MODE_JPEG" || str == "JPEG") return MODE_JPEG;
        if (str == "MODE_PNG" || str == "PNG") return MODE_PNG;
        return MODE_UNDEFINED;
    }

    frame_mode_t getFrameMode() const { return frame_mode; }
    uint32_t getPixelSize() const {
        if (isCompressed()) return 0;
        return getChannelCount() * ((data_depth + 7) / 8);
    }
    uint32_t getRowSize() const {
        if (isCompressed()) return 0;
        return getPixelSize() * size.width;
    }
    uint32_t getNumberOfBytes() const { return static_cast<uint32_t>(image.size()); }
    uint32_t getPixelCount() const { return static_cast<uint32_t>(size.width) * size.height; }
    uint32_t getDataDepth() const { return data_depth; }
    void setDataDepth(uint32_t value) {
        data_depth = value;
        pixel_size = getPixelSize();
        row_size = getRowSize();
    }
    void setFrameMode(frame_mode_t mode) {
        frame_mode = mode;
        pixel_size = getPixelSize();
        row_size = getRowSize();
    }
    frame_size_t getSize() const { return size; }
    uint16_t getWidth() const { return size.width; }
    uint16_t getHeight() const { return size.height; }
    const std::vector<uint8_t>& getImage() const { return image; }

    void validateImageSize(size_t sizeToValidate) const {
        if (!isCompressed() && sizeToValidate != static_cast<size_t>(row_size) * size.height) {
            throw std::runtime_error("Frame image size mismatch");
        }
    }

    void setImage(const std::vector<uint8_t>& newImage) {
        validateImageSize(newImage.size());
        image = newImage;
    }
    void setImage(const char* data, size_t newImageSize) { setImage(reinterpret_cast<const uint8_t*>(data), newImageSize); }
    void setImage(const uint8_t* data, size_t newImageSize) {
        validateImageSize(newImageSize);
        image.resize(newImageSize);
        if (newImageSize > 0) {
            std::memcpy(image.data(), data, newImageSize);
        }
    }

    uint8_t* getImagePtr() { return image.empty() ? nullptr : image.data(); }
    const uint8_t* getImageConstPtr() const { return image.empty() ? nullptr : image.data(); }
    uint8_t* getLastByte() { return image.empty() ? nullptr : &image.back(); }
    const uint8_t* getLastConstByte() const { return image.empty() ? nullptr : &image.back(); }

    bool hasAttribute(const std::string& name) const {
        for (const auto& attr : attributes) {
            if (attr.name_ == name) return true;
        }
        return false;
    }

    template <typename T> inline T getAttribute(const std::string& name) const {
        static T default_value{};
        std::stringstream strstr;
        for (const auto& attr : attributes) {
            if (attr.name_ == name) {
                T data{};
                strstr << attr.data_;
                strstr >> data;
                return data;
            }
        }
        return default_value;
    }

    bool deleteAttribute(const std::string& name) {
        auto it = std::remove_if(attributes.begin(), attributes.end(), [&](const frame_attrib_t& attr) { return attr.name_ == name; });
        bool removed = it != attributes.end();
        attributes.erase(it, attributes.end());
        return removed;
    }

    template <typename T> inline void setAttribute(const std::string& name, const T& data) {
        std::stringstream strstr;
        strstr << data;
        for (auto& attr : attributes) {
            if (attr.name_ == name) {
                attr.set(name, strstr.str());
                return;
            }
        }
        attributes.push_back(frame_attrib_t());
        attributes.back().set(name, strstr.str());
    }

    template <typename Tp> Tp& at(unsigned int column, unsigned int row) {
        if (column >= size.width || row >= size.height) throw std::runtime_error("out of index");
        return *((Tp*)(getImagePtr() + row * getRowSize() + column * getPixelSize()));
    }

    sonar_types_v2::Time time;
    sonar_types_v2::Time received_time;
    std::vector<uint8_t> image;
    std::vector<frame_attrib_t> attributes;
    frame_size_t size;
    uint32_t data_depth;
    uint32_t pixel_size;
    uint32_t row_size;
    frame_mode_t frame_mode;
    frame_status_t frame_status;
};

struct FramePair {
    sonar_types_v2::Time time;
    Frame first;
    Frame second;
    uint32_t id;
};

using FrameAttribute = frame_attrib_t;
using FrameSize = frame_size_t;
using FrameMode = frame_mode_t;
using FrameStatus = frame_status_t;
}}} // namespace sonar_types_v2::samples::frame
