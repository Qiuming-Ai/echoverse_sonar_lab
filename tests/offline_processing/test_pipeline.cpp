#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "OfflineProcessingPipeline.hpp"

namespace fs = std::filesystem;

namespace {

void put_u16(std::array<unsigned char, 56>& header, const size_t offset,
             const std::uint16_t value) {
    header[offset] = static_cast<unsigned char>(value & 0xffu);
    header[offset + 1] = static_cast<unsigned char>((value >> 8u) & 0xffu);
}

void put_u32(std::array<unsigned char, 56>& header, const size_t offset,
             const std::uint32_t value) {
    for (int byte = 0; byte < 4; ++byte) {
        header[offset + static_cast<size_t>(byte)] =
            static_cast<unsigned char>((value >> (byte * 8)) & 0xffu);
    }
}

void put_u64(std::array<unsigned char, 56>& header, const size_t offset,
             const std::uint64_t value) {
    for (int byte = 0; byte < 8; ++byte) {
        header[offset + static_cast<size_t>(byte)] =
            static_cast<unsigned char>((value >> (byte * 8)) & 0xffu);
    }
}

void write_fixture_esl3d(const fs::path& path) {
    const std::string metadata =
        R"({"sonar_config":{"horizontal_fov_deg":10,"vertical_fov_deg":10}})";
    const std::array<float, 4> ranges{1.0f, 1.1f, 1.2f, 1.3f};
    const std::array<float, 4> intensities{1.0f, 0.8f, 0.6f, 0.4f};
    const std::uint32_t range_bytes =
        static_cast<std::uint32_t>(ranges.size() * sizeof(float));
    const std::uint32_t intensity_bytes =
        static_cast<std::uint32_t>(intensities.size() * sizeof(float));
    const std::uint32_t metadata_bytes = static_cast<std::uint32_t>(metadata.size());

    std::array<unsigned char, 56> header{};
    put_u32(header, 0, 0x5033534Eu);
    put_u16(header, 4, 1u);
    put_u16(header, 6, 56u);
    put_u64(header, 8, 1u);
    put_u64(header, 16, 1000u);
    put_u32(header, 24, 2u);
    put_u32(header, 28, 2u);
    put_u32(header, 32, 4u);
    put_u32(header, 36, metadata_bytes);
    put_u32(header, 40, range_bytes);
    put_u32(header, 44, intensity_bytes);
    put_u32(header, 48, metadata_bytes + range_bytes + intensity_bytes);

    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    output.write(reinterpret_cast<const char*>(header.data()),
                 static_cast<std::streamsize>(header.size()));
    output.write(metadata.data(), static_cast<std::streamsize>(metadata.size()));
    output.write(reinterpret_cast<const char*>(ranges.data()),
                 static_cast<std::streamsize>(range_bytes));
    output.write(reinterpret_cast<const char*>(intensities.data()),
                 static_cast<std::streamsize>(intensity_bytes));
    if (!output) throw std::runtime_error("failed to write ESL3D fixture");
}

void write_fixture_config(const fs::path& path) {
    std::ofstream output(path, std::ios::out | std::ios::trunc);
    output << R"({
  "array_params": {
    "c0": 1500, "fs": 20000, "fc": 5000, "BW": 1000,
    "Nrx": 2, "Ntx": 1, "pulse_len": 0.001,
    "interp_factor": 2, "echo_oversample_factor": 1,
    "rx_interval_lambda": 0.5, "tx_interval_lambda": 0,
    "velocity": [0], "snr_level": [120], "compensate_range": [0]
  },
  "tx_signal_params": {
    "tx_type": "lfm", "sector_num": 1, "angles_deg": 0,
    "Subfc": 5000, "SubBW": 1000, "pol": "up"
  },
  "rx_signal_params": {
    "angle_segments_deg": [[-5, 5]], "angle_step_deg": 5,
    "array_window": "hamming", "signal_window": "hamming",
    "decimation_factor": 2
  },
  "file_opt_params": {"esl3d_path": "ignored", "output_path": "ignored"}
})";
    if (!output) throw std::runtime_error("failed to write sonar config fixture");
}

}  // namespace

int main() {
    const auto nonce = std::chrono::high_resolution_clock::now()
                           .time_since_epoch()
                           .count();
    const fs::path root = fs::temp_directory_path() /
        ("echoverse_offline_pipeline_test_" + std::to_string(nonce));
    try {
        fs::create_directories(root / "output");
        const fs::path esl3d = root / "fixture.esl3d";
        const fs::path config = root / "sonar.json";
        write_fixture_esl3d(esl3d);
        write_fixture_config(config);

        sonar::offline::ProcessingOptions options;
        options.esl3d_path = esl3d.string();
        options.sonar_config_path = config.string();
        options.output_directory = (root / "output").string();
        options.image_width = 64;
        options.image_height = 48;

        const sonar::offline::ProcessingResult result =
            sonar::offline::process_esl3d_to_images(options);
        if (result.frame_count != 1 || result.image_paths.size() != 1 ||
            !fs::is_regular_file(result.hdf5_path) ||
            !fs::is_regular_file(result.image_paths.front())) {
            std::cerr << "pipeline outputs are incomplete\n";
            fs::remove_all(root);
            return 1;
        }
        fs::remove_all(root);
        std::cout << "OFFLINE PIPELINE END-TO-END TEST PASSED\n";
        return 0;
    } catch (const std::exception& error) {
        fs::remove_all(root);
        std::cerr << "FAILED: " << error.what() << '\n';
        return 1;
    }
}
