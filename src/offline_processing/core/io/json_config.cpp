#include "io/json_config.h"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "util/log.h"

namespace sonar::io {

namespace {
template <typename T>
std::vector<T> json_to_vector(const nlohmann::json& j) {
    std::vector<T> out;
    if (j.is_array()) {
        out.reserve(j.size());
        for (const auto& e : j) out.push_back(e.get<T>());
    } else if (j.is_number()) {
        out.push_back(j.get<T>());
    }
    return out;
}

double json_double(const nlohmann::json& j, const char* key, double def) {
    if (j.contains(key) && j[key].is_number()) return j[key].get<double>();
    return def;
}
int json_int(const nlohmann::json& j, const char* key, int def) {
    if (j.contains(key) && j[key].is_number()) return j[key].get<int>();
    return def;
}
std::string json_string(const nlohmann::json& j, const char* key, const std::string& def) {
    if (j.contains(key) && j[key].is_string()) return j[key].get<std::string>();
    return def;
}
}  // namespace

SonarConfig load_sonar_config(const std::string& json_path) {
    std::ifstream f(json_path);
    if (!f) throw std::runtime_error("Cannot open config: " + json_path);
    std::stringstream ss;
    ss << f.rdbuf();
    nlohmann::json cfg;
    try {
        cfg = nlohmann::json::parse(ss.str());
    } catch (const std::exception& e) {
        throw std::runtime_error("Failed to parse JSON config " + json_path + ": " + e.what());
    }

    SonarConfig s;
    s.configPath = json_path;

    const auto& ap = cfg.value("array_params", nlohmann::json::object());
    s.c0 = json_double(ap, "c0", 1500.0);
    s.fs = json_double(ap, "fs", 2e6);
    s.fc = json_double(ap, "fc", 450e3);
    s.BW = json_double(ap, "BW", 100e3);
    s.Nrx = json_int(ap, "Nrx", 128);
    s.Ntx = json_int(ap, "Ntx", 1);
    s.pulse_len = json_double(ap, "pulse_len", 0.005);
    s.interp_factor = json_double(ap, "interp_factor", 8.0);
    s.echo_oversample_factor = json_int(ap, "echo_oversample_factor", 5);
    if (s.echo_oversample_factor <= 0)
        throw std::runtime_error("echo_oversample_factor must be a positive integer");
    s.tx_interval_lambda = json_double(ap, "tx_interval_lambda", 0.0);
    s.rx_interval_lambda = json_double(ap, "rx_interval_lambda", 0.5);
    if (ap.contains("lightPos")) s.lightPos = json_to_vector<double>(ap["lightPos"]);
    if (ap.contains("velocity")) s.velocity = json_to_vector<double>(ap["velocity"]);
    if (ap.contains("snr_level")) s.snr_level = json_to_vector<double>(ap["snr_level"]);
    if (ap.contains("compensate_range")) s.compensate_range = json_to_vector<double>(ap["compensate_range"]);

    const auto& tx = cfg.value("tx_signal_params", nlohmann::json::object());
    s.tx_type = json_string(tx, "tx_type", "lfm");
    s.sector_num = json_int(tx, "sector_num", 1);
    if (tx.contains("angles_deg")) s.angles_deg = json_to_vector<double>(tx["angles_deg"]);
    if (tx.contains("Subfc")) s.Subfc = json_to_vector<double>(tx["Subfc"]);
    if (tx.contains("SubBW")) s.SubBW = json_to_vector<double>(tx["SubBW"]);
    if (tx.contains("pol")) {
        const auto& p = tx["pol"];
        if (p.is_array()) {
            for (const auto& e : p) s.pol_vec.push_back(e.get<std::string>());
        } else if (p.is_string()) {
            s.pol = p.get<std::string>();
        }
    }

    const auto& rx = cfg.value("rx_signal_params", nlohmann::json::object());
    s.array_window = json_string(rx, "array_window", "hamming");
    s.signal_window = json_string(rx, "signal_window", "hamming");
    s.decimation_factor = json_int(rx, "decimation_factor", 16);

    // angles_div: angle_segments_deg + angle_step_deg | angles_div_deg | defaults
    if (rx.contains("angle_segments_deg") && rx.contains("angle_step_deg")) {
        const double step = json_double(rx, "angle_step_deg", 1.0);
        if (step <= 0.0) throw std::runtime_error("angle_step_deg must be positive.");
        const auto& segs = rx["angle_segments_deg"];
        if (segs.is_array()) {
            for (const auto& seg : segs) {
                std::vector<double> row = json_to_vector<double>(seg);
                if (row.size() != 2) throw std::runtime_error("angle_segments_deg row must be [start end]");
                std::vector<double> flat;
                for (double a = row[0]; a <= row[1] + 1e-9; a += step) flat.push_back(a);
                s.angles_div.push_back(std::move(flat));
            }
        }
    } else if (rx.contains("angles_div_deg")) {
        const auto& adv = rx["angles_div_deg"];
        if (adv.is_array()) {
            if (!adv.empty() && adv[0].is_array()) {
                for (const auto& row : adv) s.angles_div.push_back(json_to_vector<double>(row));
            } else {
                s.angles_div.push_back(json_to_vector<double>(adv));
            }
        }
    }
    if (s.angles_div.empty()) {
        // backward-compatible default
        s.angles_div = {{-60, 0.1, -34.1}, {-34, 0.1, -17.1}, {-17, 0.1, -0.1},
                        {0, 0.1, 16.9}, {17, 0.1, 33.9}, {34, 0.1, 60}};
    }

    const auto& fop = cfg.value("file_opt_params", nlohmann::json::object());
    s.esl3d_path = json_string(fop, "esl3d_path", "");
    s.output_path = json_string(fop, "output_path", "");
    if (fop.contains("cuda_echo") && fop["cuda_echo"].is_boolean())
        s.cuda_echo = fop["cuda_echo"].get<bool>();
    if (fop.contains("cpu_fft_echo") && fop["cpu_fft_echo"].is_boolean())
        s.cpu_fft_echo = fop["cpu_fft_echo"].get<bool>();

    // ---- derived geometry (SonarInit) ----
    s.dt = 1.0 / s.fs;
    s.lambda = s.c0 / s.fc;

    s.rx_xyz.resize(s.Nrx, 3);
    if (s.Nrx == 1) {
        s.rx_xyz(0, 0) = 0.0;
    } else {
        const double span = (s.Nrx - 1) * s.lambda / 2.0 * s.rx_interval_lambda;
        for (int i = 0; i < s.Nrx; ++i) {
            s.rx_xyz(i, 0) = -span + 2.0 * span * i / (s.Nrx - 1);
            s.rx_xyz(i, 1) = 0.0;
            s.rx_xyz(i, 2) = 0.0;
        }
    }

    s.tx_xyz.resize(s.Ntx, 3);
    if (s.Ntx == 1) {
        s.tx_xyz(0, 0) = 0.0;
    } else {
        const double span = (s.Ntx - 1) * s.lambda / 2.0 * s.tx_interval_lambda;
        for (int i = 0; i < s.Ntx; ++i) {
            s.tx_xyz(i, 0) = -span + 2.0 * span * i / (s.Ntx - 1);
            s.tx_xyz(i, 1) = 0.0;
            s.tx_xyz(i, 2) = 0.0;
        }
    }

    SONAR_LOG_INFO("Config loaded: tx_type=%s Nrx=%d Ntx=%d fs=%.0f fc=%.0f decim=%d",
                   s.tx_type.c_str(), s.Nrx, s.Ntx, s.fs, s.fc, s.decimation_factor);
    return s;
}

}  // namespace sonar::io

