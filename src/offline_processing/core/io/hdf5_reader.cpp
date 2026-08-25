#include "io/hdf5_reader.h"

#ifdef SONAR_HAVE_HDF5

#include <hdf5.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <stdexcept>

#include "util/log.h"

namespace sonar::io {

namespace {

[[noreturn]] void h5fail(const std::string& what) { throw std::runtime_error("HDF5: " + what); }

bool has_attr(hid_t obj, const char* name) {
    H5E_BEGIN_TRY { return H5Aexists(obj, name) > 0; }
    H5E_END_TRY;
    return false;
}

long long read_attr_int(hid_t obj, const char* name) {
    long long v = 0;
    H5E_BEGIN_TRY {
        hid_t a = H5Aopen(obj, name, H5P_DEFAULT);
        if (a < 0) return v;
        H5Aread(a, H5T_STD_I64LE, &v);
        H5Aclose(a);
    }
    H5E_END_TRY;
    return v;
}

bool is_group(hid_t loc, const std::string& name) {
    H5E_BEGIN_TRY {
        hid_t g = H5Gopen2(loc, name.c_str(), H5P_DEFAULT);
        if (g < 0) return false;
        H5Gclose(g);
        return true;
    }
    H5E_END_TRY;
    return false;
}

bool is_dataset(hid_t loc, const std::string& name) {
    H5E_BEGIN_TRY {
        hid_t d = H5Dopen2(loc, name.c_str(), H5P_DEFAULT);
        if (d < 0) return false;
        H5Dclose(d);
        return true;
    }
    H5E_END_TRY;
    return false;
}

// Read a dataset (or group with real/imag) into out. Returns true on success.
bool read_object_into(hid_t loc, const std::string& name, MatC& out) {
    // 1) group with real & imag datasets -> complex
    if (is_group(loc, name)) {
        hid_t g = H5Gopen2(loc, name.c_str(), H5P_DEFAULT);
        if (has_attr(g, "complex") && read_attr_int(g, "complex") == 1) {
            // complex marker: try real/imag children
            if (is_dataset(g, "real") && is_dataset(g, "imag")) {
                MatC re, im;
                if (read_object_into(g, "real", re) && read_object_into(g, "imag", im)) {
                    out = re;
                    for (size_t i = 0; i < out.size(); ++i)
                        out.data()[i] += cplx(0.0, im.data()[i].real());
                    H5Gclose(g);
                    return true;
                }
            }
        }
        // group without complex marker: check real/imag children directly
        if (is_dataset(g, "real") && is_dataset(g, "imag")) {
            MatC re, im;
            if (read_object_into(g, "real", re) && read_object_into(g, "imag", im)) {
                out = re;
                for (size_t i = 0; i < out.size(); ++i)
                    out.data()[i] += cplx(0.0, im.data()[i].real());
                H5Gclose(g);
                return true;
            }
        }
        H5Gclose(g);
        return false;
    }

    // 2) dataset
    hid_t d = H5Dopen2(loc, name.c_str(), H5P_DEFAULT);
    if (d < 0) return false;

    // complex attr == 1 on dataset path -> read real/imag children
    if (has_attr(d, "complex") && read_attr_int(d, "complex") == 1) {
        H5Dclose(d);
        if (is_group(loc, name)) {
            MatC re, im;
            if (read_object_into(loc, name + "/real", re) && read_object_into(loc, name + "/imag", im)) {
                out = re;
                for (size_t i = 0; i < out.size(); ++i)
                    out.data()[i] += cplx(0.0, im.data()[i].real());
                return true;
            }
        }
        return false;
    }

    hid_t space = H5Dget_space(d);
    const int rank = H5Sget_simple_extent_ndims(space);
    hsize_t dims[8] = {0};
    H5Sget_simple_extent_dims(space, dims, nullptr);
    // HDF5 stores flipped dims vs MATLAB: HDF5 (C, R) -> MATLAB [R, C].
    int R = 1, C = 1;
    if (rank == 1) {
        C = static_cast<int>(dims[0]);
    } else if (rank >= 2) {
        R = static_cast<int>(dims[rank - 1]);
        C = static_cast<int>(dims[rank - 2]);
    }
    const size_t n = static_cast<size_t>(R) * C;

    hid_t type = H5Dget_type(d);
    const H5T_class_t tclass = H5Tget_class(type);
    const size_t esize = H5Tget_size(type);

    if (tclass == H5T_COMPOUND) {
        H5Tclose(type);
        H5Sclose(space);
        H5Dclose(d);
        return false;  // unsupported for now
    }

    std::vector<double> buf(n, 0.0);
    if (tclass == H5T_FLOAT) {
        if (esize == 4) {
            std::vector<float> f(n);
            H5Dread(d, H5T_IEEE_F32LE, H5S_ALL, H5S_ALL, H5P_DEFAULT, f.data());
            for (size_t i = 0; i < n; ++i) buf[i] = static_cast<double>(f[i]);
        } else {
            H5Dread(d, H5T_IEEE_F64LE, H5S_ALL, H5S_ALL, H5P_DEFAULT, buf.data());
        }
    } else if (tclass == H5T_INTEGER) {
        std::vector<long long> iv(n);
        H5Dread(d, H5T_STD_I64LE, H5S_ALL, H5S_ALL, H5P_DEFAULT, iv.data());
        for (size_t i = 0; i < n; ++i) buf[i] = static_cast<double>(iv[i]);
    } else {
        H5Tclose(type);
        H5Sclose(space);
        H5Dclose(d);
        return false;
    }

    out.resize(R, C);
    for (size_t i = 0; i < n; ++i) out.data()[i] = cplx(buf[i], 0.0);

    H5Tclose(type);
    H5Sclose(space);
    H5Dclose(d);
    return true;
}

std::vector<std::string> list_members(hid_t loc) {
    std::vector<std::string> names;
    H5G_info_t ginfo{};
    if (H5Gget_info(loc, &ginfo) < 0) return names;
    for (hsize_t i = 0; i < ginfo.nlinks; ++i) {
        char name[1024] = {0};
        if (H5Lget_name_by_idx(loc, ".", H5_INDEX_NAME, H5_ITER_INC, i, name, sizeof(name), H5P_DEFAULT) >= 0) {
            names.emplace_back(name);
        }
    }
    return names;
}

void read_group_attributes(hid_t loc, Hdf5Data& out) {
    // numeric / string attributes at this group
    hsize_t idx = 0;
    while (H5Aiterate_by_name(loc, ".", H5_INDEX_NAME, H5_ITER_INC, &idx,
                              [](hid_t o, const char* aname, const H5A_info_t*, void* udata) -> herr_t {
        auto* data = static_cast<Hdf5Data*>(udata);
        hid_t a = H5Aopen(o, aname, H5P_DEFAULT);
        if (a < 0) return 0;
        hid_t type = H5Aget_type(a);
        hid_t space = H5Aget_space(a);
        const H5T_class_t tc = H5Tget_class(type);
        const size_t esize = H5Tget_size(type);
        const hssize_t nelmts = H5Sget_simple_extent_npoints(space);
        if (tc == H5T_STRING) {
            const hsize_t ssize = H5Tget_size(type);
            if (H5Tis_variable_str(type)) {
                // variable-length string
                hid_t mem = H5Tcopy(H5T_C_S1);
                H5Tset_size(mem, H5T_VARIABLE);
                H5Tset_cset(mem, H5Tget_cset(type));
                H5Tset_strpad(mem, H5T_STR_NULLTERM);
                char* sval = nullptr;
                if (H5Aread(a, mem, &sval) >= 0 && sval) {
                    data->attr_strings[aname] = sval;
                    H5free_memory(sval);
                }
                H5Tclose(mem);
            } else {
                // fixed-length string: read with the source type itself
                std::vector<char> buf(static_cast<size_t>(ssize) + 1, '\0');
                if (H5Aread(a, type, buf.data()) >= 0) {
                    data->attr_strings[aname] = std::string(buf.data());
                }
            }
        } else if (tc == H5T_FLOAT || tc == H5T_INTEGER) {
            if (nelmts == 1) {
                double v = 0.0;
                if (tc == H5T_FLOAT) {
                    if (esize <= 4) {
                        float f = 0.f;
                        H5Aread(a, H5T_IEEE_F32LE, &f);
                        v = f;
                    } else {
                        H5Aread(a, H5T_IEEE_F64LE, &v);
                    }
                } else {
                    long long iv = 0;
                    H5Aread(a, H5T_STD_I64LE, &iv);
                    v = static_cast<double>(iv);
                }
                data->attr_scalars[aname] = v;
            }
            // (non-scalar numeric attributes ignored here; datasets cover those)
        }
        H5Tclose(type);
        H5Sclose(space);
        H5Aclose(a);
        return 0;
    }, &out, H5P_DEFAULT) > 0) {
        // continue
    }
}

}  // namespace

bool read_hdf5_dataset(const std::string& file, const std::string& path, MatC& out) {
    H5E_BEGIN_TRY {
        hid_t f = H5Fopen(file.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
        if (f < 0) return false;
        const bool ok = read_object_into(f, path, out);
        H5Fclose(f);
        return ok;
    }
    H5E_END_TRY;
    return false;
}

Hdf5Data read_baseline_hdf5(const std::string& path) {    Hdf5Data out;
    hid_t file = H5Fopen(path.c_str(), H5F_ACC_RDONLY, H5P_DEFAULT);
    if (file < 0) h5fail("H5Fopen " + path);

    // ---- attributes group ----
    if (H5Lexists(file, "/raw_data/.attributes", H5P_DEFAULT) > 0) {
        hid_t ag = H5Gopen2(file, "/raw_data/.attributes", H5P_DEFAULT);
        read_group_attributes(ag, out);
        for (const auto& name : list_members(ag)) {
            MatC val;
            if (read_object_into(ag, name, val)) {
                out.attr_datasets[name] = val;
            }
        }
        H5Gclose(ag);
    }

    // ---- pings ----
    if (H5Lexists(file, "/raw_data", H5P_DEFAULT) > 0) {
        hid_t rg = H5Gopen2(file, "/raw_data", H5P_DEFAULT);
        std::vector<std::pair<int, std::string>> pings;
        for (const auto& name : list_members(rg)) {
            if (name.rfind("ping_", 0) == 0 && name.size() > 5) {
                bool all_digit = true;
                for (size_t k = 5; k < name.size(); ++k)
                    if (!std::isdigit(static_cast<unsigned char>(name[k]))) {
                        all_digit = false;
                        break;
                    }
                if (all_digit) {
                    int num = std::stoi(name.substr(5));
                    pings.emplace_back(num, name);
                }
            }
        }
        std::sort(pings.begin(), pings.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });
        for (const auto& [num, name] : pings) {
            MatC ping;
            if (read_object_into(rg, name, ping)) {
                out.pings.push_back(ping);
            } else {
                SONAR_LOG_WARN("failed to read ping %s", name.c_str());
            }
        }
        H5Gclose(rg);
    }

    H5Fclose(file);

    map_attributes_into(out.attributes, out.attr_scalars, out.attr_strings, out.attr_datasets);
    out.attributes.ping_num = static_cast<int>(out.pings.size());
    SONAR_LOG_INFO("Read %llu ping(s) from %s",
                   static_cast<unsigned long long>(out.pings.size()), path.c_str());
    return out;
}

void map_attributes_into(SonarAttributes& a, const std::map<std::string, double>& scalars,
                         const std::map<std::string, std::string>& strings,
                         const std::map<std::string, MatC>& datasets) {
    auto str = [&](const char* key, std::string& dst) {
        auto it = strings.find(key);
        if (it != strings.end()) dst = it->second;
    };
    auto sca = [&](const char* key, double& dst) {
        auto it = scalars.find(key);
        if (it != scalars.end()) dst = it->second;
    };
    auto ds = [&](const char* key, MatC& dst) {
        auto it = datasets.find(key);
        if (it != datasets.end()) dst = it->second;
    };

    str("array_type", a.array_type);
    str("signal_type", a.signal_type);
    str("signal_win", a.signal_win);

    // helper: prefer dataset, fall back to scalar (MATLAB stores scalars as attrs)
    auto ds_or_scalar = [&](const char* key, std::vector<double>& dst) {
        if (auto it = datasets.find(key); it != datasets.end()) {
            dst.resize(static_cast<size_t>(it->second.size()));
            for (size_t i = 0; i < dst.size(); ++i) dst[i] = it->second.data()[i].real();
        } else if (auto it = scalars.find(key); it != scalars.end()) {
            dst = {it->second};
        }
    };
    auto ds_or_scalar_mat = [&](const char* key, MatD& dst) {
        if (auto it = datasets.find(key); it != datasets.end()) {
            dst.resize(it->second.rows(), it->second.cols());
            for (size_t i = 0; i < it->second.size(); ++i)
                dst.data()[i] = it->second.data()[i].real();
        }
    };

    ds_or_scalar("bandwidth", a.bandwidth);
    sca("sampling_frequency", a.sampling_frequency);
    ds_or_scalar("center_frequency", a.center_frequency);
    if (auto it = scalars.find("decimate_factor"); it != scalars.end())
        a.decimate_factor = static_cast<int>(it->second);
    if (auto it = scalars.find("sector_num"); it != scalars.end())
        a.sector_num = static_cast<int>(it->second);
    ds("match_filter_data", a.match_filter_data);
    if (auto it = scalars.find("receive_array_num"); it != scalars.end())
        a.receive_array_num = static_cast<int>(it->second);
    ds_or_scalar_mat("receive_array_position", a.receive_array_position);
    ds_or_scalar_mat("receive_array_win", a.receive_array_win);
    sca("pulse_duration", a.pulse_duration);
    sca("sound_velocity", a.sound_velocity);
    sca("velocity", a.velocity);
    sca("snr_level", a.snr_level);
    str("timestamp", a.timestamp);
    ds_or_scalar("scan_angle", a.scan_angle);
    ds_or_scalar("sector_div", a.sector_div);
    ds_or_scalar("sample_delay", a.sample_delay);
}

}  // namespace sonar::io

#endif  // SONAR_HAVE_HDF5

