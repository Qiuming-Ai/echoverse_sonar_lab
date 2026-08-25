# Sonar H5 Data Specification

## 1. Purpose and Implementation

The native offline pipeline writes sonar waveform files through
`src/offline_processing/core/io/hdf5_writer.cpp`. The layout remains compatible with
the original MATLAB `DataMakerInit.m` / `SonarDataMaker` format so existing readers
can consume newly generated files. Native C++ generation and reconstruction complete
the product workflow without MATLAB; the retained MATLAB readers and analysis scripts
are optional research post-analysis tools for parameter studies, reproducibility
checks, alternative plots, and native/MATLAB result comparison.

`OfflineProcessingPipeline` builds `SonarAttributes` and calls:

1. `Hdf5Writer::start(hdf5_path, attributes)`;
2. `Hdf5Writer::write(waveform)` once per ping;
3. `Hdf5Writer::close()` to finalize `ping_num`.

## 2. Required Inputs and Naming

The native entry point requires an existing ESL3D file, an existing sonar JSON file,
a non-empty writable output directory, and at least one ESL3D frame. The JSON loader
derives the excitation, matched filter, array geometry, and HDF5 attributes.

The output filename is `<esl3d-stem>.h5` inside
`ProcessingOptions::output_directory`. The GUI supplies the module's `Waveform Data`
directory. JSON `file_opt_params.esl3d_path` and `output_path` are overridden only in
memory; the configuration file is never rewritten.

Example:

```text
input:  D:/proj/Sonar Data/20260825_120000/FLS 1/3d.esl3d
output: D:/proj/Sonar Data/20260825_120000/FLS 1/Waveform Data/3d.h5
```

Starting a writer uses HDF5 truncate semantics, so an existing file at the same output
path is replaced.

## 3. Root Layout

```text
/raw_data
  /.attributes
  /ping_1
    /real
    /imag
  /ping_2
    /real
    /imag
  ...
```

Complex values are represented by `real` and `imag` sub-datasets and a `complex=1`
attribute on their parent. Real values use one dataset and `complex=0`. Matrices are
column-major in memory; HDF5 dimensions are reversed to preserve MATLAB shape
semantics without transposing the payload.

## 4. Transmission Mode Mapping

`tx_signal_params.tx_type` controls `array_type`:

- `cdm` → `CDM`;
- `fdm` → `FDM`;
- all other supported modes → `Baseline`.

LFM, CDM, and FDM excitation and matched-filter generation are implemented natively.

## 5. Receive Window Policy

`rx_signal_params.array_window` selects the receive-element window:

- `hamming` → Hamming window;
- `hann` → Hann window;
- `blackman` → Blackman window;
- unknown names → all ones.

The default is Hamming. The generated window is stored in `receive_array_win`.

## 6. Sector, Frequency, and Bandwidth Rules

The JSON loader derives one scan-angle vector per requested sector from
`angle_segments_deg` plus `angle_step_deg`, or reads `angles_div_deg` directly.
`scan_angle` is the concatenation of all sectors. `sector_div` contains the first
sector start followed by every sector end.

Center frequency uses the per-sector `Subfc` vector when it has multiple values;
otherwise it uses `array_params.fc`. Bandwidth follows the same rule with `SubBW` and
`array_params.BW`.

## 7. Metadata Contract

The following values are stored under `/raw_data/.attributes` as scalar attributes or
datasets as appropriate:

- `array_type`;
- `signal_type` (`Baseband`);
- `signal_win`;
- `bandwidth`;
- `sampling_frequency`;
- `center_frequency`;
- `decimate_factor`;
- `sector_num`;
- `match_filter_data`;
- `receive_array_num`;
- `receive_array_position`;
- `receive_array_win`;
- `pulse_duration`;
- `sound_velocity`;
- `velocity`;
- `snr_level`;
- `timestamp` (`yyyyMMdd_HHmmss`);
- `scan_angle`;
- `sector_div` when available;
- `sample_delay`;
- `ping_num`, written on close.

## 8. Ping Data

Each `/raw_data/ping_N` entry is a complex baseband matrix with shape
`samples × receive_channels`. Values already contain the configured propagation,
Doppler, AWGN, downconversion, FIR, and decimation effects. Image reconstruction reads
the same data model, then applies matched filtering, TVG, and beamforming.

## 9. Verification

`offline_hdf5_test` writes a synthetic ping, reads it through
`core/io/hdf5_reader.cpp`, and verifies dimensions, complex values, and attributes.
The reader also supports MATLAB-produced files that follow the same contract.
