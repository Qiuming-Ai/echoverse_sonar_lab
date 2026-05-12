# Offline Waveform and Image Reconstruction Pipeline

Based on the existing MATLAB code in `src/matlab_point2file2image`, this document explains how the offline pipeline:

1. reads `esl3d`,
2. generates an `hdf5` waveform file,
3. reconstructs a sonar image that includes Doppler and noise effects.

---

## 1. End-to-End Flow (Two Stages)

The offline pipeline has two entry functions:

- `pointcloud2file.m`: `esl3d -> echo simulation -> h5`
- `file2image.m`: `h5 -> matched filtering / TVG / beamforming -> sector image`

Recommended call order:

1. Run `pointcloud2file(sonar_json_path)` first to generate `.h5`.
2. Run `file2image(h5_path)` next to generate an image (GIF).

---

## 2. Stage 1: How ESL3D Is Read

### 2.1 Entry and Initialization

Main flow in `pointcloud2file.m`:

- `EnvInit`: adds TX/RX/Beamformer/ESL3D and related paths.
- `SonarInit`: reads `Sonar.json`, initializes arrays, transmit signals, filters, matched filters, etc.
- `DataMakerInit`: creates `SonarDataMaker` and initializes output HDF5.
- `SceneInit(sonar.esl3d_path)`: loads `.esl3d`.

### 2.2 Validation in SceneInit

`Initialize/SceneInit.m` performs two checks:

- file existence check (`isfile`),
- extension must be `.esl3d`.

If both pass, it calls the `esl3d(esl3dPath)` constructor for actual parsing.

### 2.3 ESL3D Binary Read Details

In `ESL3D/esl3d.m`, `readEsl3dPackets` reads frame by frame:

1. Read a fixed `56`-byte header first (little-endian, format consistent with `Sonar Point Cloud TCP Protocol`).
2. Parse fields (key items):
   - `magic = 0x5033534E`
   - `version = 1`
   - `seq`, `ts_us`
   - `width`, `height`, `point_count`
   - `metadata_bytes`, `range_bytes`, `intensity_bytes`, `payload_bytes`
3. Validate:
   - `magic/version/header_bytes` are correct,
   - `payload_bytes == metadata + range + intensity`.
4. Split payload by lengths:
   - metadata (UTF-8 JSON),
   - range (`float32`),
   - intensity (`float32`).
5. `reshape` to 2D image (`height x width`).

Each frame is stored with:

- `range`, `intensity`
- `metadata`
- `sonar_cfg` (from `metadata.sonar_config`)
- `env` (from `metadata.environment`)
- `pose` (from `metadata.pose`)

### 2.4 From Range/Intensity to Point Cloud

`getPointCloud(i)` internally calls `rangeToPointCloud`:

- Read `horizontal_fov_deg` / `vertical_fov_deg` from `sonar_cfg`.
- Build azimuth and elevation grids.
- Keep valid points where `range > 0` and finite.
- Map spherical to Cartesian coordinates:
  - `z = r * cos(el) * cos(az)`
  - `x = r * cos(el) * sin(az)`
  - `y = r * sin(el)`
- Intensity comes from `intensity(valid)`.

In `pointcloud2file.m`, each frame also performs:

- `PointCloudDecimate(..., 0.3)` to downsample the point cloud (reduce downstream simulation load).

---

## 3. Stage 1: How HDF5 Is Generated

### 3.1 Per-Frame Echo Simulation (`EchoInit`)

`EchoInit.m` outputs `echo.y_deci` (downsampled complex baseband channel signal). Key steps:

1. **Scattered echo synthesis**  
   Calls `sim_rx_from_scatterers_perTX_cuda(...)` (when NVIDIA GPU is available) or `sim_rx_from_scatterers_perTX(...)` (non-CUDA path; this `.m` file is not visible in the current repo, which usually implies external dependency or implementation elsewhere).

2. **Propagation-delay zero padding**  
   Computes `delay_samples` using `t` and pads zeros at the front.

3. **Normalization**  
   Normalizes waveform to approximately `[-1,1]`.

4. **Interpolation rollback**  
   Uses `resample(v,1,interp_factor)` to return to the target sampling-rate scale.

5. **Doppler injection (core)**  
   ```text
   v_calc = resample(v, c0 + 2*velocity, c0)
   ```
   This step modifies the time axis through resampling, equivalent to frequency/time stretching based on radial velocity (approximate Doppler processing). `velocity` comes from `array_params.velocity` in `Sonar.json`.

6. **Noise injection (core)**  
   ```text
   v_calc = awgn(v_calc, snr_level, 'measured')
   ```
   Adds AWGN with the configured `snr_level`.

7. **Downconversion**  
   Mixes with `exp(j*2*pi*Subfc*t)` to get complex baseband.

8. **FIR low-pass + decimation**  
   Runs `SignalFilter.process`, then decimates by `decimation_factor` to produce `y_deci`.

### 3.2 HDF5 Writer (`SonarDataMaker`)

`DataMakerInit.m` creates HDF5 through `SonarDataMaker` and writes two categories:

- Metadata: `/raw_data/.attributes/...`
- Per-frame data: `/raw_data/ping_1`, `/raw_data/ping_2`, ...

Filename policy:

- Output to the same directory as `esl3d` by default.
- Filename is `<esl3d_filename>.h5`.
- If `Sonar.json:file_opt_params.output_path` is non-empty, output there instead.

### 3.3 Internal HDF5 Organization

`SonarDataMaker.writeNumericDatasetWithComplexSupport` stores complex values via `real/imag` sub-datasets:

- Real: writes one dataset directly and marks attribute `complex=0`.
- Complex: writes
  - `<path>/real`
  - `<path>/imag`
  and marks `complex=1` on `<path>`.

Therefore, `echo.y_deci` (usually complex) is written as `real/imag` under each `ping_i`.

On `close()`, it also writes:

- `ping_num = total frame count` under `/raw_data/.attributes`.

---

## 4. Stage 2: How Images Are Generated from HDF5

### 4.1 Read HDF5

`file2image.m` first calls `ReadBaselineHDF5(filePath)`:

- Automatically reads `/raw_data/.attributes` (array parameters, matched filter, scan angle, sampling rate, etc.).
- Automatically reads all `ping_i` datasets (supports real values or `real/imag` complex reconstruction).

### 4.2 Main Reconstruction Chain (Per Ping)

For each ping (multi-channel), it performs:

1. **Matched filtering**  
   Applies `conv(ping, match_filter_data, 'same')` on each receive channel, then multiplies by the element window (default: Hamming).

2. **TVG (time-varying gain)**  
   Uses `tvg_factory(..., K=20)` to compute gain curve `G_amp(tau)` and compensates range attenuation channel by channel.

3. **Beamforming**  
   Runs `das_plane_wave_id(cfg, sig_mf)` to obtain `beam` and `ranges_out`.

4. **Sector mapping and dB dynamic-range compression**  
   Calls `sector_plot(cfg.angles, ranges_out, abs(beam), 40, "sector", false)`.

5. **Plot output**  
   `surf + view(2) + colormap(jet) + caxis([-dynRange,0])`；
   Each frame is written to GIF, producing a dynamic sonar image sequence.

---

## 5. How "Images with Doppler and Noise" Are Formed

Key point: Doppler and noise are not post-added in `file2image.m`; they are already injected along **`EchoInit -> y_deci -> HDF5`**.

- Doppler: frequency shift/time stretch from `resample(v, c0+2*velocity, c0)`.
- Noise: injected by `awgn(..., snr_level, 'measured')`.

Then `file2image` applies matched filtering, TVG, and beamforming to these channels that already include Doppler and noise, so the final image naturally carries both effects.

---

## 6. Minimal Running Example

```matlab
% 1) Generate h5 from esl3d (echoes include Doppler + noise)
pointcloud2file("./SonarParameter/Sonar.json");

% 2) Reconstruct image from h5 (output gif)
file2image("./Sonar Data/default.h5");
```

Recommended minimum configuration in `Sonar.json`:

- `file_opt_params.esl3d_path`
- `file_opt_params.output_path`
- `array_params.velocity` (controls Doppler)
- `array_params.snr_level` (controls noise intensity)

---

## 7. One-Sentence Summary

This MATLAB offline pipeline is essentially:  
**`ESL3D frame data -> geometric point cloud -> channel echo simulation (with Doppler + AWGN) -> HDF5 archival -> matched filtering / TVG / DAS -> sector sonar image`**.
