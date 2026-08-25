# Offline Waveform and Image Reconstruction Pipeline

EchoVerse Sonar Lab contains a native C++ offline pipeline that converts a recorded
ESL3D point-cloud stream into multi-channel receive waveforms and reconstructed sonar
images. The implementation is linked into `echoverse_sonar_lab`; MATLAB, MATLAB
Runtime, `pointcloud2file.exe`, and `file2image.exe` are not runtime dependencies.

The product workflow is therefore self-contained: recording, waveform synthesis,
HDF5 serialization, and image reconstruction complete in native C++. The MATLAB
files under `src/matlab_point2file2image/` remain available as an additional research
post-analysis toolkit and reference implementation. They support algorithm
inspection, parameter studies, reproducibility checks, alternative plots, and
comparison with native output, but are not a runtime fallback, deployment
dependency, or required step in the product workflow. Production execution uses
`src/offline_processing/`.

## 1. How the Pipeline Is Started

The pipeline runs when all of the following are true for an FLS or MBES module:

1. the module and its point-cloud sonar are enabled;
2. ESL3D file output is enabled;
3. **Settings → Output → Generate raw waveform and reconstructed images** is enabled;
4. a recording session is stopped after at least one ESL3D frame is written.

`FlsModule::endOutputSession` or `MbesModule::endOutputSession` closes the ESL3D writer
before calling `runPointCloudPostProcess`. That function starts
`sonar::offline::process_esl3d_to_images` on a background thread and reports progress
through the Qt dialog. No child process is created.

The public native entry point is:

```cpp
sonar::offline::ProcessingOptions options;
options.esl3d_path = "recording/3d.esl3d";
options.sonar_config_path = "SonarParameter/Sonar.json";
options.output_directory = "recording/Waveform Data";

const auto result = sonar::offline::process_esl3d_to_images(options);
```

## 2. End-to-End Data Flow

```text
ESL3D packet stream
  -> range/intensity frames
  -> Cartesian scatterer point clouds
  -> transmit excitation + receive-array model
  -> channel echo synthesis
  -> Doppler, AWGN, downconversion, FIR, decimation
  -> HDF5 ping datasets
  -> matched filtering, TVG, delay-and-sum beamforming
  -> sector mapping and grayscale PNG rendering
```

The application processes one frame at a time. A waveform is written to HDF5 and its
image is reconstructed before moving to the next frame, so all pings do not have to be
held in memory simultaneously.

## 3. ESL3D Input and Point-Cloud Recovery

`core/io/esl3d_reader.cpp` reads the same little-endian packet structure documented in
[`sonar_esl3d_data_spec.md`](sonar_esl3d_data_spec.md):

- 56-byte fixed header;
- magic `0x5033534E`, version `1`;
- UTF-8 JSON metadata;
- `float32` range and intensity arrays.

For each valid finite range sample, the reader derives azimuth/elevation from the
recorded horizontal and vertical fields of view and computes:

```text
z = r * cos(elevation) * cos(azimuth)
x = r * cos(elevation) * sin(azimuth)
y = r * sin(elevation)
```

Intensity becomes the scatterer amplitude. The production default retains the complete
point cloud (`fraction = 1.0`); no implicit 30% decimation is applied. Validation runs
can cap frames, range, or scatterer count with the environment variables listed below.

## 4. Sonar Configuration and Excitation

`core/io/json_config.cpp` reads the selected `SonarParameter/*.json` file. LFM, CDM,
and FDM transmit modes are supported. `core/sim/excitation.cpp` builds the transmit
waveform, matched filters, array geometry, scan-angle divisions, and HDF5 attributes.

Important parameters include:

| JSON field | Purpose |
|---|---|
| `array_params.fs`, `fc`, `BW` | sample rate, center frequency, and bandwidth |
| `array_params.Nrx`, `Ntx` | receive/transmit element counts |
| `array_params.interp_factor` | excitation interpolation factor |
| `array_params.echo_oversample_factor` | internal echo-delay resolution; default `5` |
| `array_params.velocity` | radial velocity used by Doppler resampling |
| `array_params.snr_level` | measured-signal AWGN level in dB |
| `rx_signal_params.decimation_factor` | final baseband decimation |
| `rx_signal_params.angle_segments_deg` | beamforming scan sectors |
| `file_opt_params.*` | standalone/reference defaults only |

The built-in templates live under `src/offline_processing/config/` and are deployed
beside the executables for new-project creation. The GUI supplies the recorded ESL3D
path and session output directory through `ProcessingOptions`. These values override
`file_opt_params.esl3d_path` and
`file_opt_params.output_path` in memory. The JSON file is never rewritten.

## 5. Channel Echo Synthesis

`core/sim/echo_simulator.cpp` ports the point-scatterer receive model and the former
`EchoInit` stages:

1. compute transmit-to-scatterer and scatterer-to-receiver delays;
2. accumulate delayed, amplitude-weighted excitation for every receive channel;
3. resample back from the internal echo-oversampling rate;
4. apply Doppler resampling using the configured radial velocity;
5. inject measured-signal AWGN at `snr_level`;
6. mix to complex baseband;
7. apply the FIR low-pass filter and final decimation.

The embedded path is CPU-only and never invokes an external CUDA executable. OpenMP is
used when available. The optimized direct kernel fuses downsampling to reduce peak
memory. FFTW hooks inherited from the source port are compiled as fallbacks only;
the distributed EchoVerse build uses the direct CPU backend and does not link FFTW.

## 6. HDF5 Output

`core/io/hdf5_writer.cpp` writes the MATLAB-compatible baseline layout described in
[`sonar_h5_data_spec.md`](sonar_h5_data_spec.md):

```text
/raw_data/.attributes/...
/raw_data/ping_1/real
/raw_data/ping_1/imag
/raw_data/ping_2/real
/raw_data/ping_2/imag
...
```

Complex matrices use `real` and `imag` sub-datasets and a `complex=1` marker. Closing
the writer stores `ping_num`. The output filename is `<esl3d-stem>.h5`; an existing
file with that name in `Waveform Data` is replaced by the current run.

## 7. Image Reconstruction

Each just-generated ping passes through `core/image/file2image_pipeline.cpp`:

1. per-channel matched filtering with the receive-array window;
2. time-varying gain compensation;
3. inverse-domain delay-and-sum plane-wave beamforming;
4. sector coordinate mapping;
5. logarithmic compression to a 40 dB dynamic range by default;
6. 800 × 600, 8-bit grayscale PNG rendering by default.

Output names follow:

```text
<esl3d-stem>_<yyyyMMdd_HHmmss>_ping001.png
<esl3d-stem>_<yyyyMMdd_HHmmss>_ping002.png
...
```

Doppler and noise are introduced before the waveform is stored. Matched filtering,
TVG, and beamforming therefore operate on—and preserve the effects of—the modified
channel data rather than adding artificial effects after image formation.

## 8. Output Layout

For a module named `FLS 1`, a typical session contains:

```text
Sonar Data/<timestamp>/FLS 1/
  3d.esl3d
  Waveform Data/
    3d.h5
    3d_<timestamp>_ping001.png
    3d_<timestamp>_ping002.png
```

## 9. Diagnostics and Validation Controls

Normal GUI use requires no environment variables. The following controls are useful
for benchmarks and bounded validation runs:

| Variable | Effect |
|---|---|
| `ESL_OFFLINE_PERF_CSV` | write one timing row per processed frame |
| `ESL_MAX_FRAMES` | process at most this many ESL3D frames |
| `ESL_MAX_RANGE` | keep scatterers at or below this range in metres |
| `ESL_MAX_SCATTERERS` | keep at most this many scatterers per frame |
| `ESL_ECHO_OVERSAMPLE_FACTOR` | override internal echo oversampling |
| `ESL_ECHO_TILE_SAMPLES` | tune direct-kernel tile size |
| `ESL_ECHO_FUSED_DOWNSAMPLE` | enable/disable fused downsampling (`1`/`0`) |
| `ESL_ECHO_LEGACY_DIRECT` | select the retained validation kernel |

The test targets `offline_hdf5_test`, `offline_render_test`,
`offline_multisector_test`, and `offline_pipeline_test` verify HDF5 round trips,
grayscale PNG output, multi-sector beam reconstruction, and an end-to-end synthetic
ESL3D-to-HDF5/PNG run.

## 10. Runtime and Failure Behavior

The GUI remains responsive while processing and updates progress between synthesis and
image stages. Stopping the application or terminating the process still interrupts the
job; resumable HDF5 generation and user cancellation are not currently implemented.
Input/config errors and processing exceptions are logged and cause the post-processing
call to return `false` without launching a fallback executable.

In one sentence, the native pipeline is:

**ESL3D → scatterers → CPU channel simulation with Doppler/AWGN → HDF5 → matched filter/TVG/DAS → PNG.**
