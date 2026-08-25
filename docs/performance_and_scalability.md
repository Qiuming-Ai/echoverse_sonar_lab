# Performance and Scalability Measurement

## 1. Purpose

The performance instrumentation is intended to support a reproducible manuscript
section on simulation size, runtime, and current limitations. It reports measurements;
it does not make an unverified real-time or HPC claim.

Profiling is disabled by default.

## 2. C++ Performance CSV

Set `ESL_CPP_PERF_CSV` before starting the application.

Linux:

```bash
ESL_CPP_PERF_CSV=results/cpp_shipwreck.csv \
ESL_PERF_RUN_LABEL=shipwreck_100_pings \
ESL_PERF_SAMPLE_STRIDE=1 \
./build_linux/esl_launcher
```

Windows PowerShell:

```powershell
$env:ESL_CPP_PERF_CSV = "results\cpp_shipwreck.csv"
$env:ESL_PERF_RUN_LABEL = "shipwreck_100_pings"
$env:ESL_PERF_SAMPLE_STRIDE = "1"
build_vcpkg\Release\esl_launcher.exe
```

`ESL_PERF_SAMPLE_STRIDE=N` records one of every `N` profiler samples. Use `1` for
paper measurements.

### Recorded C++ components

- `scene_inventory`: nodes, drawables, vertices, and estimated triangles
- `main_viewer_frame`: OSG main-view rendering time
- `fls_realtime_ping`: complete FLS real-time intensity-ping path
- `mbes_realtime_ping`: complete MBES real-time intensity-ping path
- `sss_realtime_ping_pair`: paired port/starboard SSS ping path
- `fls_point_cloud`: FLS point-cloud simulation, display, and output path
- `mbes_point_cloud`: MBES point-cloud simulation, display, and output path

The CSV includes duration, input/output point counts, output-byte estimates, beam/bin
counts, polar width/height, range, platform, hardware-thread count, and a user-supplied
run label.

## 3. Native Offline Performance CSV

Set `ESL_OFFLINE_PERF_CSV` before starting the application, then enable ESL3D file
output and native waveform/image generation for the module under test.

```powershell
$env:ESL_OFFLINE_PERF_CSV = "results\offline_shipwreck.csv"
build_vcpkg\Release\esl_launcher.exe
```

Each row reports:

- ESL3D point-cloud read time;
- point-cloud decimation time;
- channel echo-synthesis time;
- HDF5 write time;
- total per-ping time;
- input and retained scatterer counts;
- output sample/channel dimensions;
- the current embedded backend label, `cpp_cpu_direct`.

Unset the variable to disable logging:

```powershell
Remove-Item Env:ESL_OFFLINE_PERF_CSV
```

`ESL_MAX_FRAMES`, `ESL_MAX_RANGE`, and `ESL_MAX_SCATTERERS` can bound validation
runs. Record those values with every result because they directly change workload.

## 4. Completed Windows Characterization

On 2026-08-10, the prepared pipeline-inspection and coral/underwater projects were
each executed three times for 100 measured GUI-loop frames on Windows 11. Every
process returned code 0 and wrote a complete C++ profiler CSV. The original project
files were not modified. File and TCP outputs were disabled in temporary benchmark
copies so that the reported component times characterize the C++ generation and
display path rather than storage or network throughput.

| Scene | Loaded vertices | Estimated triangles | Mean effective loop rate |
|---|---:|---:|---:|
| Pipeline inspection | 136,994 | 52,096 | 4.78 fps |
| Coral/underwater | 331,321 | 591,116 | 4.75 fps |

The main loop was configured for 5 fps. The result therefore shows that both projects
sustained the configured cap; it is not an uncapped maximum-throughput result. The
projects also use different sensor grids, so their component times must not be used
as an isolated estimate of triangle-count scaling.

Both projects exceeded the geometry-texture warning threshold of 32,768 triangle
entries. The geometry-texture path reported 52,096 entries for the pipeline project
and 591,350 for the coral project; OpenSceneGraph may rescale that texture and reduce
sonar-image accuracy. The completed runs are performance evidence, not evidence that
image fidelity is preserved above the threshold.

Generated result tables, raw CSV files, execution logs, and benchmark records are kept
outside the source repository in the
[performance and benchmark data archive](https://drive.google.com/drive/folders/1FLh2osev_QVqSBR7Gu0UJmejG-zf4_zh?usp=drive_link).
The current measurements do not cover native offline execution, file/TCP I/O,
profiler overhead, Linux performance, or a failure threshold for larger scenes.

## 5. Recommended Experiment Matrix

Use at least three scene/trajectory scales and keep the sensor configuration fixed
within each cross-scene comparison.

| Scale | Suggested scene | Pings | Purpose |
|---|---|---:|---|
| Small | Simple target/seafloor | 100 | Baseline overhead and warm-up behavior |
| Medium | Multiple objects/terrain | 100–500 | Typical interactive workload |
| Large | Shipwreck or dense asset scene | 500–1000 | Stress test for dataset generation |

For every run, record:

- exact Git commit and Eigen submodule commit;
- OS, compiler, CMake, Qt, OpenCV, OSG, and HDF5 versions;
- CPU model/core count, RAM, GPU model/VRAM, and OpenMP availability;
- scene inventory from `scene_inventory`;
- range, FOV, angular resolution, beam count, bin count, and point budget;
- enabled attenuation, reverb, speckle, file, TCP, and visualization options;
- warm-up policy and number of measured pings;
- output directory size after the run.

Run each condition at least three times. Exclude a documented warm-up interval, then
report median and interquartile range or mean, standard deviation, and p95. Report
online and native-offline timings separately because they represent different stages.

## 6. Current Implementation Limits

- The main OSG viewer is intentionally single-threaded for rendering stability.
- The native offline loop is sequential across ESL3D frames; OpenMP parallelizes
  receive-channel and selected DSP work inside a frame when available.
- The embedded offline path is CPU-only and never launches the source port's optional
  standalone CUDA accelerator.
- The point-cloud UI constrains the configured maximum point count to 500,000.
- Point-cloud capture height is capped at 2,048 pixels.
- General offscreen render textures are capped at 4,096 pixels per dimension.
- The scene-depth triangle texture path warns when a texture-width limit of 32,768 is
  exceeded.
- Native processing keeps 100% of reconstructed scatterers by default. Validation
  caps configured through `ESL_MAX_RANGE` or `ESL_MAX_SCATTERERS` must be reported.
- Output size grows with ping count, polar frame dimensions, and waveform dimensions.

These are implementation limits, not measured performance results. The manuscript
should cite measurements produced on declared hardware.

## 7. HPC and Parallelization Status

The current release has no built-in MPI, Slurm, or distributed execution layer.
Independent scenes, paths, or ESL3D recordings can be launched as separate processes
by an external scheduler when each job has an isolated output directory. That is an
embarrassingly parallel experiment strategy, but it has not yet been packaged as a
supported cluster workflow.

Do not describe the current GUI or native offline frame loop as HPC-parallel. A future release
could add headless batch mode, deterministic partitioning, scheduler templates, and
result merging before claiming native HPC support.

## 8. Reporting and Interpretation Rules

- Preserve raw CSV files in the external revision-data archive.
- Derive reported table cells from those CSV files with the accompanying summary
  script or notebook.
- Check run completeness before using timing values from execution logs.
- Report renderer, point-cloud, native echo-synthesis, and HDF5/image stages
  separately.
- State the OpenMP status, internal oversampling factor, direct-kernel tile size, and
  fused-downsampling setting.
- Report failed or memory-limited large runs as limits rather than silently excluding them.
