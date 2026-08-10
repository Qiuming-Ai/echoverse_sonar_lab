# EchoVerse Sonar Lab

EchoVerse Sonar Lab is an open-source multimodal sonar simulation, visualization,
streaming, and offline signal-processing environment. It supports forward-looking
sonar (FLS), multibeam echo sounder (MBES), and side-scan sonar (SSS) workflows in a
shared 3D scene.

## What Each Runtime Does

The software uses two connected, but technically distinct, execution paths:

| Runtime | Responsibility | Main outputs |
|---|---|---|
| C++ online runtime | Loads the scene, renders real-time sonar echo/intensity images, recovers echo point clouds, displays the GUI, streams data, and records packetized frames | Real-time FLS/MBES/SSS images, `.esl2d`, `.esl3d`, TCP streams |
| MATLAB offline runtime | Reads C++-generated `.esl3d` point-cloud frames, synthesizes received channel echoes, applies signal processing, and reconstructs the final waveform-domain sonar image | HDF5 channel waveforms, processed echo data, reconstructed sector images/GIFs |

The C++ output is therefore the geometric/intensity-domain input to the MATLAB
signal-level pipeline. The C++ runtime does not generate the final multi-channel raw
waveform used by the MATLAB beamforming path.

## Features

- Interactive shared scenes for FLS, MBES, and SSS modules
- Real-time C++ echo/intensity image generation
- C++ polar range/intensity frame and point-cloud recovery
- Session-scoped `.esl2d` / `.esl3d` recording and TCP streaming
- MATLAB echo synthesis, Doppler/noise processing, HDF5 export, and image reconstruction
- Optional NVIDIA CUDA MEX acceleration for MATLAB echo synthesis
- Opt-in C++ and MATLAB performance CSV logging for scale experiments
- Windows and Linux GUI support; Linux uses an offscreen main-camera path compatible with X11 and Wayland

## Repository Layout

- `src/`: C++ runtime and MATLAB offline pipeline
- `docs/`: architecture, formats, build, performance, and development documentation
- `uwmodels/`: example underwater models and scenes
- `CMakeLists.txt`: top-level CMake configuration
- `vcpkg.json`: Qt/OpenCV/OpenSceneGraph manifest for vcpkg builds
- `third_party/eigen`: Eigen Git submodule pinned by this repository

## Supported and Tested Environments

| Platform | Status | Tested environment |
|---|---|---|
| Windows | Supported | Windows 11, Visual Studio 2022, vcpkg |
| Linux | Supported | Ubuntu 24.04, GCC 13.3, Qt 6.10.2, OpenCV 4.12.0 with Qt 6, OpenSceneGraph 3.6.5 |
| Arch Linux | Expected to work with compatible dependency versions, but not yet independently verified | Clean-clone verification remains part of the revision checklist |

The Linux GUI implementation does not share a native X11 window with Qt. The main
OSG camera renders offscreen and is displayed in a Qt widget, which also works on
Wayland.

## Clone With Eigen

Eigen is a Git submodule rather than a copied dependency directory. Clone with:

```bash
git clone --recurse-submodules https://github.com/Qiuming-Ai/echoverse_sonar_lab.git
cd echoverse_sonar_lab
```

For an existing clone:

```bash
git submodule update --init --recursive
```

The pinned Eigen revision provides APIs used by this project that are not available
in Eigen 3.4. A different compatible Eigen 5 checkout can be supplied with
`-DEIGEN3_INCLUDE_DIR=/path/to/eigen`.

## Build Requirements

- CMake 3.16 or newer; CMake 3.30+ is recommended for the tested Linux Qt toolchain
- C++17 compiler
- Eigen 5 from the pinned submodule
- Qt Widgets and Network; Qt 6.9+ is recommended
- OpenCV core, imgproc, imgcodecs, videoio, and highgui
- OpenSceneGraph and OpenThreads 3.6.x
- MATLAB for the offline waveform/image pipeline
- NVIDIA CUDA Toolkit plus a supported host compiler only when CUDA MEX acceleration is required

## Quick Start: Windows

Use a normal vcpkg installation outside or inside the ignored `third_party` working
directory. In PowerShell:

```powershell
git submodule update --init --recursive

$env:VCPKG_ROOT = "C:\path\to\vcpkg"
& "$env:VCPKG_ROOT\vcpkg.exe" install --triplet x64-windows

cmake -S . -B build_vcpkg -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" `
  -DVCPKG_TARGET_TRIPLET=x64-windows
cmake --build build_vcpkg --config Release
```

Start `build_vcpkg\Release\esl_launcher.exe`. The launcher supplies the required
project path to `echoverse_sonar_lab.exe`.

## Quick Start: Linux

The tested Ubuntu build uses Qt 6.10.2 and an OpenCV 4.12.0 build linked only to Qt 6.
This matters because some distribution OpenCV `highgui` packages pull Qt 5 into a
Qt 6 application and can crash before `main()`.

```bash
git submodule update --init --recursive

cmake -S . -B build_linux \
  -DCMAKE_BUILD_TYPE=Release \
  -DQt6_DIR="$HOME/Qt/6.10.2/gcc_64/lib/cmake/Qt6" \
  -DOpenCV_DIR="$HOME/opencv-4.12.0/lib/cmake/opencv4"
cmake --build build_linux -j2
./build_linux/esl_launcher
```

See [`docs/linux_build.md`](docs/linux_build.md) for the tested Ubuntu dependency
setup, Qt/OpenCV compatibility checks, virtual-display smoke tests, and troubleshooting.

## MATLAB Offline Echo and Image Pipeline

The recommended order is:

```matlab
pointcloud2file("./SonarParameter/Sonar.json");  % ESL3D -> channel echoes -> HDF5
file2image("./Sonar Data/default.h5");          % HDF5 -> processing -> sonar image
```

CPU echo synthesis uses
`src/matlab_point2file2image/RXSignalGen/sim_rx_from_scatterers_perTX.m`.

### Compile the Optional CUDA MEX Backend

Having an NVIDIA GPU is not sufficient by itself. Before enabling the GPU path,
compile the CUDA source from MATLAB:

```matlab
cd("src/matlab_point2file2image/RXSignalGen");
mexcuda -setup C++;
mexcuda -R2018a NVCCFLAGS='-allow-unsupported-compiler' ...
    sim_rx_from_scatterers_perTX_cuda_mex.cu;
```

Verify that `sim_rx_from_scatterers_perTX_cuda_mex` is discoverable on the MATLAB
path before running `pointcloud2file`. `EchoInit.m` selects the CUDA MEX path when a
supported NVIDIA GPU is detected; otherwise it uses the MATLAB CPU implementation.
Detailed requirements and failure modes are documented in
[`docs/Offline Waveform and Image Reconstruction Pipeline.md`](docs/Offline%20Waveform%20and%20Image%20Reconstruction%20Pipeline.md).

## Performance and Scale Logging

C++ profiling is opt-in through `ESL_CPP_PERF_CSV`:

```bash
ESL_CPP_PERF_CSV=results/cpp_performance.csv \
ESL_PERF_RUN_LABEL=shipwreck_100_pings \
./build_linux/esl_launcher
```

MATLAB profiling is also opt-in:

```matlab
setenv('ESL_MATLAB_PERF_CSV', fullfile(pwd, 'results', 'matlab_performance.csv'));
pointcloud2file("./SonarParameter/Sonar.json");
```

The logs contain per-frame/per-ping execution time, scene inventory, beam/bin size,
point counts, output byte estimates, waveform dimensions, and CPU/CUDA backend labels.
See [`docs/performance_and_scalability.md`](docs/performance_and_scalability.md) for
the complete experiment protocol and current scalability limitations.

A three-repeat Windows characterization of the prepared pipeline-inspection and coral
projects is preserved in
[`benchmarks/results/2026-08-10`](benchmarks/results/2026-08-10/README.md), together
with six raw CSV files and a PowerShell summary script. The tested scenes contained
52,096 and 591,116 estimated loaded triangles, respectively. Both sustained the
configured 5 fps GUI-loop cap with mean effective rates of 4.78 and 4.75 fps. These
results cover the C++ runtime with file/TCP output disabled; they are not MATLAB,
CUDA, uncapped-throughput, or universal scene-limit claims.

## Output Session Layout

When file or TCP output is enabled, the application creates:

- `Sonar Data/<timestamp>/<module_name>/2d.esl2d`
- `Sonar Data/<timestamp>/<module_name>/3d.esl3d`
- `Sonar Data/<timestamp>/<module_name>/Waveform Data/`
- `Sonar Data/<timestamp>/recording_summary.json`

## Documentation

- Overall architecture: [`docs/software_architecture_analysis.md`](docs/software_architecture_analysis.md)
- C++ acoustic core: [`docs/acoustic_simulation_core_overview.md`](docs/acoustic_simulation_core_overview.md)
- MATLAB offline pipeline: [`docs/Offline Waveform and Image Reconstruction Pipeline.md`](docs/Offline%20Waveform%20and%20Image%20Reconstruction%20Pipeline.md)
- Linux build: [`docs/linux_build.md`](docs/linux_build.md)
- Performance/scalability: [`docs/performance_and_scalability.md`](docs/performance_and_scalability.md)
- AI-assisted development disclosure: [`docs/ai_assisted_development.md`](docs/ai_assisted_development.md)
- TCP protocol: [`docs/sonar_tcp_protocol.md`](docs/sonar_tcp_protocol.md)
- ESL2D format: [`docs/sonar_esl2d_data_spec.md`](docs/sonar_esl2d_data_spec.md)
- ESL3D format: [`docs/sonar_esl3d_data_spec.md`](docs/sonar_esl3d_data_spec.md)
- HDF5 format: [`docs/sonar_h5_data_spec.md`](docs/sonar_h5_data_spec.md)

## AI-Assisted Development Disclosure

Generative-AI assistance was used during selected development stages. Early GUI and
UX code drafting used DeepSeek V4 Pro Token together with VS Code Copilot. The core
C++ sonar-image generation functionality and the MATLAB waveform-generation
functionality were designed and implemented by the author. OpenAI Codex was later
used for cross-platform integration, performance instrumentation, documentation, and
revision organization.

Human maintainers remain responsible for scientific and architectural decisions,
source review, builds, tests, numerical interpretation, licensing, and release
approval. No performance result is treated as valid until it has been produced by an
executed experiment and checked by an author. The detailed, versioned disclosure and
verification procedure are in
[`docs/ai_assisted_development.md`](docs/ai_assisted_development.md).

## License

The software code is released under the Apache License 2.0. See `LICENSE` and
`NOTICE`. Third-party models, meshes, textures, world files, and other assets may
have separate upstream terms and are not automatically covered by the code license.
