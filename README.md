# EchoVerse Sonar Lab

EchoVerse Sonar Lab is a standalone multibeam sonar simulation and visualization project.
It provides:

- GUI application (`multibeam_gui`) for scene editing and sonar visualization
- Launcher (`esl_launcher`) for startup and configuration
- Unified output sessions for TCP streaming and `.esl2d` / `.esl3d` recording

## Features

- Configurable sonar simulation (FLS / MBES / SSS related modules)
- Session-scoped output management (per-module output directories + summary JSON)
- TCP streaming protocol for ESL2D/ESL3D packets
- `.esl2d` and `.esl3d` packet-based binary storage formats
- Optional "green package" ZIP output for runtime distribution

## Repository Layout

- `src/`: main source code
- `docs/`: protocol and file format documents
- `CMakeLists.txt`: top-level build configuration
- `vcpkg.json`: dependency manifest

## Build Requirements

- CMake >= 3.16
- C++17 compiler
- vcpkg (recommended on Windows)

## vcpkg Packages (Before CMake)

Install these packages (from `vcpkg.json`) before running CMake:

- `qtbase`
- `opencv4`
- `osg[collada,plugins]`

Example installation command:

```powershell
vcpkg install --triplet x64-windows qtbase opencv4 osg[collada,plugins]
```

If your vcpkg root is not in `PATH`, use the full path to `vcpkg.exe`.

## PowerShell Version

- Recommended: PowerShell 7.x (`pwsh`)
- Supported: Windows PowerShell 5.1 also works for the commands in this README
- No special PowerShell version is strictly required by this project; CMake and MSVC toolchain availability are the key requirements

## Third-Party Software Versions

| Component | Version | Source |
|-----------|---------|--------|
| Eigen | 5.0.1-dev+master | Bundled header-only library in `third_party/eigen` |
| Qt (`qtbase`) | 6.10.2 | vcpkg (`vcpkg.json`) |
| OpenCV (`opencv4`) | 4.12.0 | vcpkg (`vcpkg.json`) |
| OpenSceneGraph (`osg`) | 3.6.5 (`collada`, `plugins`) | vcpkg (`vcpkg.json`) |

Upstream links:

- [Eigen](https://gitlab.com/libeigen/eigen)
- [OpenCV](https://github.com/opencv/opencv)
- [OpenSceneGraph](https://github.com/openscenegraph/OpenSceneGraph)
- [Qt](https://github.com/qt/qtbase)
- [vcpkg](https://github.com/microsoft/vcpkg)

Versions above reflect the bundled Eigen snapshot and the vcpkg baseline used by `third_party/vcpkg`. Exact resolved versions may vary slightly if you use an external vcpkg root; check `build_vcpkg/vcpkg-manifest-install.log` after configure.

## Quick Start (Windows PowerShell, relative paths)

```powershell
# Recommended: configure with vcpkg toolchain (runtime DLL deployment works out-of-box)
cmake -S . -B build_vcpkg -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE="./third_party/vcpkg/scripts/buildsystems/vcpkg.cmake" `
  -DVCPKG_TARGET_TRIPLET=x64-windows
cmake --build build_vcpkg --config Release
```

Common executables after build:

- `build_vcpkg\Release\esl_launcher.exe` — **start the application from here**
- `build_vcpkg\Release\echoverse_sonar_lab.exe` — main app (launched by the launcher only)

Do not run `echoverse_sonar_lab.exe` directly; it requires `--from-esl-launcher` and a `--project` path that the launcher supplies.

`USE_REAL_SONAR_CORE` is always enabled in this repository and is no longer configurable.

## Output Session Layout

When at least one module enables TCP output or file recording, the app creates a new output session under:

- `Sonar Data/<yyyyMMdd_HHmmss>/` (project-local root)
- `Sonar Data/<timestamp>/<module_name>/2d.esl2d` (if ESL2D output enabled)
- `Sonar Data/<timestamp>/<module_name>/3d.esl3d` (if point-cloud output enabled)
- `Sonar Data/<timestamp>/<module_name>/Waveform Data/` (offline post-process output, when ESL3D file output enabled)
- `Sonar Data/<timestamp>/recording_summary.json` (session summary: duration, per-module frame counts, config snapshots)

## Documentation

- TCP protocol: `docs/sonar_tcp_protocol.md`
- ESL2D format: `docs/sonar_esl2d_data_spec.md`
- ESL3D format: `docs/sonar_esl3d_data_spec.md`
- H5 data spec: `docs/sonar_h5_data_spec.md`
- Model loading and custom assets: `docs/model_loading_and_custom_assets.md`
- Output session and recording summary: `docs/output_session_layout.md`

## License

The software code in this repository is released under the Apache License 2.0.
See `LICENSE` for the full text and `NOTICE` for repository-level notices.

Third-party models, meshes, textures, world files, and related simulation  
assets are not automatically covered by the repository code license. Their use  
and redistribution remain subject to original upstream license and attribution  
requirements.

## Notes

- This repository intentionally excludes large local/build/dependency directories from version control.
- If you need reproducible third-party dependencies, prefer using `vcpkg.json` and project setup scripts.

