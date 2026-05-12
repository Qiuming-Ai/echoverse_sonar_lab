# EchoVerse Sonar Lab

EchoVerse Sonar Lab is a standalone multibeam sonar simulation and visualization project.
It provides:

- GUI application (`multibeam_gui`) for scene editing and sonar visualization
- Launcher (`esl_launcher`) for startup and configuration
- TCP point cloud streaming and `.esl3d` binary recording workflow

## Features

- Configurable sonar simulation (FLS / MBES / SSS related modules)
- Point cloud TCP streaming protocol
- `.esl3d` packet-based binary storage format
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

## Third-Party Library Links (Referenced by CMake)

- [Eigen](https://github.com/eigenteam/eigen-git-mirror)
- [OpenCV](https://github.com/opencv/opencv)
- [OpenSceneGraph](https://github.com/openscenegraph/OpenSceneGraph)
- [Qt](https://github.com/qt/qtbase)
- [vcpkg](https://github.com/microsoft/vcpkg)

## Quick Start (Windows PowerShell, relative paths)

```powershell
# Recommended: configure with vcpkg toolchain (runtime DLL deployment works out-of-box)
cmake -S . -B build_vcpkg -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE="./third_party/vcpkg/scripts/buildsystems/vcpkg.cmake" `
  -DVCPKG_TARGET_TRIPLET=x64-windows
cmake --build build_vcpkg --config Release
```

Common executables after build:

- `build_vcpkg\Release\multibeam_gui.exe`
- `build_vcpkg\Release\esl_launcher.exe`

`USE_REAL_SONAR_CORE` is always enabled in this repository and is no longer configurable.

## Documentation

- TCP protocol: `docs/sonar_pointcloud_tcp_protocol.md`
- ESL3D format: `docs/esl3d_file_format.md`
- H5 data spec: `docs/sonar_h5_data_spec.md`
- Model loading and custom assets: `docs/model_loading_and_custom_assets.md`

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

