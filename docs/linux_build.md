# Linux Build and Runtime Guide

This guide uses dependency capabilities and CMake package locations instead of tying
the main procedure to one Linux distribution. Exact package names differ between
package managers. The final section records the distributions and versions that were
actually tested.

## 1. Dependency Requirements

Install the following components with the package manager or toolchain appropriate to
your system:

| Component | Required capability |
|---|---|
| Compiler | C++17-capable GCC or Clang |
| CMake | 3.16 or newer; a current release is recommended |
| Git | Submodule support for the pinned Eigen checkout |
| Qt | Qt 6.9 or newer with Widgets, Network, and OpenGL support |
| OpenCV | Core, image processing, codecs, video I/O, and `highgui`; `highgui` must not introduce Qt 5 into the Qt 6 process |
| OpenSceneGraph | OSG and OpenThreads 3.6.x development files and runtime plugins |
| HDF5 | C development library used by the native offline waveform writer/reader |
| nlohmann-json | CMake-enabled development package used by JSON and ESL3D readers |
| Boost | Regex development library |
| Graphics stack | OpenGL/Mesa development files plus the XCB cursor runtime |
| Headless testing | Xvfb or an equivalent virtual display, optional but recommended |

Package names are distribution-specific. Common package families include a base
development toolchain, `cmake`, `git`, `pkg-config`/`pkgconf`, Qt 6 base development
files, OpenCV development files, OpenSceneGraph/OpenThreads, HDF5, nlohmann-json,
Boost.Regex, Mesa/OpenGL, an XCB cursor utility package, and Xvfb. Search the local
package index for the exact
names when they differ.

## 2. Clone and Initialize Eigen

```bash
git clone --recurse-submodules https://github.com/Qiuming-Ai/echoverse_sonar_lab.git
cd echoverse_sonar_lab
```

For an existing clone:

```bash
git submodule update --init --recursive
```

The project uses Eigen APIs newer than Eigen 3.4, so the pinned
`third_party/eigen` submodule is the default reproducible source. A compatible Eigen
5 checkout may instead be supplied with
`-DEIGEN3_INCLUDE_DIR=/path/to/eigen`.

## 3. Check Qt and OpenCV Compatibility

The application requires Qt 6.9 or newer. Before configuring, locate the Qt 6 and
OpenCV CMake package directories if they are not on the system CMake search path.
Typical directory endings are:

```text
lib/cmake/Qt6
lib/cmake/opencv4
lib/cmake/opencv5
```

An OpenCV `highgui` library compiled against Qt 5 can cause a pre-`main()` crash when
loaded into this Qt 6 application. Check the selected library with:

```bash
ldd /path/to/libopencv_highgui.so | grep -E 'Qt5|Qt6'
```

The output should contain Qt 6 dependencies and should not contain Qt 5 dependencies.
If both major versions appear, select or build an OpenCV package that uses Qt 6 only.

## 4. Configure and Build

Start with automatic system-package discovery:

```bash
cmake -S . -B build_linux -DCMAKE_BUILD_TYPE=Release
cmake --build build_linux --parallel 2
```

If Qt or OpenCV is installed in a nonstandard prefix, supply the package locations:

```bash
cmake -S . -B build_linux \
  -DCMAKE_BUILD_TYPE=Release \
  -DQt6_DIR=/path/to/lib/cmake/Qt6 \
  -DOpenCV_DIR=/path/to/lib/cmake/opencv4
cmake --build build_linux --parallel 2
```

For an OpenCV 5 installation, the final path commonly ends in `opencv5` instead.
Use a low parallel-build level on memory-constrained systems.

The complete build should report the offline library and both application targets:

```text
Built target echoverse_offline_processing
Built target echoverse_sonar_lab
Built target esl_launcher
```

When `BUILD_TESTING` is enabled (the default), validate the embedded HDF5/image stages
with:

```bash
ctest --test-dir build_linux --output-on-failure
```

## 5. Run and Smoke-Test

Normal launcher:

```bash
./build_linux/esl_launcher
```

Headless launcher-residency test:

```bash
timeout 60 xvfb-run -a -s "-screen 0 1280x800x24" \
  ./build_linux/esl_launcher
```

Exit code 124 from this particular command means `timeout` ended an application that
remained alive for the requested interval; it is not itself an application failure.

Headless project test:

```bash
xvfb-run -a -s "-screen 0 1280x800x24" \
  ./build_linux/echoverse_sonar_lab \
  --from-esl-launcher --project /absolute/path/to/Test.eslproj
```

The expected startup log includes:

```text
[gui] main camera renders offscreen (embedded in Qt UI)
```

For repeatable validation, use a fixed project, record the executable exit code, and
retain the complete configure, build, and runtime logs. A successful render loop does
not by itself prove a clean shutdown; the process should also exit with code 0.

## 6. Why the Linux Camera Path Is Different

The Windows build can embed the OSG native window through an HWND. That approach does
not generalize reliably when Qt and OSG use different Linux window-system backends.
The Linux path therefore renders the main OSG camera into an offscreen texture and
copies the image into `MainCameraView`, a Qt widget. This avoids an extra top-level
OSG window and does not depend on X11 re-parenting, so it is compatible with both X11
and Wayland desktop sessions.

## 7. Verified Environment Matrix

The general procedure above is distribution-neutral. The following entries record
only environments for which build or runtime evidence is available.

| Distribution | Toolchain and libraries | Verified scope | Remaining issue |
|---|---|---|---|
| Ubuntu 24.04 (VirtualBox) | GCC 13.3.0; CMake 3.30.5; Qt 6.10.2; OpenCV 4.12.0 with Qt 6; OpenSceneGraph/OpenThreads 3.6.5; pinned Eigen submodule | Author-supplied implementation compiled and ran; 4 virtual CPU cores and approximately 3.8 GB RAM | The newly merged public revision still needs a recorded clean-clone reproduction if Ubuntu-specific evidence is required |
| Arch Linux, kernel 7.1.6-arch1-1, x86_64 | GCC 16.1.1; CMake 4.4.2; Qt 6.11.1; OpenCV 5.0.0 with Qt 6; OpenSceneGraph 3.6.5-34; Boost; pinned Eigen submodule | A final clean public clone configured and built successfully; `echoverse_sonar_lab` and `esl_launcher` both reached 100%; the launcher and prepared project ran normally and exited normally | No error was observed in the author-confirmed final test |

The Arch commands that succeeded were:

```bash
cmake -S . -B build_linux \
  -DCMAKE_BUILD_TYPE=Release \
  -DQt6_DIR=/usr/lib/cmake/Qt6 \
  -DOpenCV_DIR=/usr/lib/cmake/opencv5
cmake --build build_linux --parallel 2
cmake --build build_linux --target esl_launcher --parallel 2
```

The installation record is retained locally at
`linux/arch Linux_session_log_2026-08-10_install_deps.md`. The final support statement
above reflects the author's subsequent clean-clone build and normal runtime/exit
confirmation.

## 8. Common Failures

### Missing `Eigen/Core`

Run:

```bash
git submodule update --init --recursive
```

Then remove the stale CMake cache or configure a new build directory.

### `QImage::flipped` is missing

The selected Qt is too old. Use Qt 6.9 or newer and reconfigure from a clean build
directory.

### Application crashes before printing startup logs

Inspect the selected OpenCV `highgui` library. If both Qt 5 and Qt 6 are loaded, use
an OpenCV build linked to Qt 6 only.

### Qt or OpenCV is not discovered

Supply `Qt6_DIR` and `OpenCV_DIR` as shown in Section 4. Point each variable to the
directory containing the package's `*Config.cmake` file, not merely to an include or
library directory.

### Qt package exposes unevaluated generator expressions

Use a current CMake release and a clean build directory. Avoid editing an installed
Qt package configuration unless the failure is reproducible and the local change is
documented.

### `BadWindow` or `GLXBadContext`

Confirm that the current source includes `MainCameraView`. The supported Linux path
does not depend on sharing a native X11 window.
