# Linux Build and Runtime Guide

## 1. Verified Environment

The Linux port was compiled and run on:

- Ubuntu 24.04 in VirtualBox
- GCC 13.3.0
- CMake 3.30.5 for the final dependency build
- Qt 6.10.2
- OpenCV 4.12.0 built with Qt 6 support
- OpenSceneGraph/OpenThreads 3.6.5
- Eigen revision pinned by the `third_party/eigen` submodule
- 4 virtual CPU cores and approximately 3.8 GB RAM

This is the currently verified Linux configuration. Arch Linux clean-clone testing is
still required before claiming Arch as a tested platform.

## 2. Clone and Initialize Dependencies

```bash
git clone --recurse-submodules https://github.com/Qiuming-Ai/echoverse_sonar_lab.git
cd echoverse_sonar_lab
```

For an existing clone:

```bash
git submodule update --init --recursive
```

The project uses Eigen APIs newer than Eigen 3.4, so the pinned submodule is the
default and reproducible Eigen source.

## 3. Base Ubuntu Packages

```bash
sudo apt-get update
sudo apt-get install -y \
  build-essential cmake libopenscenegraph-dev libopenthreads-dev \
  libxcb-cursor0 xvfb mesa-utils
```

Qt and OpenCV must be mutually compatible. The verified setup uses Qt 6.10.2 and an
OpenCV 4.12.0 build whose `highgui` module is linked to Qt 6 only.

## 4. Qt Requirement

The verified Qt installation was obtained with `aqtinstall`:

```bash
python3 -m pip install --user aqtinstall
python3 -m aqt install-qt linux desktop 6.10.2 linux_gcc_64 -O "$HOME/Qt"
```

Qt 6.9+ is recommended because the current image-processing code uses APIs introduced
after the Qt version shipped by Ubuntu 24.04.

## 5. OpenCV Requirement

Some Ubuntu OpenCV packages build `opencv_highgui` against Qt 5. Loading such a
library into this Qt 6 application can cause a pre-`main()` crash. Check with:

```bash
ldd /path/to/libopencv_highgui.so | grep -E 'Qt5|Qt6'
```

The verified OpenCV build was configured as follows:

```bash
cmake -S "$HOME/opencv_src/opencv-4.12.0" -B "$HOME/opencv_build/4.12.0" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$HOME/opencv-4.12.0" \
  -DBUILD_LIST=core,imgproc,imgcodecs,videoio,highgui \
  -DWITH_QT=ON \
  -DQt6_DIR="$HOME/Qt/6.10.2/gcc_64/lib/cmake/Qt6" \
  -DQT_QMAKE_EXECUTABLE="$HOME/Qt/6.10.2/gcc_64/bin/qmake" \
  -DWITH_GTK=OFF -DWITH_FFMPEG=ON -DWITH_OPENEXR=OFF -DWITH_V4L=ON \
  -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF \
  -DBUILD_opencv_apps=OFF -DBUILD_opencv_python3=OFF -DBUILD_opencv_java=OFF \
  -DENABLE_PRECOMPILED_HEADERS=OFF
cmake --build "$HOME/opencv_build/4.12.0" -j2
cmake --install "$HOME/opencv_build/4.12.0"
```

After installation, verify that `libopencv_highgui.so` links to Qt 6 and not Qt 5.

## 6. Configure and Build EchoVerse Sonar Lab

```bash
cmake -S . -B build_linux \
  -DCMAKE_BUILD_TYPE=Release \
  -DQt6_DIR="$HOME/Qt/6.10.2/gcc_64/lib/cmake/Qt6" \
  -DOpenCV_DIR="$HOME/opencv-4.12.0/lib/cmake/opencv4"
cmake --build build_linux -j2
```

On memory-constrained systems, keep the parallel build level low or temporarily add
swap space.

## 7. Run and Smoke-Test

Normal launcher:

```bash
./build_linux/esl_launcher
```

Virtual-display GUI test:

```bash
xvfb-run -a -s "-screen 0 1280x800x24" \
  ./build_linux/echoverse_sonar_lab \
  --from-esl-launcher --project /absolute/path/to/Test.eslproj
```

The expected Linux startup log includes:

```text
[gui] main camera renders offscreen (embedded in Qt UI)
```

## 8. Why the Linux Camera Path Is Different

The Windows build can embed the OSG native window through an HWND. That approach does
not generalize reliably to Linux, especially when Qt uses Wayland and OSG uses X11 or
GLX. Linux therefore renders the main OSG camera into an offscreen texture and copies
the image into `MainCameraView`, a Qt widget. This prevents the extra top-level OSG
window and avoids X11 re-parenting.

## 9. Common Failures

### Missing Eigen/Core

Run:

```bash
git submodule update --init --recursive
```

### `QImage::flipped` is missing

The selected Qt is too old for the verified code path. Use Qt 6.9+ or the tested Qt
6.10.2 build.

### Application crashes before printing startup logs

Inspect `ldd` output. If both Qt 5 and Qt 6 are loaded, use an OpenCV `highgui` build
linked to Qt 6 only.

### OpenCV/Qt build exposes unevaluated Qt generator expressions

Use a current CMake release and a clean build directory. The verified environment used
CMake 3.30.5. Do not modify an installed Qt package configuration unless the failure is
reproduced and the change is documented locally; a clean compatible Qt/OpenCV build is
preferred.

### `BadWindow` or `GLXBadContext`

Ensure the current source includes `MainCameraView`. The supported Linux path does not
depend on sharing a native X11 window.

