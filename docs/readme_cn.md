# EchoVerse Sonar Lab

> [English](../README.md) | 简体中文

EchoVerse Sonar Lab 是一个开源的多模态声呐仿真、可视化、数据流传输与离线信号处理环境。它在同一个共享三维场景中支持前视声呐（FLS）、多波束测深仪（MBES）和侧扫声呐（SSS）工作流。

## 各运行时的职责

本软件在同一个 C++ 进程内使用两条相互衔接的执行路径：

| 运行时 | 职责 | 主要输出 |
|---|---|---|
| C++ 在线运行时 | 加载场景，实时渲染声呐回波/强度图像，恢复回波点云，显示 GUI，传输数据流，并记录封包帧 | 实时 FLS/MBES/SSS 图像、`.esl2d`、`.esl3d`、TCP 数据流 |
| 原生 C++ 离线管线 | 读取已记录的 `.esl3d` 帧，合成多通道回波，执行匹配滤波/TVG/波束形成，重建波形域图像 | HDF5 通道波形数据，以及每个 ping 一张灰度 PNG |

在线输出是原生信号级管线的几何/强度域输入。启用该功能后，处理在 ESL3D 录制停止后启动。它直接调用内嵌库，不启动转换器可执行文件，不依赖 MATLAB，也不会修改所选的声呐 JSON 文件。

## 功能特性

- 面向 FLS、MBES 和 SSS 模块的交互式共享场景
- 实时 C++ 回波/强度图像生成
- C++ 极坐标距离/强度帧与点云恢复
- 会话级 `.esl2d` / `.esl3d` 录制与 TCP 数据流传输
- 原生 C++ 回波合成、多普勒/噪声处理、HDF5 导出与 PNG 图像重建
- 后台离线处理，GUI 显示处理进度
- 可选启用的 C++ 性能日志，用于规模实验
- 支持 Windows 和 Linux GUI；Linux 采用与 X11 和 Wayland 兼容的离屏主相机路径

## 仓库结构

- `src/`：C++ 运行时、原生离线管线，以及可选的 MATLAB 科研分析工具箱
- `src/offline_processing/`：内嵌的 ESL3D 到 HDF5/图像处理库
- `docs/`：架构、数据格式、构建、性能与开发文档
- `uwmodels/`：示例水下模型与场景
- `CMakeLists.txt`：顶层 CMake 配置
- `vcpkg.json`：面向 vcpkg 构建的 Qt/OpenCV/OpenSceneGraph 清单
- `third_party/eigen`：由本仓库固定的 Eigen Git 子模块

## 支持与已测试环境

| 平台 | 验证状态 | 已测试环境 |
|---|---|---|
| Windows | 已完成构建与运行验证 | Windows 11、Visual Studio 2022、vcpkg |
| Linux (Ubuntu) | 已完成构建与运行验证 | Ubuntu 24.04、GCC 13.3、CMake 3.30.5、Qt 6.10.2、OpenCV 4.12.0（Qt 6 版）、OpenSceneGraph 3.6.5 |
| Linux (Arch) | 已完成构建与运行验证 | Arch Linux、内核 7.1.6-arch1-1、GCC 16.1.1、CMake 4.4.2、Qt 6.11.1、OpenCV 5.0.0（Qt 6 版）、OpenSceneGraph 3.6.5-34 |

Linux GUI 实现不与 Qt 共享原生 X11 窗口。主 OSG 相机进行离屏渲染，并在 Qt 控件中显示，因此在 Wayland 上同样可用。

## 克隆（含 Eigen 子模块）

Eigen 以 Git 子模块形式提供，而不是直接复制的依赖目录。请使用以下命令克隆：

```bash
git clone --recurse-submodules https://github.com/Qiuming-Ai/echoverse_sonar_lab.git
cd echoverse_sonar_lab
```

对于已存在的克隆：

```bash
git submodule update --init --recursive
```

固定的 Eigen 版本提供了本项目所需、而 Eigen 3.4 中不可用的 API。也可以通过
`-DEIGEN3_INCLUDE_DIR=/path/to/eigen` 指定其他兼容的 Eigen 5 检出目录。

## 构建要求

- CMake 3.16 或更新版本；已测试的 Linux Qt 工具链推荐 CMake 3.30+
- C++17 编译器
- 来自固定子模块的 Eigen 5
- Qt Widgets 与 Network 模块；推荐 Qt 6.9+
- OpenCV 的 core、imgproc、imgcodecs、videoio 与 highgui 模块
- OpenSceneGraph 与 OpenThreads 3.6.x
- HDF5 C 库与 nlohmann-json（Windows 上由 vcpkg 清单自动安装）

## 快速开始：Windows

在 `third_party` 工作目录（已被忽略）之内或之外使用常规 vcpkg 安装。在 PowerShell 中：

```powershell
git submodule update --init --recursive

$env:VCPKG_ROOT = "C:\path\to\vcpkg"
& "$env:VCPKG_ROOT\vcpkg.exe" install --triplet x64-windows

cmake -S . -B build_vcpkg -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE="$env:VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake" `
  -DVCPKG_TARGET_TRIPLET=x64-windows
cmake --build build_vcpkg --config Release
ctest --test-dir build_vcpkg -C Release --output-on-failure
```

启动 `build_vcpkg\Release\esl_launcher.exe`。启动器会为 `echoverse_sonar_lab.exe` 提供所需的项目路径。

## 快速开始：Linux

使用系统包管理器安装 C++17 编译器、CMake、Git、Qt 6 开发文件、OpenCV 开发文件、
OpenSceneGraph/OpenThreads、Boost.Regex、OpenGL/Mesa 开发文件以及 XCB cursor 运行库。
Qt 必须为 6.9 或更新版本。OpenCV 的 `highgui` 必须只链接到 Qt 6；在同一进程中同时加载
Qt 5 与 Qt 6 可能导致程序在 `main()` 之前崩溃。

```bash
git submodule update --init --recursive

cmake -S . -B build_linux -DCMAKE_BUILD_TYPE=Release
cmake --build build_linux --parallel 2
ctest --test-dir build_linux --output-on-failure
./build_linux/esl_launcher
```

当 Qt 或 OpenCV 安装在系统搜索路径之外时，请添加对应的 CMake 包目录，不要假设特定发行版的安装位置：

```bash
cmake -S . -B build_linux \
  -DCMAKE_BUILD_TYPE=Release \
  -DQt6_DIR=/path/to/lib/cmake/Qt6 \
  -DOpenCV_DIR=/path/to/lib/cmake/opencv4
```

依赖角色说明、已验证的发行版/版本记录、Qt/OpenCV 兼容性检查、无头冒烟测试以及故障排查，请参阅
[`docs/linux_build.md`](linux_build.md)。

## 原生离线回波与图像管线

在 **Settings → Output** 中：

1. 为 FLS 或 MBES 模块启用 ESL3D 点云文件输出。
2. 启用 **Generate raw waveform and reconstructed images**。
3. 开始并停止一次录制会话。

ESL3D 写入器关闭后，应用程序会在后台线程上运行内嵌管线。它会在该模块的 `Waveform Data`
目录中写入 `<recording>.h5` 以及带时间戳的 `pingNNN.png` 文件。
`src/offline_processing/config/` 下的配置提供工程模板，但其 `esl3d_path` 和 `output_path`
值仅在内存中被覆盖。

原生实现与模板位于 `src/offline_processing/`。这使产品工作流完全自包含：录制、波形合成、
HDF5 序列化与图像重建均在 C++ 中完成，无需 MATLAB。`src/matlab_point2file2image/` 下的
MATLAB 代码仍可作为额外的科研后分析工具箱，用于算法检视、参数研究、可复现性检查、备选绘图
以及与原生结果的对比。它不是运行时回退方案、部署依赖，也不是必需的产品工作流步骤。
分阶段的数据流与配置细节请参阅
[`docs/Offline Waveform and Image Reconstruction Pipeline.md`](Offline%20Waveform%20and%20Image%20Reconstruction%20Pipeline.md)。

## 性能与规模日志

C++ 性能分析通过 `ESL_CPP_PERF_CSV` 可选启用：

```bash
ESL_CPP_PERF_CSV=results/cpp_performance.csv \
ESL_PERF_RUN_LABEL=shipwreck_100_pings \
./build_linux/esl_launcher
```

原生离线性能分析同样为可选启用：

```powershell
$env:ESL_OFFLINE_PERF_CSV = "results/offline_performance.csv"
& .\build_vcpkg\Release\esl_launcher.exe
```

日志包含每帧/每 ping 的执行时间、场景清单、波束/距离单元数量、点数、输出字节估算、波形维度
以及 CPU 后端标识。完整的实验规程与当前可扩展性限制请参阅
[`docs/performance_and_scalability.md`](performance_and_scalability.md)。

针对预先准备的管线检视与珊瑚项目，已基于 Windows 完成了三次重复的特征测量。所测场景分别包含
52,096 和 591,116 个估计加载的三角形，且两者均维持了配置的 5 fps GUI 循环上限。这些测量仅涵盖
关闭文件/TCP 输出的在线 C++ 运行时，不构成对离线管线、无上限吞吐量或通用场景承载能力的结论。
生成的结果表格、CSV 文件、执行日志与基准测试记录保存在源码仓库之外的
[性能与基准测试数据归档](https://drive.google.com/drive/folders/1FLh2osev_QVqSBR7Gu0UJmejG-zf4_zh?usp=drive_link)。

## 输出会话目录结构

启用文件或 TCP 输出后，应用程序会创建：

- `Sonar Data/<timestamp>/<module_name>/2d.esl2d`
- `Sonar Data/<timestamp>/<module_name>/3d.esl3d`
- `Sonar Data/<timestamp>/<module_name>/Waveform Data/3d.h5`（启用原生离线处理时）
- `Sonar Data/<timestamp>/<module_name>/Waveform Data/3d_<timestamp>_pingNNN.png`
- `Sonar Data/<timestamp>/recording_summary.json`

## 文档

- 总体架构：[`docs/software_architecture_analysis.md`](software_architecture_analysis.md)
- C++ 声学核心：[`docs/acoustic_simulation_core_overview.md`](acoustic_simulation_core_overview.md)
- 原生离线管线：[`docs/Offline Waveform and Image Reconstruction Pipeline.md`](Offline%20Waveform%20and%20Image%20Reconstruction%20Pipeline.md)
- Linux 构建：[`docs/linux_build.md`](linux_build.md)
- 性能/可扩展性：[`docs/performance_and_scalability.md`](performance_and_scalability.md)
- AI 辅助开发声明：[`docs/ai_assisted_development.md`](ai_assisted_development.md)
- TCP 协议：[`docs/sonar_tcp_protocol.md`](sonar_tcp_protocol.md)
- ESL2D 格式：[`docs/sonar_esl2d_data_spec.md`](sonar_esl2d_data_spec.md)
- ESL3D 格式：[`docs/sonar_esl3d_data_spec.md`](sonar_esl3d_data_spec.md)
- HDF5 格式：[`docs/sonar_h5_data_spec.md`](sonar_h5_data_spec.md)

## AI 辅助开发声明

生成式 AI 工具辅助完成了部分编码与写作任务。核心科学思想、软件架构、核心声呐仿真与
信号处理功能、实验工作及其解读均由作者完成。所有 AI 辅助产出均经作者审阅与编辑，所有被
接受的代码变更与报告结果均通过人工主导的构建、测试与实际实验加以验证。声明与验证原则详见
[`docs/ai_assisted_development.md`](ai_assisted_development.md)。

## 引用

如果您在研究中使用了 EchoVerse Sonar Lab,请引用配套的软件论文:[https://doi.org/10.1016/j.softx.2026.102994](https://doi.org/10.1016/j.softx.2026.102994)

## 许可证

软件代码以 Apache License 2.0 发布。详见 `LICENSE` 与 `NOTICE`。第三方模型、网格、纹理、
世界文件及其他资源可能有其各自的上游条款，不会自动被代码许可证覆盖。
