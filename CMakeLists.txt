cmake_minimum_required(VERSION 3.16)
project(multibeam_standalone_mvp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# USE_REAL_SONAR_CORE is always enabled in this project.
# Remove stale cache entries from older configurations and force local ON.
unset(USE_REAL_SONAR_CORE CACHE)
set(USE_REAL_SONAR_CORE ON)

# Minimal local tree for standalone MVP build
set(MVP_REPO_ROOT "${CMAKE_CURRENT_LIST_DIR}" CACHE PATH "standalone local root")
file(TO_CMAKE_PATH "${MVP_REPO_ROOT}" MVP_REPO_ROOT_CMAKE)

if(USE_REAL_SONAR_CORE)
    add_compile_definitions(USE_REAL_SONAR_CORE=1)

    set(REPO_SIMULATION_DIR "${CMAKE_CURRENT_LIST_DIR}/src" CACHE PATH "Path to local simulation source directory")
    file(TO_CMAKE_PATH "${REPO_SIMULATION_DIR}" REPO_SIMULATION_DIR_CMAKE)
    set(SONAR_TYPES_V2_DIR "${CMAKE_CURRENT_LIST_DIR}/src/sonar_types_v2" CACHE PATH "Path containing sonar_types_v2 headers")
    set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/third_party/eigen" CACHE PATH "Path to Eigen headers" FORCE)
    set(OSG_ROOT "$ENV{OSG_ROOT}" CACHE PATH "OpenSceneGraph installation root")
    set(OPENTHREADS_ROOT "$ENV{OPENTHREADS_ROOT}" CACHE PATH "OpenThreads installation root")
    set(OpenCV_ROOT "$ENV{OpenCV_ROOT}" CACHE PATH "OpenCV installation root")
    set(OpenCV_DIR "$ENV{OpenCV_DIR}" CACHE PATH "OpenCV CMake config directory")

    if(NOT EXISTS "${REPO_SIMULATION_DIR}/sonar_core/AcousticRaySimulator.cpp")
        message(FATAL_ERROR "USE_REAL_SONAR_CORE=ON requires sonar_core sources under ${REPO_SIMULATION_DIR}")
    endif()
    if(NOT EXISTS "${SONAR_TYPES_V2_DIR}/echoverse_sonar_types.hpp")
        message(FATAL_ERROR "USE_REAL_SONAR_CORE=ON requires sonar_types_v2 headers, missing ${SONAR_TYPES_V2_DIR}/echoverse_sonar_types.hpp")
    endif()

    if(NOT EXISTS "${EIGEN3_INCLUDE_DIR}/Eigen/Core")
        message(FATAL_ERROR "USE_REAL_SONAR_CORE=ON requires Eigen headers, missing ${EIGEN3_INCLUDE_DIR}/Eigen/Core")
    endif()
endif()

set(ON_COMMON_INCLUDE_DIRS)
set(ON_OSG_LINK_LIBS)
set(ON_OPENCV_LINK_LIBS)

if(USE_REAL_SONAR_CORE)
    set(ON_COMMON_INCLUDE_DIRS
        src
        ${EIGEN3_INCLUDE_DIR}
        ${REPO_SIMULATION_DIR}/sonar_core
        ${REPO_SIMULATION_DIR}/sonar_imaging
        ${MVP_REPO_ROOT}/src/sonar_types_v2
    )
    set(STANDALONE_OSG_PLUGIN_ROOT_HINT "")

    set(HAVE_VCPKG OFF)
    if(DEFINED CMAKE_TOOLCHAIN_FILE OR DEFINED ENV{VCPKG_ROOT})
        set(HAVE_VCPKG ON)
    endif()

    set(ON_OSG_LINK_LIBS)
    set(ON_OPENCV_LINK_LIBS)

    if(DEFINED VCPKG_INSTALLED_DIR AND DEFINED VCPKG_TARGET_TRIPLET)
        list(APPEND ON_COMMON_INCLUDE_DIRS "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/include")
        set(STANDALONE_OSG_PLUGIN_ROOT_HINT "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/plugins")
    endif()

    if(HAVE_VCPKG OR OpenCV_DIR)
        find_package(OpenCV CONFIG QUIET)
        find_package(OpenSceneGraph QUIET COMPONENTS osg osgDB osgUtil osgGA osgViewer OpenThreads)
        if(OpenCV_FOUND AND TARGET opencv_world)
            list(APPEND ON_OPENCV_LINK_LIBS opencv_world)
        elseif(OpenCV_FOUND AND TARGET opencv_core AND TARGET opencv_imgproc)
            list(APPEND ON_OPENCV_LINK_LIBS opencv_core opencv_imgproc)
        endif()
        if(OpenSceneGraph_FOUND)
            foreach(lib ${OPENSCENEGRAPH_LIBRARIES})
                list(APPEND ON_OSG_LINK_LIBS ${lib})
            endforeach()
            # Ensure OSG headers are available during compilation when package discovery
            # succeeds via find_package() (not just during fallback find_path()).
            if(OPENSCENEGRAPH_INCLUDE_DIRS)
                list(APPEND ON_COMMON_INCLUDE_DIRS ${OPENSCENEGRAPH_INCLUDE_DIRS})
            endif()
            foreach(_osg_inc_var
                OSG_INCLUDE_DIR
                OSGDB_INCLUDE_DIR
                OSGUTIL_INCLUDE_DIR
                OSGGA_INCLUDE_DIR
                OSGVIEWER_INCLUDE_DIR
                OPENTHREADS_INCLUDE_DIR)
                if(DEFINED ${_osg_inc_var} AND ${_osg_inc_var})
                    list(APPEND ON_COMMON_INCLUDE_DIRS ${${_osg_inc_var}})
                endif()
            endforeach()
        endif()
    endif()
    list(REMOVE_DUPLICATES ON_COMMON_INCLUDE_DIRS)
    if(NOT STANDALONE_OSG_PLUGIN_ROOT_HINT AND OSG_ROOT)
        set(STANDALONE_OSG_PLUGIN_ROOT_HINT "${OSG_ROOT}/plugins")
    endif()
    if(NOT STANDALONE_OSG_PLUGIN_ROOT_HINT AND OSG_ROOT)
        set(STANDALONE_OSG_PLUGIN_ROOT_HINT "${OSG_ROOT}/bin")
    endif()

    if(NOT ON_OPENCV_LINK_LIBS)
        find_path(OPENCV_INCLUDE_DIR opencv2/core/core.hpp
            HINTS ${OpenCV_ROOT}
            PATH_SUFFIXES include include/opencv4 build/include)
        find_library(OPENCV_WORLD_LIB
            NAMES opencv_world opencv_world4100 opencv_world490 opencv_world480 opencv_world470
            HINTS ${OpenCV_ROOT}
            PATH_SUFFIXES x64/vc17/lib x64/vc16/lib x64/vc15/lib lib build/x64/vc17/lib build/x64/vc16/lib build/x64/vc15/lib)
        if(OPENCV_WORLD_LIB)
            list(APPEND ON_OPENCV_LINK_LIBS ${OPENCV_WORLD_LIB})
            if(OPENCV_INCLUDE_DIR)
                list(APPEND ON_COMMON_INCLUDE_DIRS ${OPENCV_INCLUDE_DIR})
            endif()
        endif()
    endif()

    if(NOT ON_OSG_LINK_LIBS)
        set(OSG_HINTS ${OSG_ROOT} ${OPENTHREADS_ROOT})
        find_path(OSG_INCLUDE_DIR osg/Group
            HINTS ${OSG_HINTS}
            PATH_SUFFIXES include)
        find_library(OSG_LIB NAMES osg HINTS ${OSG_HINTS} PATH_SUFFIXES lib lib64)
        find_library(OSGDB_LIB NAMES osgDB HINTS ${OSG_HINTS} PATH_SUFFIXES lib lib64)
        find_library(OSGUTIL_LIB NAMES osgUtil HINTS ${OSG_HINTS} PATH_SUFFIXES lib lib64)
        find_library(OSGGA_LIB NAMES osgGA HINTS ${OSG_HINTS} PATH_SUFFIXES lib lib64)
        find_library(OSGVIEWER_LIB NAMES osgViewer HINTS ${OSG_HINTS} PATH_SUFFIXES lib lib64)
        find_library(OPENTHREADS_LIB NAMES OpenThreads HINTS ${OSG_HINTS} PATH_SUFFIXES lib lib64)
        if(OSG_INCLUDE_DIR)
            list(APPEND ON_COMMON_INCLUDE_DIRS ${OSG_INCLUDE_DIR})
        endif()
        if(OSG_LIB AND OSGDB_LIB AND OSGUTIL_LIB AND OSGGA_LIB AND OSGVIEWER_LIB AND OPENTHREADS_LIB)
            list(APPEND ON_OSG_LINK_LIBS
                ${OSG_LIB}
                ${OSGDB_LIB}
                ${OSGUTIL_LIB}
                ${OSGGA_LIB}
                ${OSGVIEWER_LIB}
                ${OPENTHREADS_LIB})
        endif()
    endif()

    if(NOT ON_OPENCV_LINK_LIBS)
        message(FATAL_ERROR "USE_REAL_SONAR_CORE=ON requires OpenCV link libs. Set OpenCV_ROOT/OpenCV_DIR or vcpkg toolchain.")
    endif()
    if(NOT ON_OSG_LINK_LIBS)
        message(FATAL_ERROR "USE_REAL_SONAR_CORE=ON requires OSG/OpenThreads link libs. Set OSG_ROOT/OPENTHREADS_ROOT or vcpkg toolchain.")
    endif()

endif()

if(USE_REAL_SONAR_CORE)
    option(BUILD_MULTIBEAM_GUI "Build echoverse_sonar_lab viewer (needs Qt5 or Qt6 Widgets; see vcpkg.json qtbase)" ON)
    if(BUILD_MULTIBEAM_GUI)
        set(MVP_QT_WIDGETS_LIB "")
        set(MVP_QT_NETWORK_LIB "")
        find_package(Qt6 QUIET COMPONENTS Widgets)
        if(Qt6_FOUND)
            set(MVP_QT_WIDGETS_LIB Qt6::Widgets)
            find_package(Qt6 QUIET COMPONENTS Network)
            if(Qt6Network_FOUND)
                set(MVP_QT_NETWORK_LIB Qt6::Network)
            endif()
        else()
            find_package(Qt5 QUIET COMPONENTS Widgets)
            if(Qt5_FOUND)
                set(MVP_QT_WIDGETS_LIB Qt5::Widgets)
                find_package(Qt5 QUIET COMPONENTS Network)
                if(Qt5Network_FOUND)
                    set(MVP_QT_NETWORK_LIB Qt5::Network)
                endif()
            endif()
        endif()
        if(NOT MVP_QT_WIDGETS_LIB OR NOT MVP_QT_NETWORK_LIB)
            message(FATAL_ERROR
                "echoverse_sonar_lab requires Qt5 or Qt6 Widgets + Network, but not all components were found.\n"
                "  vcpkg: ensure 'qtbase' is installed for your triplet (see simulation/standalone_mvp/vcpkg.json), e.g.\n"
                "    ${CMAKE_CURRENT_LIST_DIR}/third_party/vcpkg/vcpkg install qtbase:x64-windows\n"
                "  Or set Qt6_DIR or Qt5_DIR to the directory containing Qt*Config.cmake .\n"
                "  To build without the GUI: configure with -DBUILD_MULTIBEAM_GUI=OFF")
        endif()
        set(CMAKE_AUTOMOC ON)

        set(SONAR_UI_DIR "${MVP_REPO_ROOT}/src/sonar_ui")
        add_executable(echoverse_sonar_lab
        src/SharedScene.cpp
        src/RockSonarPlotView.cpp
        src/AppConfig.cpp
        src/InfoPanelWidget.cpp
        src/SettingsDialog.cpp
        src/PointCloudSonarSimulation.cpp
        src/PointCloudTcpStreamer.cpp
        src/PointCloudViewerWindow.cpp
        src/CameraModule.cpp
        src/FlsModule.cpp
        src/MbesModule.cpp
        src/SssModule.cpp
        src/WorldFileIo.cpp
        src/SceneEditorPanel.cpp
        src/PathEditorPanel.cpp
        src/PathFollower.cpp
        src/echoverse_sonar_lab.cpp
        src/ui/DockWorkspace.cpp
        src/ui/DraggableTabBar.cpp
        src/ui/TabDropOverlay.cpp
        ${SONAR_UI_DIR}/SonarCanvas.cpp
        ${SONAR_UI_DIR}/SonarControlPanel.cpp
        ${SONAR_UI_DIR}/SideScanWaterfallCanvas.cpp
        ${SONAR_UI_DIR}/SideScanControlPanel.cpp
        ${MVP_REPO_ROOT}/src/sonar_palette/PaletteRamp.cpp
        ${MVP_REPO_ROOT}/src/sonar_types_v2/echoverse_time_types.cpp
        ${MVP_REPO_ROOT}/src/sonar_types_v2/echoverse_sonar_types.cpp
        ${REPO_SIMULATION_DIR}/sonar_core/AcousticRaySimulator.cpp
        ${REPO_SIMULATION_DIR}/sonar_core/BeamImageSynthesizer.cpp
        ${REPO_SIMULATION_DIR}/sonar_core/SignalProcessingUtils.cpp
        ${REPO_SIMULATION_DIR}/sonar_imaging/SceneDepthComposer.cpp
        ${REPO_SIMULATION_DIR}/sonar_imaging/OffscreenCaptureRig.cpp
        ${REPO_SIMULATION_DIR}/sonar_imaging/TopDownDepthMapGenerator.cpp
        ${REPO_SIMULATION_DIR}/sonar_imaging/GeometryTextureTools.cpp
    )
    if(WIN32)
        target_sources(echoverse_sonar_lab PRIVATE src/resources/echoverse_sonar_lab.rc)
    endif()
    target_include_directories(echoverse_sonar_lab PRIVATE ${ON_COMMON_INCLUDE_DIRS}
        "${SONAR_UI_DIR}"
        "${MVP_REPO_ROOT}/src/sonar_palette")
    target_link_libraries(echoverse_sonar_lab PRIVATE
        ${ON_OSG_LINK_LIBS}
        ${ON_OPENCV_LINK_LIBS}
        ${MVP_QT_WIDGETS_LIB}
        ${MVP_QT_NETWORK_LIB})
    if(TARGET opencv_highgui)
        target_link_libraries(echoverse_sonar_lab PRIVATE opencv_highgui)
    else()
        find_library(OPENCV_HIGHGUI_LIB
            NAMES opencv_highgui opencv_highgui4100 opencv_highgui490 opencv_highgui480 opencv_highgui470
            HINTS ${OpenCV_ROOT}
            PATH_SUFFIXES x64/vc17/lib x64/vc16/lib x64/vc15/lib lib build/x64/vc17/lib build/x64/vc16/lib build/x64/vc15/lib)
        if(OPENCV_HIGHGUI_LIB)
            target_link_libraries(echoverse_sonar_lab PRIVATE ${OPENCV_HIGHGUI_LIB})
        endif()
    endif()
    target_compile_definitions(echoverse_sonar_lab PRIVATE STANDALONE_SIMULATION_DIR="${MVP_REPO_ROOT_CMAKE}")
    if(STANDALONE_OSG_PLUGIN_ROOT_HINT_CMAKE)
        target_compile_definitions(echoverse_sonar_lab PRIVATE STANDALONE_OSG_PLUGIN_ROOT="${STANDALONE_OSG_PLUGIN_ROOT_HINT_CMAKE}")
    endif()
    if(STANDALONE_OSG_PLUGIN_ROOT_HINT)
        add_custom_command(TARGET echoverse_sonar_lab POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${STANDALONE_OSG_PLUGIN_ROOT_HINT}"
                "$<TARGET_FILE_DIR:echoverse_sonar_lab>/")
    endif()
    # Ensure DAE(COLLADA) runtime pieces are present beside the GUI executable.
    if(DEFINED VCPKG_INSTALLED_DIR AND DEFINED VCPKG_TARGET_TRIPLET)
        set(MVP_VCPKG_BIN_DIR "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/bin")
        set(MVP_OSG_PLUGIN_DIR "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/plugins")
        if(EXISTS "${MVP_VCPKG_BIN_DIR}")
            # Keep runtime dependency DLLs (including transitive COLLADA deps) near the executable.
            add_custom_command(TARGET echoverse_sonar_lab POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory
                    "${MVP_VCPKG_BIN_DIR}"
                    "$<TARGET_FILE_DIR:echoverse_sonar_lab>/")
        endif()
        file(GLOB MVP_COLLADA_DLLS "${MVP_VCPKG_BIN_DIR}/collada-dom*.dll")
        foreach(_dll ${MVP_COLLADA_DLLS})
            add_custom_command(TARGET echoverse_sonar_lab POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${_dll}"
                    "$<TARGET_FILE_DIR:echoverse_sonar_lab>/")
        endforeach()
        if(EXISTS "${MVP_OSG_PLUGIN_DIR}/osgPlugins-3.6.5/osgdb_dae.dll")
            add_custom_command(TARGET echoverse_sonar_lab POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:echoverse_sonar_lab>/osgPlugins-3.6.5"
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${MVP_OSG_PLUGIN_DIR}/osgPlugins-3.6.5/osgdb_dae.dll"
                    "$<TARGET_FILE_DIR:echoverse_sonar_lab>/osgPlugins-3.6.5/osgdb_dae.dll")
        endif()
    endif()
    # Deploy Qt platform plugins for Windows runtime (qwindows.dll).
    set(MVP_QT_PLUGIN_DIR "")
    if(DEFINED VCPKG_INSTALLED_DIR AND DEFINED VCPKG_TARGET_TRIPLET)
        if(Qt6_FOUND AND EXISTS "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/Qt6/plugins")
            set(MVP_QT_PLUGIN_DIR "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/Qt6/plugins")
        elseif(Qt5_FOUND AND EXISTS "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/Qt5/plugins")
            set(MVP_QT_PLUGIN_DIR "${VCPKG_INSTALLED_DIR}/${VCPKG_TARGET_TRIPLET}/Qt5/plugins")
        endif()
    endif()
    if(MVP_QT_PLUGIN_DIR)
        add_custom_command(TARGET echoverse_sonar_lab POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:echoverse_sonar_lab>/platforms"
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${MVP_QT_PLUGIN_DIR}/platforms"
                "$<TARGET_FILE_DIR:echoverse_sonar_lab>/platforms")
        if(EXISTS "${MVP_QT_PLUGIN_DIR}/platforms/qwindows.dll")
            add_custom_command(TARGET echoverse_sonar_lab POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${MVP_QT_PLUGIN_DIR}/platforms/qwindows.dll"
                    "$<TARGET_FILE_DIR:echoverse_sonar_lab>/platforms/qwindows.dll")
        endif()
    endif()
    if(MSVC)
        target_compile_options(echoverse_sonar_lab PRIVATE /FI"${CMAKE_CURRENT_LIST_DIR}/src/rock_windows_compat.hpp" /W3)
    else()
        target_compile_options(echoverse_sonar_lab PRIVATE -include "${CMAKE_CURRENT_LIST_DIR}/src/rock_windows_compat.hpp" -Wall -Wextra -Wpedantic)
    endif()

    add_executable(esl_launcher
        src/esl_launcher.cpp
        src/AppConfig.cpp
    )
    if(WIN32)
        target_sources(esl_launcher PRIVATE src/resources/esl_launcher.rc)
    endif()
    target_include_directories(esl_launcher PRIVATE "${MVP_REPO_ROOT}/src")
    target_link_libraries(esl_launcher PRIVATE ${MVP_QT_WIDGETS_LIB})
    target_compile_definitions(esl_launcher PRIVATE STANDALONE_SIMULATION_DIR="${MVP_REPO_ROOT_CMAKE}")
    if(MSVC)
        target_compile_options(esl_launcher PRIVATE /W3)
    else()
        target_compile_options(esl_launcher PRIVATE -Wall -Wextra -Wpedantic)
    endif()
    if(MVP_QT_PLUGIN_DIR)
        add_custom_command(TARGET esl_launcher POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E make_directory "$<TARGET_FILE_DIR:esl_launcher>/platforms"
            COMMAND ${CMAKE_COMMAND} -E copy_directory
                "${MVP_QT_PLUGIN_DIR}/platforms"
                "$<TARGET_FILE_DIR:esl_launcher>/platforms")
        if(EXISTS "${MVP_QT_PLUGIN_DIR}/platforms/qwindows.dll")
            add_custom_command(TARGET esl_launcher POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${MVP_QT_PLUGIN_DIR}/platforms/qwindows.dll"
                    "$<TARGET_FILE_DIR:esl_launcher>/platforms/qwindows.dll")
        endif()
    endif()
    else()
        message(STATUS "Skipping echoverse_sonar_lab (BUILD_MULTIBEAM_GUI=OFF)")
    endif()
endif()

# ---- Scheme A: "green folder" packaging (EXE + deps + uwmodels) ----
option(MVP_ENABLE_GREEN_PACKAGE "Create a relocatable green folder ZIP via cmake --install + CPack" ON)
set(MVP_GREEN_PACKAGE_CONFIG "Release" CACHE STRING "CMake build config used for packaging")

if(MVP_ENABLE_GREEN_PACKAGE)
    # ZIP package only; easiest distribution.
    set(CPACK_GENERATOR "ZIP")
    set(CPACK_PACKAGE_NAME "echoverse_sonar_lab_green")
    set(CPACK_PACKAGE_VERSION "0")
    set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
    set(CPACK_FILE_NAME "${CPACK_PACKAGE_NAME}-${CPACK_PACKAGE_VERSION}-${CMAKE_SYSTEM_NAME}-${MVP_GREEN_PACKAGE_CONFIG}")
    set(CPACK_PACKAGE_FILE_NAME "${CPACK_FILE_NAME}")

    # Copy the already-deployed GUI runtime tree (EXE + Qt/OSG DLLs) from the build dir.
    # Note: the build dir path is known at configure time; install prefix is provided by `cmake --install --prefix`.
    if(TARGET echoverse_sonar_lab)
        # Use install(DIRECTORY ...) so --prefix is honored by CMake.
        install(DIRECTORY "${CMAKE_BINARY_DIR}/${MVP_GREEN_PACKAGE_CONFIG}/"
            DESTINATION "."
        )
    endif()

    # Add required runtime assets.
    # - uwmodels/: SDF models, dae meshes, textures, materials/scripts
    install(DIRECTORY "${MVP_REPO_ROOT}/uwmodels" DESTINATION "." )

    include(CPack)
endif()
