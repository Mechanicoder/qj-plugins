cmake_minimum_required(VERSION 3.22)
project(qjcam_plugins VERSION 0.1.0 LANGUAGES CXX)

if(NOT WIN32 OR NOT MSVC)
    message(FATAL_ERROR "QJCAM plugins require Windows and an MSVC-compatible toolchain.")
endif()

if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "Use a 64-bit generator (-A x64).")
endif()

# QJCAM ships release Qt/OCC binaries. Debug's runtime/iterator ABI is incompatible.
if(CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_CONFIGURATION_TYPES "Release;RelWithDebInfo" CACHE STRING "Supported configurations" FORCE)
else()
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
    endif()
    if(NOT CMAKE_BUILD_TYPE MATCHES "^(Release|RelWithDebInfo)$")
        message(FATAL_ERROR "Use Release or RelWithDebInfo, not Debug.")
    endif()
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(QjcamSDK)
include(QjcamPlugin)

# Keep registration explicit: unrelated directories will never become plugins by accident.
option(QJCAM_BUILD_INTERSECTION "Build intersection (face-face and mesh)" ON)
if(QJCAM_BUILD_INTERSECTION)
    add_subdirectory(intersection)
endif()

option(QJCAM_BUILD_CURVE_FACE_INTERSECTION "Build curve-face-intersection" ON)
if(QJCAM_BUILD_CURVE_FACE_INTERSECTION)
    add_subdirectory(curve-face-intersection)
endif()

option(QJCAM_BUILD_XYZ_DRAW "Build xyz-draw" ON)
if(QJCAM_BUILD_XYZ_DRAW)
    add_subdirectory(xyz-draw)
endif()
