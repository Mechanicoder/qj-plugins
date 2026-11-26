include_guard(GLOBAL)

# 预编译包按 OCCT 8.0.0 ABI 构建，而 QJCAM 26Q1 使用 OCCT 7.9.2；
# 因此直接使用包内源码，基于宿主 OCC 重新编译，保持 ABI 一致。
set(QJCAM_OCCSMBRIDGE_ROOT
    "${PROJECT_SOURCE_DIR}/3rdparty/OccSmBridge-2.0.0-win64-msvc-release/OccSmBridge-2.0.0-win64-msvc-release"
    CACHE PATH "OccSmBridge + SMLib SDK package")

set(_qjcam_bridge_sdk "${QJCAM_OCCSMBRIDGE_ROOT}/sdk")
set(_qjcam_bridge_source "${QJCAM_OCCSMBRIDGE_ROOT}/source")
if(NOT EXISTS "${_qjcam_bridge_sdk}/lib/cmake/SMLib/SMLibConfig.cmake"
   OR NOT EXISTS "${_qjcam_bridge_source}/CMakeLists.txt")
    message(FATAL_ERROR "OccSmBridge SDK package not found under ${QJCAM_OCCSMBRIDGE_ROOT}")
endif()

find_package(SMLib 10 CONFIG REQUIRED PATHS "${_qjcam_bridge_sdk}/lib/cmake/SMLib" NO_DEFAULT_PATH)
set(OCC_ROOT "${QJCAM_OCC_DIR}")
set(TBB_ROOT "${_qjcam_bridge_sdk}/dependencies/tbb")

set(OCC_SM_BRIDGE_BUILD_SHARED OFF CACHE BOOL "Build bridge DLL" FORCE)
set(OCC_SM_BRIDGE_BUILD_TESTS OFF CACHE BOOL "Build bridge tests" FORCE)
set(OCC_SM_BRIDGE_BUILD_TOOLS OFF CACHE BOOL "Build bridge tools" FORCE)
set(OCC_SM_BRIDGE_WITH_SMLIB ON CACHE BOOL "Build SMLib conversion" FORCE)
set(OCC_SM_BRIDGE_MSVC_RUNTIME_LIBRARY MultiThreadedDLL CACHE STRING "Bridge runtime" FORCE)
set(OCC_SM_BRIDGE_STAGING_SOURCE_DIR
    "${_qjcam_bridge_source}/third_party/usd-brep/source" CACHE PATH "USD BREP staging source" FORCE)

add_subdirectory("${_qjcam_bridge_source}" "${CMAKE_BINARY_DIR}/thirdparty/OccSmBridge" EXCLUDE_FROM_ALL)

function(qjcam_use_smlib target)
    target_link_libraries(${target} PRIVATE OccSmBridge::Native)
    # The SDK headers mark consumers dllexport. Use the bridge's private import
    # view instead of accidentally re-exporting SMLib classes from the plugin.
    occ_sm_bridge_smlib_imports(${target})
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_qjcam_bridge_sdk}/bin/SMLib.dll"
            "${_qjcam_bridge_sdk}/bin/tbb12.dll"
            "${_qjcam_bridge_sdk}/bin/tbbmalloc.dll"
            "${_qjcam_bridge_sdk}/bin/jemalloc.dll"
            "$<TARGET_FILE_DIR:${target}>"
        VERBATIM)
endfunction()
