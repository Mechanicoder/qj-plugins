include_guard(GLOBAL)

set(QJCAM_SMLIB_ROOT "${PROJECT_SOURCE_DIR}/3rdparty/SMLib-10.0.0-localfixes-win64-release" CACHE PATH "SMLib SDK")
set(QJCAM_BRIDGE_SOURCE_DIR "${PROJECT_SOURCE_DIR}/3rdparty/occ-sm-bridge-2.0.0-source/occ_sm_bridge" CACHE PATH "OccSmBridge source rebuilt against host OCC")
find_package(SMLib 10 CONFIG REQUIRED PATHS "${QJCAM_SMLIB_ROOT}/lib/cmake/SMLib" NO_DEFAULT_PATH)
set(OCC_ROOT "${QJCAM_OCC_DIR}")
# The supplied precompiled bridge uses a different OCC ABI. Rebuild the
# independent source snapshot with the exact headers/libs used by QJCAM.
set(OCC_SM_BRIDGE_BUILD_SHARED OFF CACHE BOOL "Build bridge DLL" FORCE)
set(OCC_SM_BRIDGE_BUILD_TESTS OFF CACHE BOOL "Build bridge tests" FORCE)
set(OCC_SM_BRIDGE_BUILD_TOOLS OFF CACHE BOOL "Build bridge tools" FORCE)
set(OCC_SM_BRIDGE_MSVC_RUNTIME_LIBRARY MultiThreadedDLL)
set(OCC_SM_BRIDGE_STAGING_SOURCE_DIR "${QJCAM_BRIDGE_SOURCE_DIR}/third_party/usd-brep/source")
add_subdirectory("${QJCAM_BRIDGE_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/thirdparty/OccSmBridge" EXCLUDE_FROM_ALL)

function(qjcam_use_smlib target)
    target_link_libraries(${target} PRIVATE OccSmBridge::Native)
    # The SDK headers mark consumers dllexport. Use the bridge's private import
    # view instead of accidentally re-exporting SMLib classes from the plugin.
    occ_sm_bridge_smlib_imports(${target})
    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:SMLib::SMLib>" "$<TARGET_FILE:TBB::tbb>" "$<TARGET_FILE:TBB::tbbmalloc>"
            "$<TARGET_FILE_DIR:${target}>"
        VERBATIM)
endfunction()
