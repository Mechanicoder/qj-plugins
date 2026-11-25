include_guard(GLOBAL)

# NAME is both the DLL basename and the deployment subdirectory name.
function(qjcam_add_plugin name)
    cmake_parse_arguments(PLUGIN "" "" "SOURCES;OCC_LIBRARIES" ${ARGN})
    if(PLUGIN_UNPARSED_ARGUMENTS OR NOT PLUGIN_SOURCES)
        message(FATAL_ERROR "qjcam_add_plugin(${name}): supply SOURCES and optional OCC_LIBRARIES")
    endif()
    add_library(${name} SHARED ${PLUGIN_SOURCES})
    target_compile_features(${name} PRIVATE cxx_std_17)
    target_compile_options(${name} PRIVATE /utf-8 /W4 /permissive-)
    target_compile_definitions(${name} PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
    target_link_libraries(${name} PRIVATE Qjcam::SDK)
    foreach(lib IN LISTS PLUGIN_OCC_LIBRARIES)
        if(NOT EXISTS "${QJCAM_OCC_DIR}/lib/${lib}.lib")
            message(FATAL_ERROR "Missing OpenCascade library: ${lib}")
        endif()
        target_link_libraries(${name} PRIVATE "${QJCAM_OCC_DIR}/lib/${lib}.lib")
    endforeach()
    set_target_properties(${name} PROPERTIES
        AUTOMOC ON
        PREFIX ""
        MSVC_RUNTIME_LIBRARY MultiThreadedDLL
        RUNTIME_OUTPUT_DIRECTORY "${QJCAM_ROOT}/plugins/${name}"
        PDB_OUTPUT_DIRECTORY "${QJCAM_ROOT}/plugins/${name}"
        ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/$<CONFIG>")
    # QJCAM expects plugins/<name>/<name>.dll, without a configuration subfolder.
    foreach(config IN ITEMS RELEASE RELWITHDEBINFO)
        set_target_properties(${name} PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY_${config} "${QJCAM_ROOT}/plugins/${name}"
            PDB_OUTPUT_DIRECTORY_${config} "${QJCAM_ROOT}/plugins/${name}")
    endforeach()
endfunction()
