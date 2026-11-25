include(QjcamSMLib)
qjcam_add_plugin(face-face-intersection
    SOURCES
        src/plugin.cpp
        src/intersection_dialog.h
        src/intersection_dialog.cpp
        src/intersection_algorithm.h
        src/intersection_algorithm.cpp
        src/validation.h
        src/validation.cpp
    OCC_LIBRARIES TKBRep TKTopAlgo TKBO TKBool TKGeomAlgo TKGeomBase TKG3d)
qjcam_use_smlib(face-face-intersection)

option(QJCAM_BUILD_FFI_TESTS "Build face-face validation smoke tests" OFF)
if(QJCAM_BUILD_FFI_TESTS)
    add_executable(ffi-validation-tests tests/validation_tests.cpp src/validation.cpp src/intersection_algorithm.cpp)
    target_compile_features(ffi-validation-tests PRIVATE cxx_std_17)
    target_compile_options(ffi-validation-tests PRIVATE /utf-8)
    target_include_directories(ffi-validation-tests PRIVATE src)
    target_link_libraries(ffi-validation-tests PRIVATE Qjcam::SDK
        "${QJCAM_OCC_DIR}/lib/TKBRep.lib" "${QJCAM_OCC_DIR}/lib/TKTopAlgo.lib"
        "${QJCAM_OCC_DIR}/lib/TKBO.lib" "${QJCAM_OCC_DIR}/lib/TKBool.lib"
        "${QJCAM_OCC_DIR}/lib/TKGeomAlgo.lib" "${QJCAM_OCC_DIR}/lib/TKGeomBase.lib"
        "${QJCAM_OCC_DIR}/lib/TKG3d.lib")
    set_target_properties(ffi-validation-tests PROPERTIES MSVC_RUNTIME_LIBRARY MultiThreadedDLL
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests/$<CONFIG>")
    qjcam_use_smlib(ffi-validation-tests)
endif()
