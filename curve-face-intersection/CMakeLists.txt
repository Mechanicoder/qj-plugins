include(QjcamSMLib)
qjcam_add_plugin(curve-face-intersection
    SOURCES src/plugin.cpp src/intersection_dialog.h src/intersection_dialog.cpp
        src/intersection_algorithm.h src/intersection_algorithm.cpp src/validation.h src/validation.cpp
    OCC_LIBRARIES TKBRep TKTopAlgo TKBO TKBool TKGeomAlgo TKGeomBase TKG3d)
qjcam_use_smlib(curve-face-intersection)

option(QJCAM_BUILD_CFI_TESTS "Build curve-face validation tests" OFF)
if(QJCAM_BUILD_CFI_TESTS)
    add_executable(cfi-validation-tests tests/validation_tests.cpp src/validation.cpp src/intersection_algorithm.cpp)
    target_compile_features(cfi-validation-tests PRIVATE cxx_std_17)
    target_compile_options(cfi-validation-tests PRIVATE /utf-8)
    target_include_directories(cfi-validation-tests PRIVATE src)
    target_link_libraries(cfi-validation-tests PRIVATE Qjcam::SDK)
    foreach(lib IN ITEMS TKBRep TKTopAlgo TKBO TKBool TKGeomAlgo TKGeomBase TKG3d)
        target_link_libraries(cfi-validation-tests PRIVATE "${QJCAM_OCC_DIR}/lib/${lib}.lib")
    endforeach()
    set_target_properties(cfi-validation-tests PROPERTIES MSVC_RUNTIME_LIBRARY MultiThreadedDLL
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests/$<CONFIG>")
    qjcam_use_smlib(cfi-validation-tests)
endif()
