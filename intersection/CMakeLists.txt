include(QjcamSMLib)

# 面面求交算法来自 SoulCAD 的 kernels/smlib；私有仓库通过本机 GitHub 凭据拉取。
set(QJCAM_SOULCAD_SOURCE_DIR "" CACHE PATH "Local SoulCAD checkout (optional, skips FetchContent)")
if(QJCAM_SOULCAD_SOURCE_DIR)
    add_subdirectory("${QJCAM_SOULCAD_SOURCE_DIR}/kernels/smlib"
                     "${CMAKE_BINARY_DIR}/soulcad-smlib" EXCLUDE_FROM_ALL)
else()
    include(FetchContent)
    FetchContent_Declare(SoulCAD
        GIT_REPOSITORY https://github.com/Mechanicoder/SoulCAD.git
        GIT_TAG 20a8876dd7a7186492326a85a24921dfff1a29fc
        GIT_SHALLOW TRUE
        GIT_PROGRESS TRUE
        SOURCE_SUBDIR kernels/smlib)
    FetchContent_MakeAvailable(SoulCAD)
endif()

qjcam_add_plugin(intersection
    SOURCES
        src/plugin.cpp
        src/face_compare_algorithm.h
        src/face_compare_algorithm.cpp
        src/face_compare_validation.h
        src/face_compare_validation.cpp
        src/face_compare_dialog.h
        src/face_compare_dialog.cpp
        src/face_realtime.h
        src/face_realtime.cpp
        src/face_dialog.h
        src/face_dialog.cpp
        src/mesh_inter.h
        src/mesh_inter.cpp
        src/mesh_pair.h
        src/mesh_pair.cpp
        src/mesh_dialog.h
        src/mesh_dialog.cpp
        resources.qrc
    OCC_LIBRARIES TKBRep TKTopAlgo TKBO TKBool TKGeomAlgo TKGeomBase TKG3d TKMesh)
target_include_directories(intersection PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
target_link_libraries(intersection PRIVATE SoulCAD::SmlibFaceIntersection)
set_target_properties(intersection PROPERTIES AUTORCC ON)
qjcam_use_smlib(intersection)

option(QJCAM_BUILD_INTERSECTION_TESTS "Build face-face comparison smoke tests" OFF)
if(QJCAM_BUILD_INTERSECTION_TESTS)
    add_executable(intersection-tests
        tests/face_compare_tests.cpp
        src/face_compare_algorithm.cpp
        src/face_compare_validation.cpp)
    target_compile_features(intersection-tests PRIVATE cxx_std_17)
    target_compile_options(intersection-tests PRIVATE /utf-8)
    target_include_directories(intersection-tests PRIVATE src)
    target_link_libraries(intersection-tests PRIVATE Qjcam::SDK SoulCAD::SmlibFaceIntersection
        "${QJCAM_OCC_DIR}/lib/TKBRep.lib" "${QJCAM_OCC_DIR}/lib/TKTopAlgo.lib"
        "${QJCAM_OCC_DIR}/lib/TKBO.lib" "${QJCAM_OCC_DIR}/lib/TKBool.lib"
        "${QJCAM_OCC_DIR}/lib/TKGeomAlgo.lib" "${QJCAM_OCC_DIR}/lib/TKGeomBase.lib"
        "${QJCAM_OCC_DIR}/lib/TKG3d.lib")
    set_target_properties(intersection-tests PROPERTIES MSVC_RUNTIME_LIBRARY MultiThreadedDLL
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests/$<CONFIG>")
    qjcam_use_smlib(intersection-tests)

    add_executable(intersection-face-tests
        tests/face_realtime_tests.cpp
        src/face_realtime.cpp
        src/face_compare_algorithm.cpp)
    target_compile_features(intersection-face-tests PRIVATE cxx_std_17)
    target_compile_options(intersection-face-tests PRIVATE /utf-8)
    target_include_directories(intersection-face-tests PRIVATE src)
    target_link_libraries(intersection-face-tests PRIVATE Qjcam::SDK SoulCAD::SmlibFaceIntersection
        "${QJCAM_OCC_DIR}/lib/TKBRep.lib" "${QJCAM_OCC_DIR}/lib/TKTopAlgo.lib"
        "${QJCAM_OCC_DIR}/lib/TKBO.lib" "${QJCAM_OCC_DIR}/lib/TKBool.lib"
        "${QJCAM_OCC_DIR}/lib/TKGeomAlgo.lib" "${QJCAM_OCC_DIR}/lib/TKGeomBase.lib"
        "${QJCAM_OCC_DIR}/lib/TKG3d.lib")
    set_target_properties(intersection-face-tests PROPERTIES MSVC_RUNTIME_LIBRARY MultiThreadedDLL
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/tests/$<CONFIG>")
    qjcam_use_smlib(intersection-face-tests)
endif()
