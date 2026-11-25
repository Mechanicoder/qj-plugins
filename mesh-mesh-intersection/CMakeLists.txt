qjcam_add_plugin(mesh-mesh-intersection
    SOURCES
        src/plugin.cpp
        src/intersection_dialog.h
        src/intersection_dialog.cpp
        src/mesh_pair.h
        src/mesh_pair.cpp
        mesh_inter.h
        mesh_inter.cpp
    OCC_LIBRARIES TKBRep TKTopAlgo TKMesh)
target_include_directories(mesh-mesh-intersection PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}")
