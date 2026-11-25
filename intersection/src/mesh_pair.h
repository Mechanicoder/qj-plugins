#pragma once

#include "mesh_inter.h"

#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include <cstddef>
#include <string>
#include <vector>

namespace mmi {

// 两组面的三角网格缓存。剖分只做一次，移动时直接平移节点后重新求交，
// 因此拖动位移不会重复剖分。两组面按组内顺序一一对应缓存网格。
struct FacePairMeshes
{
    std::vector<TopoDS_Face>         firstFaces;
    std::vector<TopoDS_Face>         secondFaces;
    std::vector<mesh_inter::TriMesh> firstMeshes;
    std::vector<mesh_inter::TriMesh> secondMeshes;
    TopoDS_Shape                     firstCompound;   // 用于移动面预览
    TopoDS_Shape                     secondCompound;
    double                           linearDeflection  = 0.05;
    double                           angularDeflection = 0.35;
    bool                             ready             = false;
};

void clear(FacePairMeshes& meshes);

// 按弦高/转角误差剖分两组面并展平为世界坐标网格。
bool tessellate(const std::vector<TopoDS_Face>& first,
                const std::vector<TopoDS_Face>& second,
                double linearDeflection, double angularDeflection,
                FacePairMeshes& meshes, std::string& error);

// 与缓存顺序无关地比较两组面是否相同。
bool sameFaceLists(const std::vector<TopoDS_Face>& left,
                   const std::vector<TopoDS_Face>& right);

std::size_t triangleCount(const std::vector<mesh_inter::TriMesh>& meshes);

struct IntersectionRun
{
    mesh_inter::MeshIntersectResult result;
    double                          milliseconds = 0.0;
};

// 把其中一组面的网格整体平移后与另一组面两两求交，结果合并。
IntersectionRun intersect(const FacePairMeshes& meshes, bool moveSecond,
                          double dx, double dy, double dz,
                          double distanceTolerance);

// 把缝合后的折线转成可显示、可更新的模型（由线框组成的复合体）。
TopoDS_Shape polylinesShape(const std::vector<mesh_inter::MeshPolyline>& polylines);

} // namespace mmi
