#pragma once

#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include <string>
#include <vector>

namespace sfi {

// 两组面的精确求交（SMLib FaceIntersect）。组内面缓存为复合体，用于移动面预览。
struct FaceGroup
{
    std::vector<TopoDS_Face> firstFaces;
    std::vector<TopoDS_Face> secondFaces;
    TopoDS_Shape             firstCompound;
    TopoDS_Shape             secondCompound;
    double                   tolerance      = 1.0e-6;
    double                   angleTolerance = 20.0; // 度
    bool                     ready          = false;
};

void clear(FaceGroup& group);

bool prepare(const std::vector<TopoDS_Face>& first,
             const std::vector<TopoDS_Face>& second,
             double tolerance, double angleTolerance,
             FaceGroup& group, std::string& error);

bool sameFaceLists(const std::vector<TopoDS_Face>& left,
                   const std::vector<TopoDS_Face>& right);

struct FaceIntersectionRun
{
    TopoDS_Shape shape;                 // 合并后的交线与孤立交点
    int          edges           = 0;
    int          vertices        = 0;
    int          succeededPairs  = 0;
    int          failedPairs     = 0;
    int          skippedPairs    = 0;   // 同一面（自交退化）
    double       kernelMilliseconds = 0.0;
    double       totalMilliseconds  = 0.0;
    std::string  message;
};

// 把其中一组面整体平移后与另一组面逐对求交，结果合并。
FaceIntersectionRun intersect(const FaceGroup& group, bool moveSecond,
                              double dx, double dy, double dz);

} // namespace sfi
