#include "face_realtime.h"

#include <intersect_face_face.h>

#include <BRepBuilderAPI_Transform.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace sfi {
namespace {

constexpr double kOffsetEpsilon = 1e-12;

TopoDS_Shape facesCompound(const std::vector<TopoDS_Face>& faces)
{
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    for (const TopoDS_Face& face : faces) builder.Add(compound, face);
    return compound;
}

} // namespace

void clear(FaceGroup& group)
{
    group.firstFaces.clear();
    group.secondFaces.clear();
    group.firstCompound = TopoDS_Shape();
    group.secondCompound = TopoDS_Shape();
    group.ready = false;
}

bool prepare(const std::vector<TopoDS_Face>& first,
             const std::vector<TopoDS_Face>& second,
             double tolerance, double angleTolerance,
             FaceGroup& group, std::string& error)
{
    clear(group);
    error.clear();
    if (first.empty() || second.empty()) {
        error = "面 A、面 B 至少各选择一个面。";
        return false;
    }
    for (const TopoDS_Face& face : first) {
        if (face.IsNull()) {
            error = "面 A 中存在无效面。";
            return false;
        }
    }
    for (const TopoDS_Face& face : second) {
        if (face.IsNull()) {
            error = "面 B 中存在无效面。";
            return false;
        }
    }
    group.firstFaces = first;
    group.secondFaces = second;
    group.firstCompound = facesCompound(first);
    group.secondCompound = facesCompound(second);
    group.tolerance = tolerance;
    group.angleTolerance = angleTolerance;
    group.ready = true;
    return true;
}

bool sameFaceLists(const std::vector<TopoDS_Face>& left, const std::vector<TopoDS_Face>& right)
{
    if (left.size() != right.size()) return false;
    std::vector<bool> matched(right.size(), false);
    for (const TopoDS_Face& face : left) {
        bool found = false;
        for (std::size_t i = 0; i < right.size(); ++i) {
            if (!matched[i] && face.IsSame(right[i])) {
                matched[i] = true;
                found = true;
                break;
            }
        }
        if (!found) return false;
    }
    return true;
}

FaceIntersectionRun intersect(const FaceGroup& group, bool moveSecond,
                              double dx, double dy, double dz)
{
    FaceIntersectionRun run;
    if (!group.ready) return run;

    const bool hasOffset = std::abs(dx) > kOffsetEpsilon
        || std::abs(dy) > kOffsetEpsilon || std::abs(dz) > kOffsetEpsilon;
    gp_Trsf translation;
    if (hasOffset) translation.SetTranslation(gp_Vec(dx, dy, dz));

    const std::vector<TopoDS_Face>& moving = moveSecond ? group.secondFaces : group.firstFaces;
    const std::vector<TopoDS_Face>& other = moveSecond ? group.firstFaces : group.secondFaces;
    std::vector<TopoDS_Face> placed;
    placed.reserve(moving.size());
    for (const TopoDS_Face& face : moving) {
        placed.push_back(hasOffset
            ? TopoDS::Face(BRepBuilderAPI_Transform(face, translation, Standard_False).Shape())
            : face);
    }

    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);

    sm::FaceIntersectionOptions options;
    options.intersectionToleranceMm = group.tolerance;
    options.simplificationToleranceMm = std::max(1.0e-6, group.tolerance * 10.0);
    options.angleToleranceDegrees = group.angleTolerance;
    options.simplifyCurves = true;

    const auto start = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < placed.size(); ++i) {
        for (std::size_t j = 0; j < other.size(); ++j) {
            // 同一面自身求交是退化情形，跳过以避免无效结果。
            if (placed[i].IsSame(other[j])) {
                ++run.skippedPairs;
                continue;
            }
            const auto pairStart = std::chrono::steady_clock::now();
            const sm::FaceIntersection part = sm::IntersectFaceFace(placed[i], other[j], options);
            run.kernelMilliseconds += std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - pairStart).count();
            if (!part.ok) {
                ++run.failedPairs;
                if (run.message.empty()) run.message = part.error;
                continue;
            }
            ++run.succeededPairs;
            run.edges += static_cast<int>(part.edges.size());
            run.vertices += static_cast<int>(part.points.size());
            if (!part.shape.IsNull() && (!part.edges.empty() || !part.points.empty()))
                builder.Add(compound, part.shape);
        }
    }
    run.totalMilliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();

    if (run.edges > 0 || run.vertices > 0) run.shape = compound;
    return run;
}

} // namespace sfi
