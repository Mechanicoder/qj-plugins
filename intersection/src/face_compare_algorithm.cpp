#include "face_compare_algorithm.h"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <chrono>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <OccSmBridge/Convert.h>
#include <tbb/global_control.h>
// Keep SDK macros out of UI/validation headers. Include host and STL headers first.
#include <SmBrep.h>
#include <SmFace.h>
#include <SmCurve.h>
#include <SmContext.h>

namespace ffi {

IntersectionResult intersectFaces(const IntersectionRequest& request)
{
    if (request.first.IsNull() || request.second.IsNull()
        || request.first.IsSame(request.second)
        || !std::isfinite(request.tolerance) || request.tolerance <= 0.0
        || !std::isfinite(request.angleTolerance) || request.angleTolerance <= 0.0) {
        return {IntersectionStatus::InvalidInput, {}, "请选择两个不同的有效面，并设置正容差。"};
    }
    const auto start = std::chrono::steady_clock::now();
    tbb::global_control singleThread(tbb::global_control::max_allowed_parallelism, 1);
    OccSmBridge::Options options;
    options.approx_tol = request.tolerance;
    options.enable_healer = false;
    auto first = OccSmBridge::Convert(request.first, options);
    if (!first.valid()) return {IntersectionStatus::Failed, {}, "OCC -> SMLib A: " + first.report().error};
    auto second = OccSmBridge::Convert(request.second, options);
    if (!second.valid()) return {IntersectionStatus::Failed, {}, "OCC -> SMLib B: " + second.report().error};
    const auto& context = *first.context();
    SmTArray<SmFace*> facesA(context), facesB(context);
    first.brep()->GetFaces(facesA);
    second.brep()->GetFaces(facesB);
    if (facesA.GetSize() != 1 || facesB.GetSize() != 1)
        return {IntersectionStatus::Failed, {}, "Bridge must produce exactly one face per input."};

    // Arrays do not own their pointer elements. Clean up even when the SDK
    // returns a failure after producing partial results. Contexts outlive them.
    struct Curves {
        SmTArray<SmCurve*> values;
        explicit Curves(const SmContext& c) : values(c) {}
        ~Curves() { for (ULONG i = 0; i < values.GetSize(); ++i) delete values[i]; }
    } curves(context);
    SmTArray<SmPoint3d> points(context);
    const double angleToleranceDegrees = request.angleTolerance;
    const auto kernelStart = std::chrono::steady_clock::now();
    const auto status = facesA[0]->FaceIntersect(context, facesB[0], request.tolerance,
        angleToleranceDegrees, curves.values, nullptr, nullptr, &points);
    const auto kernelEnd = std::chrono::steady_clock::now();
    if (status != SM_SUCCESS)
        return {IntersectionStatus::Failed, {}, "SMLib FaceIntersect failed: status " + std::to_string(static_cast<int>(status))};

    IntersectionResult result{IntersectionStatus::Success, {}, "SMLib FaceIntersect completed (OccSmBridge round-trip)."};
    // Result BRep owns each consumed curve. It is destroyed before the Context.
    SmBrep wireBrep;
    for (ULONG i = 0; i < curves.values.GetSize(); ++i) {
        auto* curve = curves.values[i];
        if (!curve) throw std::runtime_error("SMLib returned a null intersection curve.");
        const auto interval = curve->GetNaturalInterval();
        SmEdge* edge = nullptr;
        SmPoint3d startPoint, endPoint;
        if (curve->EvaluatePoint(interval.GetMin(), startPoint) != SM_SUCCESS
            || curve->EvaluatePoint(interval.GetMax(), endPoint) != SM_SUCCESS)
            return {IntersectionStatus::Failed, {}, "SMLib curve endpoint evaluation failed."};
        if (startPoint.DistanceBetween(endPoint) <= request.tolerance) {
            // CreateWireEdgeFromCurve glues coincident endpoint vertices and
            // crashes for a closed circle in this SDK. Author a closed edge
            // directly with one shared vertex, preserving the exact curve.
            SmShell* shell = nullptr;
            SmVertex* vertex = nullptr;
            auto* region = wireBrep.GetInfiniteRegion();
            if (wireBrep.MakeShellVertex(region, startPoint, shell, vertex) != SM_SUCCESS)
                return {IntersectionStatus::Failed, {}, "SMLib closed wire vertex creation failed."};
            curves.values[i] = nullptr;
            if (wireBrep.MakeWireEdge(region, vertex, vertex, curve, interval, SM_OT_SAME, edge) != SM_SUCCESS || !edge)
                return {IntersectionStatus::Failed, {}, "SMLib closed wire edge creation failed."};
        } else {
            curves.values[i] = nullptr; // API consumes the curve.
            if (wireBrep.CreateWireEdgeFromCurve(curve, interval, edge) != SM_SUCCESS || !edge)
                return {IntersectionStatus::Failed, {}, "SMLib curve -> wire edge failed."};
        }
    }
    if (curves.values.GetSize() > 0) {
        TopoDS_Shape output;
        std::string error;
        if (!OccSmBridge::ConvertToOcc(wireBrep, output, &error))
            return {IntersectionStatus::Failed, {}, "SMLib -> OCC: " + error};
        result.curves.push_back(output);
    }
    for (ULONG i = 0; i < points.GetSize(); ++i)
        result.curves.push_back(BRepBuilderAPI_MakeVertex(gp_Pnt(points[i].x, points[i].y, points[i].z)).Shape());
    result.kernelMilliseconds = std::chrono::duration<double, std::milli>(kernelEnd - kernelStart).count();
    result.conversionMilliseconds = std::chrono::duration<double, std::milli>(
        (kernelStart - start) + (std::chrono::steady_clock::now() - kernelEnd)).count();
    for (const auto& warning : first.report().warnings) result.message += "\nBridge A: " + warning;
    for (const auto& warning : second.report().warnings) result.message += "\nBridge B: " + warning;
    return result;
}

} // namespace ffi
