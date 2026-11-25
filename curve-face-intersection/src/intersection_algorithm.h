#pragma once

#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <string>
#include <vector>

namespace cfi {

struct IntersectionRequest {
    TopoDS_Edge first;
    TopoDS_Face second;
    double tolerance = 1.0e-6; // Model length units.
};

enum class IntersectionStatus { NotImplemented, Success, InvalidInput, Failed };

struct IntersectionResult {
    IntersectionStatus status = IntersectionStatus::NotImplemented;
    std::vector<TopoDS_Shape> curves;
    std::string message;
    double kernelMilliseconds = -1; // SMLib intersection + native trim/classification.
    double conversionMilliseconds = -1; // Both bridge directions and output construction.
};

// OCC -> OccSmBridge -> SMLib GlobalCurveIntersect -> OCC, without QJCAM UI calls.
IntersectionResult intersectCurveFace(const IntersectionRequest& request);

} // namespace cfi

