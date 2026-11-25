#pragma once

#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <string>
#include <vector>

namespace ffi {

struct IntersectionRequest {
    TopoDS_Face first;
    TopoDS_Face second;
    double tolerance = 1.0e-6; // Model length units.
};

enum class IntersectionStatus { NotImplemented, Success, InvalidInput, Failed };

struct IntersectionResult {
    IntersectionStatus status = IntersectionStatus::NotImplemented;
    std::vector<TopoDS_Shape> curves;
    std::string message;
    double kernelMilliseconds = -1; // SMLib FaceIntersect only; -1 if unavailable.
    double conversionMilliseconds = -1; // Both bridge directions and output construction.
};

// OCC -> OccSmBridge -> SMLib FaceIntersect -> OCC, without QJCAM UI calls.
IntersectionResult intersectFaces(const IntersectionRequest& request);

} // namespace ffi
