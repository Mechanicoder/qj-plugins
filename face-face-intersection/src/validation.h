#pragma once
#include "intersection_algorithm.h"
#include <QString>
#include <functional>

namespace ffi {
struct ShapeMetrics {
    int edges = 0;
    int vertices = 0;
    double length = 0;
    bool valid = false;
};
struct AlgorithmRun {
    IntersectionResult result;
    double milliseconds = 0;
};
struct ValidationReport {
    AlgorithmRun occ;
    AlgorithmRun custom;
    ShapeMetrics occMetrics;
    ShapeMetrics customMetrics;
    double sampledDeviation = -1; // -1: unavailable; model length units.
    QString text;
};
IntersectionRequest readCase(const QString& directory, double tolerance);
TopoDS_Shape combineResults(const IntersectionResult& result);
using IntersectionAlgorithm = std::function<IntersectionResult(const IntersectionRequest&)>;
ValidationReport validate(const IntersectionRequest& request, const IntersectionAlgorithm& customAlgorithm = intersectFaces);
} // namespace ffi
