#include "mesh_pair.h"

#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Failure.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <utility>

namespace mmi {
namespace {

bool buildMesh(const TopoDS_Face& face, double linearDeflection, double angularDeflection,
               mesh_inter::TriMesh& mesh, std::string& error)
{
    try {
        BRepMesh_IncrementalMesh mesher(face, linearDeflection, Standard_False,
                                        angularDeflection, Standard_True);
        mesher.Perform();
    } catch (const Standard_Failure& failure) {
        const char* message = failure.GetMessageString();
        error = (message != nullptr && message[0] != '\0') ? message : "面剖分失败。";
        return false;
    }
    TopLoc_Location location;
    Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);
    if (triangulation.IsNull() || triangulation->NbTriangles() <= 0) {
        error = "面没有可用的三角剖分。";
        return false;
    }
    mesh = mesh_inter::TriMeshFromTriangulation(*triangulation, location);
    if (mesh.tris.empty()) {
        error = "面的三角剖分为空。";
        return false;
    }
    return true;
}

TopoDS_Shape facesCompound(const std::vector<TopoDS_Face>& faces)
{
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    for (const TopoDS_Face& face : faces) builder.Add(compound, face);
    return compound;
}

bool buildGroup(const std::vector<TopoDS_Face>& faces, const char* label,
                double linearDeflection, double angularDeflection,
                std::vector<mesh_inter::TriMesh>& meshes, std::string& error)
{
    meshes.clear();
    meshes.reserve(faces.size());
    for (std::size_t i = 0; i < faces.size(); ++i) {
        mesh_inter::TriMesh mesh;
        std::string faceError;
        if (!buildMesh(faces[i], linearDeflection, angularDeflection, mesh, faceError)) {
            error = std::string(label) + " 第 " + std::to_string(i + 1) + " 个面剖分失败：" + faceError;
            return false;
        }
        meshes.push_back(std::move(mesh));
    }
    return true;
}

void addStats(mesh_inter::MeshIntersectStats& total, const mesh_inter::MeshIntersectStats& part)
{
    total.bvhPairs       += part.bvhPairs;
    total.candidatePairs += part.candidatePairs;
    total.boxRejected    += part.boxRejected;
    total.triPairs       += part.triPairs;
    total.degenerateTris += part.degenerateTris;
    total.coplanarPairs  += part.coplanarPairs;
    total.pointHits      += part.pointHits;
    total.rawSegments    += part.rawSegments;
    total.duplicates     += part.duplicates;
}

} // namespace

void clear(FacePairMeshes& meshes)
{
    meshes.firstFaces.clear();
    meshes.secondFaces.clear();
    meshes.firstMeshes.clear();
    meshes.secondMeshes.clear();
    meshes.firstCompound = TopoDS_Shape();
    meshes.secondCompound = TopoDS_Shape();
    meshes.ready = false;
}

bool tessellate(const std::vector<TopoDS_Face>& first,
                const std::vector<TopoDS_Face>& second,
                double linearDeflection, double angularDeflection,
                FacePairMeshes& meshes, std::string& error)
{
    clear(meshes);
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
    if (!buildGroup(first, "面 A", linearDeflection, angularDeflection, meshes.firstMeshes, error)) return false;
    if (!buildGroup(second, "面 B", linearDeflection, angularDeflection, meshes.secondMeshes, error)) return false;

    meshes.firstFaces = first;
    meshes.secondFaces = second;
    meshes.firstCompound = facesCompound(first);
    meshes.secondCompound = facesCompound(second);
    meshes.linearDeflection = linearDeflection;
    meshes.angularDeflection = angularDeflection;
    meshes.ready = true;
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

std::size_t triangleCount(const std::vector<mesh_inter::TriMesh>& meshes)
{
    std::size_t total = 0;
    for (const mesh_inter::TriMesh& mesh : meshes) total += mesh.TriangleCount();
    return total;
}

IntersectionRun intersect(const FacePairMeshes& meshes, bool moveSecond,
                          double dx, double dy, double dz,
                          double distanceTolerance)
{
    IntersectionRun run;
    if (!meshes.ready) return run;

    mesh_inter::MeshIntersectOptions options;
    options.distanceTol = distanceTolerance;
    // 缝合半径至少保持默认值，避免容差调小后折线被拆成碎片。
    options.mergeTol = std::max(distanceTolerance, 1e-6);

    std::vector<mesh_inter::TriMesh> moved = moveSecond ? meshes.secondMeshes : meshes.firstMeshes;
    for (mesh_inter::TriMesh& mesh : moved) {
        for (gp_XYZ& point : mesh.nodes) {
            point.SetX(point.X() + dx);
            point.SetY(point.Y() + dy);
            point.SetZ(point.Z() + dz);
        }
    }
    const std::vector<mesh_inter::TriMesh>& otherMeshes = moveSecond ? meshes.firstMeshes : meshes.secondMeshes;

    const auto start = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < moved.size(); ++i) {
        for (std::size_t j = 0; j < otherMeshes.size(); ++j) {
            // 允许两侧选到同一个面：共面情形下输出重叠区域的边界。
            const mesh_inter::MeshIntersectResult part =
                mesh_inter::IntersectMeshes(moved[i], otherMeshes[j], options);
            run.result.bboxOverlap = run.result.bboxOverlap || part.bboxOverlap;
            run.result.segments.insert(run.result.segments.end(),
                                       part.segments.begin(), part.segments.end());
            run.result.polylines.insert(run.result.polylines.end(),
                                        part.polylines.begin(), part.polylines.end());
            addStats(run.result.stats, part.stats);
        }
    }
    run.milliseconds = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    return run;
}

TopoDS_Shape polylinesShape(const std::vector<mesh_inter::MeshPolyline>& polylines)
{
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    for (const auto& polyline : polylines) {
        if (polyline.points.size() < 2) continue;
        BRepBuilderAPI_MakePolygon polygon;
        for (const gp_XYZ& point : polyline.points) polygon.Add(gp_Pnt(point));
        if (polyline.closed) polygon.Close();
        if (polygon.IsDone()) builder.Add(compound, polygon.Wire());
    }
    return compound;
}

} // namespace mmi
