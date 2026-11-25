#include "intersection_algorithm.h"
#include <cmath>
#include <chrono>
#include <memory>
#include <algorithm>
#include <stdexcept>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRep_Tool.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <GeomConvert.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Geom_BSplineCurve.hxx>
#include <OccSmBridge/Convert.h>
#include <tbb/global_control.h>
#include <NL_Geometry.h>
#include <SmBrep.h>
#include <SmFace.h>
#include <SmEdge.h>
#include <SmCurve.h>
#include <SmSurface.h>
#include <SmContext.h>
#include <SmSolutionArray.h>
#include <SmPointClass.h>
#include <SmBSplineCurve.h>

namespace cfi {
namespace {
void check(SmStatus status, const char* operation)
{
    if (status != SM_SUCCESS) throw std::runtime_error(std::string(operation) + ": status " + std::to_string(static_cast<int>(status)));
}
struct OwnedCurves {
    SmTArray<SmCurve*> values;
    explicit OwnedCurves(const SmContext& context) : values(context) {}
    ~OwnedCurves() { for (ULONG i=0; i<values.GetSize(); ++i) delete values[i]; }
};
}
IntersectionResult intersectCurveFace(const IntersectionRequest& request)
{
    if (request.first.IsNull() || request.second.IsNull() || !std::isfinite(request.tolerance) || request.tolerance <= 0)
        return {IntersectionStatus::InvalidInput, {}, "Select a finite edge and a face with positive tolerance."};
    using Clock = std::chrono::steady_clock;
    const auto start = Clock::now();
    tbb::global_control singleThread(tbb::global_control::max_allowed_parallelism,1);
    OccSmBridge::Options options; options.approx_tol=request.tolerance; options.enable_healer=false;
    auto faceModel = OccSmBridge::Convert(request.second,options);
    if (!faceModel.valid()) return {IntersectionStatus::Failed, {}, "Face bridge: " + faceModel.report().error};
    const auto& context = *faceModel.context();
    SmTArray<SmFace*> faces(context); faceModel.brep()->GetFaces(faces);
    if (faces.GetSize()!=1) throw std::runtime_error("Bridge must produce one face.");
    // The bridge's staging importer does not accept a standalone edge root.
    // Transfer the bounded curve exactly as rational NURBS (no sampling/fit).
    double first=0,last=0;
    const auto geometry=BRep_Tool::Curve(request.first,first,last); // Includes edge location.
    if (geometry.IsNull() || !std::isfinite(first) || !std::isfinite(last)
        || last<=first || std::abs(first)>1e50 || std::abs(last)>1e50)
        throw std::runtime_error("Only finite, non-degenerate curve edges are supported.");
    const auto bs=GeomConvert::CurveToBSplineCurve(new Geom_TrimmedCurve(geometry,first,last));
    std::vector<gw_CPOINT> poles(static_cast<size_t>(bs->NbPoles()));
    std::vector<double> knots;
    for(int i=1;i<=bs->NbPoles();++i) {
        const auto p=bs->Pole(i); const auto w=bs->Weight(i);
        auto& pole=poles[static_cast<size_t>(i-1)];
        pole.x=p.X()*w; pole.y=p.Y()*w; pole.z=p.Z()*w; pole.w=w;
    }
    for(int i=1;i<=bs->NbKnots();++i) for(int j=0;j<bs->Multiplicity(i);++j) knots.push_back(bs->Knot(i));
    std::unique_ptr<SmCurve> ownedCurve(new(context) SmBSplineCurve(3,static_cast<ULONG>(poles.size()-1),
        static_cast<ULONG>(knots.size()-1),static_cast<short>(bs->Degree()),poles.data(),knots.data(),&context));
    auto* curve=ownedCurve.get(); auto* face=faces[0];
    const auto interval=curve->GetNaturalInterval();
    SmSolutionArray solutions;
    OwnedCurves overlaps(context);
    SmTArray<SmPoint3d> points(context);
    const auto kernelStart=Clock::now();
    check(face->GetSurface()->GlobalCurveIntersect(face->GetUVDomain(),*curve,interval,request.tolerance,solutions), "SMLib GlobalCurveIntersect");
    for (ULONG i=0; i<solutions.GetSize(); ++i) {
        const auto& solution=solutions[i];
        if (solution.m_eSolutionType==SM_ST_RANGE_OF_VALUES) {
            // Keep the solver's exact parameter range, then let SMLib trim it
            // against the actual face loops (including holes).
            SmCurve* raw=nullptr; check(curve->Copy(context,raw),"Copy overlap curve");
            std::unique_ptr<SmCurve> part(raw);
            SmExtent1d range(std::max(interval.GetMin(),std::min(solution.m_vStart[0],solution.m_vEnd[0])),
                std::min(interval.GetMax(),std::max(solution.m_vStart[0],solution.m_vEnd[0])));
            if (range.GetMax()<=range.GetMin()) continue;
            check(part->Trim(range),"Trim overlap interval");
            SmTArray<SmCurve*> input(context); input.Add(part.get());
            const SmApproxTol3d tolerance(request.tolerance);
            OwnedCurves clipped(context);
            check(face->CreateCurvesByTrimming(context,&tolerance,&input,nullptr,&clipped.values,nullptr),"Trim overlap to face");
            for (ULONG j=0; j<clipped.values.GetSize(); ++j) {
                overlaps.values.Add(clipped.values[j]); clipped.values[j]=nullptr;
            }
        } else if (solution.m_eSolutionType==SM_ST_SINGLE_VALUE) {
            const auto t=solution.m_vStart[0];
            if (t<interval.GetMin() || t>interval.GetMax()) continue;
            SmPoint3d point; check(curve->EvaluatePoint(t,point),"Evaluate intersection point");
            SmPointClassification classification;
            check(face->Point3DClassify(point,SmZoneTol3d(request.tolerance),TRUE,classification),"Classify point against trimmed face");
            const auto type=classification.GetPointClass();
            if (type!=SM_PC_FACE && type!=SM_PC_EDGE && type!=SM_PC_VERTEX) continue;
            bool duplicate=false;
            for (ULONG j=0; j<points.GetSize(); ++j) if (point.DistanceBetween(points[j])<=request.tolerance) duplicate=true;
            if (!duplicate) points.Add(point);
        } else throw std::runtime_error("Unsupported SMLib curve/surface solution type.");
    }
    const auto kernelEnd=Clock::now();
    IntersectionResult result{IntersectionStatus::Success,{},"SMLib GlobalCurveIntersect + trimmed-face classification."};
    SmBrep wires;
    for (ULONG i=0; i<overlaps.values.GetSize(); ++i) {
        auto* part=overlaps.values[i]; const auto range=part->GetNaturalInterval();
        SmPoint3d a,b; check(part->EvaluatePoint(range.GetMin(),a),"Overlap start"); check(part->EvaluatePoint(range.GetMax(),b),"Overlap end");
        SmEdge* output=nullptr;
        if (a.DistanceBetween(b)<=request.tolerance) {
            SmShell* shell=nullptr; SmVertex* vertex=nullptr;
            check(wires.MakeShellVertex(wires.GetInfiniteRegion(),a,shell,vertex),"Closed overlap vertex");
            overlaps.values[i]=nullptr;
            check(wires.MakeWireEdge(wires.GetInfiniteRegion(),vertex,vertex,part,range,SM_OT_SAME,output),"Closed overlap edge");
        } else {
            overlaps.values[i]=nullptr;
            check(wires.CreateWireEdgeFromCurve(part,range,output),"Overlap wire");
        }
    }
    if (overlaps.values.GetSize()>0) {
        TopoDS_Shape output; std::string error;
        if (!OccSmBridge::ConvertToOcc(wires,output,&error)) throw std::runtime_error("Overlap -> OCC: " + error);
        result.curves.push_back(output);
    }
    for (ULONG i=0; i<points.GetSize(); ++i) {
        const auto vertex=BRepBuilderAPI_MakeVertex(gp_Pnt(points[i].x,points[i].y,points[i].z)).Vertex();
        if (overlaps.values.GetSize()>0) {
            BRepExtrema_DistShapeShape distance(vertex,result.curves.front());
            if (!distance.IsDone()) throw std::runtime_error("Overlap point filtering failed.");
            if (distance.Value()<=request.tolerance) continue;
        }
        result.curves.push_back(vertex);
    }
    result.kernelMilliseconds=std::chrono::duration<double,std::milli>(kernelEnd-kernelStart).count();
    result.conversionMilliseconds=std::chrono::duration<double,std::milli>((kernelStart-start)+(Clock::now()-kernelEnd)).count();
    for (const auto& warning:faceModel.report().warnings) result.message+="\nFace bridge: "+warning;
    return result;
}
}
