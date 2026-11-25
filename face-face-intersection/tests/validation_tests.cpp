#include "validation.h"
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <gp_Pln.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Sphere.hxx>
#include <QTemporaryDir>
#include <QDir>
#include <QFile>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <cmath>

void require(bool condition, const char* message) { if (!condition) throw std::runtime_error(message); }
void writeBrep(const QString& filename, const TopoDS_Shape& shape)
{
    std::ostringstream stream;
    BRepTools::Write(shape, stream);
    QFile file(filename);
    require(file.open(QIODevice::WriteOnly), "open test BREP");
    const auto data = stream.str();
    require(file.write(data.data(), static_cast<qint64>(data.size())) == static_cast<qint64>(data.size()), "write test BREP");
}
template<class F> void expectFailure(F f, const char* message)
{
    bool failed = false;
    try { f(); } catch (...) { failed = true; }
    require(failed, message);
}
int main()
{
    std::cout << std::unitbuf;
    try {
        const TopoDS_Face xy = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0,0,0), gp_Dir(0,0,1)), -5,5,-5,5);
        const TopoDS_Face xz = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0,0,0), gp_Dir(0,1,0)), -5,5,-5,5);
        const TopoDS_Face above = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0,0,2), gp_Dir(0,0,1)), -5,5,-5,5);
        ffi::IntersectionRequest request{xy,xz,1e-6};
        const auto report = ffi::validate(request);
        require(report.occ.result.status == ffi::IntersectionStatus::Success, "crossing section failed");
        require(report.occMetrics.edges == 1 && std::abs(report.occMetrics.length-10) < 1e-6 && report.occMetrics.valid, "crossing section quality");
        std::cout << report.text.toUtf8().constData() << '\n';
        require(report.custom.result.status == ffi::IntersectionStatus::Success, "SMLib crossing section failed");
        require(report.customMetrics.valid && std::abs(report.customMetrics.length-10) < 1e-6
            && report.sampledDeviation >= 0 && report.sampledDeviation <= request.tolerance, "SMLib crossing section quality");
        const auto identical = ffi::validate(request, [&](const auto&) { return report.occ.result; });
        require(identical.sampledDeviation >= 0 && identical.sampledDeviation < 1e-7, "identical section distance");
        const auto mismatch = ffi::validate(request, [](const auto&) {
            return ffi::IntersectionResult{ffi::IntersectionStatus::Success, {BRepBuilderAPI_MakeVertex(gp_Pnt(0,0,3)).Shape()}, "test"};
        });
        require(mismatch.sampledDeviation > 2.9, "incorrect geometry must differ");
        const auto empty = ffi::validate({xy,above,1e-6});
        require(empty.occ.result.status == ffi::IntersectionStatus::Success && empty.occMetrics.edges == 0 && empty.occMetrics.vertices == 0, "disjoint faces must yield empty success");
        require(empty.custom.result.status == ffi::IntersectionStatus::Success && empty.customMetrics.edges == 0 && empty.customMetrics.vertices == 0, "SMLib disjoint faces");
        const TopoDS_Face outside = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0,20,0), gp_Dir(0,1,0)), -5,5,-5,5);
        const auto trimmedOut = ffi::validate({xy,outside,1e-6});
        require(trimmedOut.custom.result.status == ffi::IntersectionStatus::Success
            && trimmedOut.customMetrics.edges == 0 && trimmedOut.customMetrics.vertices == 0, "must intersect trimmed faces, not infinite support surfaces");
        BRepBuilderAPI_MakePolygon hole;
        hole.Add(gp_Pnt(-1,-1,0)); hole.Add(gp_Pnt(-1,1,0));
        hole.Add(gp_Pnt(1,1,0)); hole.Add(gp_Pnt(1,-1,0)); hole.Close();
        BRepBuilderAPI_MakeFace perforated(xy); perforated.Add(hole.Wire());
        std::cout << "START HOLE\n";
        const auto withHole = ffi::validate({perforated.Face(),xz,1e-6});
        std::cout << "HOLE\n" << withHole.text.toUtf8().constData() << '\n';
        require(withHole.custom.result.status == ffi::IntersectionStatus::Success && withHole.customMetrics.valid
            && std::abs(withHole.customMetrics.length-8) < 1e-6 && withHole.sampledDeviation <= 1e-6
            && withHole.sampledDeviation >= 0, "hole trimming must split intersection");
        constexpr double pi = 3.14159265358979323846;
        const TopoDS_Face cylinder = BRepBuilderAPI_MakeFace(gp_Cylinder(gp_Ax3(),2),0,2*pi,-3,3);
        std::cout << "START CYLINDER\n";
        const auto circle = ffi::validate({cylinder,xy,1e-6});
        std::cout << "CYLINDER\n" << circle.text.toUtf8().constData() << '\n';
        require(circle.custom.result.status == ffi::IntersectionStatus::Success && circle.customMetrics.valid
            && std::abs(circle.customMetrics.length-4*pi) < 1e-5 && circle.sampledDeviation >= 0
            && circle.sampledDeviation <= 1e-6, "curved surface intersection quality");
        const TopoDS_Face sphere = BRepBuilderAPI_MakeFace(gp_Sphere(gp_Ax3(),2),0,2*pi,-pi/2,pi/2);
        std::cout << "START TANGENT\n";
        const auto tangent = ffi::validate({sphere,above,1e-6});
        std::cout << "TANGENT\n" << tangent.text.toUtf8().constData() << '\n';
        require(tangent.custom.result.status == ffi::IntersectionStatus::Success && tangent.customMetrics.edges == 0
            && tangent.customMetrics.vertices > 0 && tangent.sampledDeviation >= 0 && tangent.sampledDeviation <= 1e-6,
            "isolated tangent point must survive round trip");
        const auto exception = ffi::validate(request, [](const auto&) -> ffi::IntersectionResult { throw std::runtime_error("test failure"); });
        require(exception.occ.result.status == ffi::IntersectionStatus::Success && exception.custom.result.status == ffi::IntersectionStatus::Failed, "custom exception isolation");
        QTemporaryDir temporary;
        require(temporary.isValid(), "temporary directory");
        const auto directory = temporary.path() + QStringLiteral("/中文测例");
        require(QDir().mkpath(directory), "unicode case directory");
        writeBrep(directory + "/a.brep", xy);
        writeBrep(directory + "/b.BREP", xz);
        const auto loaded = ffi::validate(ffi::readCase(directory,1e-6));
        require(loaded.occMetrics.edges == 1, "unicode BREP loading");
        writeBrep(directory + "/c.brep", xy);
        expectFailure([&]{ ffi::readCase(directory,1e-6); }, "three files must fail");
        QFile::remove(directory + "/c.brep");
        { QFile bad(directory + "/a.brep"); require(bad.open(QIODevice::WriteOnly), "bad file"); bad.write("not a BREP"); }
        expectFailure([&]{ ffi::readCase(directory,1e-6); }, "corrupt BREP must fail");
        TopoDS_Compound multi;
        BRep_Builder builder; builder.MakeCompound(multi); builder.Add(multi,xy); builder.Add(multi,xz);
        writeBrep(directory + "/a.brep", multi);
        expectFailure([&]{ ffi::readCase(directory,1e-6); }, "multiple faces must fail");
        QFile::remove(directory + "/a.brep");
        expectFailure([&]{ ffi::readCase(directory,1e-6); }, "missing pair must fail");
        std::cout << "PASS: OCC crossing/disjoint, comparison, exception isolation, unicode loading, invalid case rejection\n";
        return 0;
    } catch (const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n'; return 1; }
}
