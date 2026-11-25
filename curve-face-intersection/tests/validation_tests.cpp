#include "validation.h"
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepTools.hxx>
#include <gp_Pln.hxx>
#include <gp_Sphere.hxx>
#include <gp_Circ.hxx>
#include <gp_Trsf.hxx>
#include <TopoDS.hxx>
#include <QTemporaryDir>
#include <QDir>
#include <QFile>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <cmath>

void require(bool condition,const char* message) { if(!condition) throw std::runtime_error(message); }
void writeBrep(const QString& file,const TopoDS_Shape& shape) {
    std::ostringstream stream; BRepTools::Write(shape,stream);
    QFile output(file); require(output.open(QIODevice::WriteOnly),"open BREP output");
    const auto text=stream.str(); require(output.write(text.data(),static_cast<qint64>(text.size()))==static_cast<qint64>(text.size()),"write BREP output");
}
cfi::ValidationReport run(const char* name,const TopoDS_Edge& edge,const TopoDS_Face& face,int points,double length=0) {
    std::cout << "START " << name << '\n';
    const auto report=cfi::validate({edge,face,1e-6});
    std::cout << report.text.toUtf8().constData() << '\n';
    require(report.occ.result.status==cfi::IntersectionStatus::Success,"OCC failed");
    require(report.custom.result.status==cfi::IntersectionStatus::Success,"SMLib failed");
    require(report.occMetrics.valid && report.customMetrics.valid,"invalid result topology");
    require(report.occMetrics.vertices==points && report.customMetrics.vertices==points,"wrong point/vertex count");
    require(std::abs(report.occMetrics.length-length)<1e-5 && std::abs(report.customMetrics.length-length)<1e-5,"wrong overlap length");
    if(points>0) require(report.sampledDeviation>=0 && report.sampledDeviation<=1e-6,"geometric discrepancy");
    return report;
}
int main() {
    std::cout << std::unitbuf;
    try {
        const TopoDS_Face plane=BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0,0,0),gp_Dir(0,0,1)),-5,5,-5,5);
        auto line=[](gp_Pnt a,gp_Pnt b)->TopoDS_Edge { return BRepBuilderAPI_MakeEdge(a,b); };
        const auto crossing=line(gp_Pnt(2,0,-2),gp_Pnt(2,0,2));
        run("crossing",crossing,plane,1);
        run("outside trims",line(gp_Pnt(9,0,-2),gp_Pnt(9,0,2)),plane,0);
        run("finite edge misses",line(gp_Pnt(2,0,1),gp_Pnt(2,0,2)),plane,0);
        run("endpoint",line(gp_Pnt(2,0,0),gp_Pnt(2,0,2)),plane,1);
        run("boundary",line(gp_Pnt(5,0,-2),gp_Pnt(5,0,2)),plane,1);
        BRepBuilderAPI_MakePolygon hole;
        hole.Add(gp_Pnt(-1,-1,0));hole.Add(gp_Pnt(-1,1,0));hole.Add(gp_Pnt(1,1,0));hole.Add(gp_Pnt(1,-1,0));hole.Close();
        BRepBuilderAPI_MakeFace perforated(plane);perforated.Add(hole.Wire());
        run("hole excludes point",line(gp_Pnt(0,0,-2),gp_Pnt(0,0,2)),perforated.Face(),0);
        run("coplanar overlap",line(gp_Pnt(-8,0,0),gp_Pnt(8,0,0)),plane,2,10);
        run("overlap split by hole",line(gp_Pnt(-8,0,0),gp_Pnt(8,0,0)),perforated.Face(),4,8);
        constexpr double pi=3.14159265358979323846;
        const TopoDS_Face sphere=BRepBuilderAPI_MakeFace(gp_Sphere(gp_Ax3(),2),0,2*pi,-pi/2,pi/2);
        run("sphere two points",line(gp_Pnt(-4,0,0),gp_Pnt(4,0,0)),sphere,2);
        run("sphere tangent",line(gp_Pnt(-4,0,2),gp_Pnt(4,0,2)),sphere,1);
        const TopoDS_Edge circle=BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(gp_Pnt(0,0,0),gp_Dir(0,1,0)),2));
        run("curved edge",circle,plane,2);
        const TopoDS_Edge coplanarCircle=BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(gp_Pnt(0,0,0),gp_Dir(0,0,1)),2));
        run("closed curved overlap",coplanarCircle,plane,1,4*pi);
        gp_Trsf translation; translation.SetTranslation(gp_Vec(2,0,0));
        const auto local=line(gp_Pnt(0,0,-2),gp_Pnt(0,0,2));
        run("edge location",TopoDS::Edge(local.Moved(TopLoc_Location(translation))),plane,1);
        QTemporaryDir temp; require(temp.isValid(),"temporary directory");
        const auto directory=temp.path()+QStringLiteral("/中文测例");require(QDir().mkpath(directory),"unicode path");
        writeBrep(directory+"/a-face.BREP",plane);writeBrep(directory+"/z-edge.brep",crossing);
        const auto loaded=cfi::validate(cfi::readCase(directory,1e-6));
        require(loaded.customMetrics.vertices==1,"automatic edge/face file detection");
        writeBrep(directory+"/z-edge.brep",plane);
        bool rejected=false;try{cfi::readCase(directory,1e-6);}catch(...){rejected=true;}
        require(rejected,"two faces must be rejected");
        std::cout << "PASS: finite/curved edge, trimmed faces/holes, overlap, tangent, unicode loading and input checks\n";
        return 0;
    } catch(const std::exception& e) { std::cerr << "FAIL: " << e.what() << '\n';return 1; }
}
