#include "face_realtime.h"

#include <BRepBuilderAPI_MakeFace.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <iostream>
#include <string>

namespace {
int failures = 0;

void check(bool condition, const char* message)
{
    std::cout << (condition ? "PASS " : "FAIL ") << message << '\n';
    if (!condition) ++failures;
}
} // namespace

int main()
{
    std::cout << std::unitbuf;
    const TopoDS_Face xy = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1)), -5, 5, -5, 5);
    const TopoDS_Face xz = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0)), -5, 5, -5, 5);
    const TopoDS_Face above = BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(0, 0, 2), gp_Dir(0, 0, 1)), -5, 5, -5, 5);

    std::string error;
    sfi::FaceGroup single;
    check(sfi::prepare({xy}, {xz}, 1e-6, 20.0, single, error), "prepare single pair");
    const auto crossing = sfi::intersect(single, false, 0.0, 0.0, 0.0);
    check(crossing.edges == 1, "perpendicular faces intersect in one edge");
    check(crossing.succeededPairs == 1 && crossing.failedPairs == 0, "pair succeeded");

    const auto away = sfi::intersect(single, true, 20.0, 0.0, 0.0);
    check(away.edges == 0 && away.vertices == 0, "moved face leaves intersection empty");
    sfi::clear(single);

    sfi::FaceGroup multiple;
    check(sfi::prepare({xy, above}, {xz}, 1e-6, 20.0, multiple, error), "prepare multi face groups");
    check(!multiple.firstCompound.IsNull() && !multiple.secondCompound.IsNull(), "group compounds built");
    const auto both = sfi::intersect(multiple, false, 0.0, 0.0, 0.0);
    check(both.edges == 2, "two parallel faces yield two intersection edges");

    const auto duplicate = sfi::intersect(multiple, true, 0.0, 0.0, 0.0);
    check(duplicate.edges == 2, "moving group at zero offset keeps result");
    sfi::clear(multiple);
    check(!multiple.ready, "clear resets group");

    sfi::FaceGroup same;
    check(sfi::prepare({xy}, {xy}, 1e-6, 20.0, same, error), "prepare identical face groups");
    const auto self = sfi::intersect(same, false, 0.0, 0.0, 0.0);
    check(self.skippedPairs == 1 && self.edges == 0, "identical face pair is skipped");

    if (failures != 0) {
        std::cout << "FAILED " << failures << " check(s)\n";
        return 1;
    }
    std::cout << "PASSED all face realtime checks\n";
    return 0;
}
