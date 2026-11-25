#pragma once

#include <QString>
#include <gp_Pnt.hxx>
#include <vector>

namespace xyz {
std::vector<gp_Pnt> readPoints(const QString& filename);
std::vector<gp_Pnt> polygonVertices(const std::vector<gp_Pnt>& points);
}
