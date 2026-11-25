#include "xyz_points.h"

#include <QFile>
#include <QStringList>
#include <QTextStream>
#include <Precision.hxx>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace xyz {
namespace {
[[noreturn]] void fail(const QString& message)
{
    throw std::runtime_error(message.toUtf8().constData());
}
}

std::vector<gp_Pnt> readPoints(const QString& filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        fail(QStringLiteral("无法打开文件：%1").arg(file.errorString()));

    QTextStream input(&file);
    input.setCodec("UTF-8");
    std::vector<gp_Pnt> points;
    QString line;
    qsizetype lineNumber = 0;
    while (input.readLineInto(&line)) {
        ++lineNumber;
        line = line.trimmed();
        if (line.isEmpty()) continue;
        // Also accept the explicit line-ending backslash shown in the example.
        if (line.endsWith(QLatin1Char('\\'))) line.chop(1);
        const auto fields = line.split(QLatin1Char(','));
        if (fields.size() != 3)
            fail(QStringLiteral("第 %1 行需要三个逗号分隔的坐标。").arg(lineNumber));

        double coordinates[3];
        for (int axis = 0; axis < 3; ++axis) {
            bool valid = false;
            coordinates[axis] = fields[axis].trimmed().toDouble(&valid);
            if (!valid || !std::isfinite(coordinates[axis]))
                fail(QStringLiteral("第 %1 行的第 %2 列不是有限数值。").arg(lineNumber).arg(axis + 1));
        }
        points.emplace_back(coordinates[0], coordinates[1], coordinates[2]);
    }
    if (file.error() != QFileDevice::NoError)
        fail(QStringLiteral("读取 XYZ 文件失败：%1").arg(file.errorString()));
    if (points.empty()) fail(QStringLiteral("文件中没有坐标。"));
    return points;
}

std::vector<gp_Pnt> polygonVertices(const std::vector<gp_Pnt>& points)
{
    const auto coincides = [](const gp_Pnt& a, const gp_Pnt& b) {
        return a.IsEqual(b, Precision::Confusion());
    };
    std::vector<gp_Pnt> vertices;
    vertices.reserve(points.size());
    for (const auto& point : points) {
        if (vertices.empty() || !coincides(vertices.back(), point)) vertices.push_back(point);
    }
    if (vertices.size() > 1 && coincides(vertices.front(), vertices.back())) vertices.pop_back();
    if (vertices.size() < 3 || std::none_of(vertices.begin() + 2, vertices.end(), [&](const gp_Pnt& vertex) {
        return !coincides(vertex, vertices[0]) && !coincides(vertex, vertices[1]);
    })) {
        fail(QStringLiteral("多边形至少需要三个不同的顶点。"));
    }
    return vertices;
}
}
