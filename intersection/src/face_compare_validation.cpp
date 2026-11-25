#include "face_compare_validation.h"
#include <BRepAlgoAPI_Section.hxx>
#include <BRep_Builder.hxx>
#include <BRepTools.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <BRepBuilderAPI_Copy.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <Standard_Failure.hxx>
#include <QDir>
#include <QFile>
#include <chrono>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <algorithm>

namespace ffi {
namespace {
bool validRequest(const IntersectionRequest& r)
{
    return !r.first.IsNull() && !r.second.IsNull() && !r.first.IsSame(r.second)
        && std::isfinite(r.tolerance) && r.tolerance > 0;
}

TopoDS_Face readFace(const QString& filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly))
        throw std::runtime_error(QStringLiteral("无法读取：%1 (%2)").arg(filename, file.errorString()).toUtf8().constData());
    const auto bytes = file.readAll();
    if (file.error() != QFile::NoError)
        throw std::runtime_error("BREP file read failed.");
    std::istringstream stream(std::string(bytes.constData(), static_cast<size_t>(bytes.size())));
    TopoDS_Shape shape;
    BRep_Builder builder;
    BRepTools::Read(shape, stream, builder);
    if (shape.IsNull()) throw std::runtime_error("BREP file contains no readable shape.");
    TopTools_IndexedMapOfShape faces;
    TopExp::MapShapes(shape, TopAbs_FACE, faces);
    if (faces.Extent() != 1)
        throw std::runtime_error(QStringLiteral("%1 必须恰好包含一个面，实际 %2 个。")
            .arg(filename).arg(faces.Extent()).toUtf8().constData());
    const auto face = TopoDS::Face(faces(1));
    if (!BRepCheck_Analyzer(face).IsValid()) throw std::runtime_error("Input face has invalid topology.");
    return face;
}

IntersectionResult runOcc(const IntersectionRequest& request)
{
    if (!validRequest(request)) return {IntersectionStatus::InvalidInput, {}, "Invalid input."};
    BRepAlgoAPI_Section section(request.first, request.second, false);
    section.SetRunParallel(false); // Stable single-thread baseline.
    section.SetNonDestructive(true);
    section.SetFuzzyValue(request.tolerance);
    section.Approximation(true);
    section.Build();
    if (!section.IsDone() || section.HasErrors()) return {IntersectionStatus::Failed, {}, "OCC section failed."};
    return {IntersectionStatus::Success, {section.Shape()}, "OCC section completed."};
}

template<class F> AlgorithmRun timed(F&& function)
{
    AlgorithmRun run;
    const auto start = std::chrono::steady_clock::now();
    try { run.result = function(); }
    catch (const Standard_Failure& error) {
        run.result = {IntersectionStatus::Failed, {}, error.GetMessageString() ? error.GetMessageString() : "OCC exception"};
    } catch (const std::exception& error) {
        run.result = {IntersectionStatus::Failed, {}, error.what()};
    } catch (...) { run.result = {IntersectionStatus::Failed, {}, "Unknown exception"}; }
    run.milliseconds = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    return run;
}

ShapeMetrics metrics(const TopoDS_Shape& shape)
{
    ShapeMetrics result;
    TopTools_IndexedMapOfShape edges, vertices;
    TopExp::MapShapes(shape, TopAbs_EDGE, edges);
    TopExp::MapShapes(shape, TopAbs_VERTEX, vertices);
    result.edges = edges.Extent();
    result.vertices = vertices.Extent();
    GProp_GProps props;
    BRepGProp::LinearProperties(shape, props);
    result.length = props.Mass();
    result.valid = BRepCheck_Analyzer(shape).IsValid();
    return result;
}

// Samples each edge at 33 parameter values, plus all vertices. This is not a
// certified Hausdorff bound and can miss a discrepancy between samples.
double directedDeviation(const TopoDS_Shape& source, const TopoDS_Shape& target)
{
    double maximum = 0;
    auto sample = [&](const gp_Pnt& point) {
        BRepExtrema_DistShapeShape distance(BRepBuilderAPI_MakeVertex(point).Vertex(), target);
        if (!distance.IsDone() || distance.NbSolution() == 0) throw std::runtime_error("Distance calculation failed.");
        maximum = std::max(maximum, distance.Value());
    };
    TopTools_IndexedMapOfShape vertices, edges;
    TopExp::MapShapes(source, TopAbs_VERTEX, vertices);
    TopExp::MapShapes(source, TopAbs_EDGE, edges);
    for (int i = 1; i <= vertices.Extent(); ++i) sample(BRep_Tool::Pnt(TopoDS::Vertex(vertices(i))));
    for (int i = 1; i <= edges.Extent(); ++i) {
        const auto edge = TopoDS::Edge(edges(i));
        if (BRep_Tool::Degenerated(edge)) continue;
        BRepAdaptor_Curve curve(edge);
        const double first = curve.FirstParameter(), last = curve.LastParameter();
        if (!std::isfinite(first) || !std::isfinite(last) || std::abs(first) > 1e50 || std::abs(last) > 1e50)
            throw std::runtime_error("Cannot sample an unbounded edge.");
        for (int j = 0; j <= 32; ++j) sample(curve.Value(first + (last - first) * j / 32.0));
    }
    return maximum;
}

QString describe(const QString& name, const AlgorithmRun& run, const ShapeMetrics& m)
{
    if (run.result.status != IntersectionStatus::Success) {
        const auto state = run.result.status == IntersectionStatus::NotImplemented ? QStringLiteral("未实现（不参与比较）") : QStringLiteral("失败");
        return QStringLiteral("%1：%2；%3").arg(name, state, QString::fromUtf8(run.result.message.c_str()));
    }
    return QStringLiteral("%1：%2 ms；边 %3，顶点 %4，长度 %5；拓扑%6")
        .arg(name).arg(run.milliseconds, 0, 'f', 3).arg(m.edges).arg(m.vertices)
        .arg(m.length, 0, 'g', 12).arg(m.valid ? QStringLiteral("有效") : QStringLiteral("无效"));
}
} // namespace

IntersectionRequest readCase(const QString& directory, double tolerance)
{
    const QDir dir(directory);
    QStringList files;
    for (const auto& file : dir.entryInfoList(QDir::Files, QDir::Name))
        if (file.suffix().compare(QStringLiteral("brep"), Qt::CaseInsensitive) == 0) files << file.absoluteFilePath();
    if (files.size() != 2) throw std::runtime_error(QStringLiteral("测例目录必须恰有两个 .brep 文件，实际 %1 个。")
        .arg(files.size()).toUtf8().constData());
    return {readFace(files[0]), readFace(files[1]), tolerance};
}

TopoDS_Shape combineResults(const IntersectionResult& result)
{
    TopoDS_Compound compound;
    BRep_Builder builder;
    builder.MakeCompound(compound);
    for (const auto& shape : result.curves) if (!shape.IsNull()) builder.Add(compound, shape);
    return compound;
}

ValidationReport validate(const IntersectionRequest& request, const IntersectionAlgorithm& customAlgorithm)
{
    ValidationReport report;
    if (!validRequest(request)) throw std::runtime_error("Invalid face pair or tolerance.");
    // Separate geometry copies prevent either algorithm from modifying the
    // other's inputs. Preparation is excluded from both algorithm timers.
    auto copyRequest = [&] {
        return IntersectionRequest{TopoDS::Face(BRepBuilderAPI_Copy(request.first).Shape()),
            TopoDS::Face(BRepBuilderAPI_Copy(request.second).Shape()), request.tolerance};
    };
    const auto occInput = copyRequest();
    const auto customInput = copyRequest();
    report.occ = timed([&] { return runOcc(occInput); });
    report.custom = timed([&] { return customAlgorithm(customInput); });
    const auto occShape = combineResults(report.occ.result);
    const auto customShape = combineResults(report.custom.result);
    if (report.occ.result.status == IntersectionStatus::Success) report.occMetrics = metrics(occShape);
    if (report.custom.result.status == IntersectionStatus::Success) report.customMetrics = metrics(customShape);
    report.text = describe(QStringLiteral("OCC"), report.occ, report.occMetrics) + '\n'
        + describe(QStringLiteral("SMLib（含转换总耗时）"), report.custom, report.customMetrics);
    if (report.custom.result.kernelMilliseconds >= 0) {
        report.text += QStringLiteral("\nSMLib 内核 %1 ms；转换及结果构建 %2 ms；角度容差 20°")
            .arg(report.custom.result.kernelMilliseconds, 0, 'f', 3)
            .arg(report.custom.result.conversionMilliseconds, 0, 'f', 3);
        report.text += '\n' + QString::fromUtf8(report.custom.result.message.c_str());
    }
    if (report.occ.result.status == IntersectionStatus::Success && report.custom.result.status == IntersectionStatus::Success) {
        const bool occEmpty = report.occMetrics.vertices == 0 && report.occMetrics.edges == 0;
        const bool customEmpty = report.customMetrics.vertices == 0 && report.customMetrics.edges == 0;
        QString quality;
        if (occEmpty && customEmpty) quality = QStringLiteral("双方均无交线/交点");
        else if (occEmpty != customEmpty) quality = QStringLiteral("结果不一致：一方为空");
        else {
            try {
                report.sampledDeviation = std::max(directedDeviation(occShape, customShape), directedDeviation(customShape, occShape));
                quality = QStringLiteral("双向采样最大偏差 %1（%2容差）")
                    .arg(report.sampledDeviation, 0, 'g', 12)
                    .arg(report.sampledDeviation <= request.tolerance ? QStringLiteral("在") : QStringLiteral("超出"));
            } catch (const Standard_Failure&) { quality = QStringLiteral("采样偏差计算失败"); }
              catch (const std::exception& e) { quality = QStringLiteral("采样偏差不可用：%1").arg(QString::fromUtf8(e.what())); }
        }
        report.text += QStringLiteral("\n对比：%1；长度差 %2；边数差 %3；拓扑有效 %4/%5")
            .arg(quality).arg(std::abs(report.occMetrics.length - report.customMetrics.length), 0, 'g', 12)
            .arg(report.customMetrics.edges - report.occMetrics.edges)
            .arg(report.occMetrics.valid).arg(report.customMetrics.valid);
        if (report.custom.milliseconds > 0)
            report.text += QStringLiteral("\n耗时比 OCC/SMLib（含转换） = %1").arg(report.occ.milliseconds / report.custom.milliseconds, 0, 'f', 3);
    } else report.text += QStringLiteral("\n质量/效率对比：不可用（需双方算法均成功）。");
    report.text += QStringLiteral("\n耗时仅计单次算法调用，排除读文件、质量检查和模型树输出；采样每边 33 点及顶点，非严格误差上界。长度单位同模型。");
    return report;
}
} // namespace ffi
