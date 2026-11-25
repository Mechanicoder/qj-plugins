#pragma once
// kernels/occ/mesh_inter.h — 三角形网格之间的快速求交，以及基于网格的面/面交线近似。

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <TopLoc_Location.hxx>
#include <gp_XYZ.hxx>

class Poly_Triangulation;
class TopoDS_Face;

namespace mesh_inter {

// ---------------------------------------------------------------------------
// 输入网格
// ---------------------------------------------------------------------------

// 三角形索引（指向 TriMesh::nodes）。
struct TriMeshTri
{
    std::uint32_t v0 = 0;
    std::uint32_t v1 = 0;
    std::uint32_t v2 = 0;
};

// 世界坐标下的三角形汤（triangle soup）。
// 节点特意使用 double：窄相位不应继承显示用顶点缓冲的 float 精度。
struct TriMesh
{
    std::vector<gp_XYZ>     nodes;   // 顶点坐标
    std::vector<TriMeshTri> tris;    // 三角形索引

    std::size_t NodeCount() const;
    std::size_t TriangleCount() const;
    bool IsEmpty() const;
    void Clear();
};

// 把 OCC 的 Poly_Triangulation（可带位置变换）展平成 TriMesh。
// 引用了非法节点编号的三角形会被跳过。
TriMesh TriMeshFromTriangulation(const Poly_Triangulation& triangulation,
                                 const TopLoc_Location&    location = TopLoc_Location());

// ---------------------------------------------------------------------------
// 求交参数
// ---------------------------------------------------------------------------

struct MeshIntersectOptions
{
    // 几何距离容差（模型单位，通常为 mm）：点到平面/区间的判定阈值。
    double distanceTol   = 1e-7;
    // 两平面法向叉积长度低于该值视为“平行”；平行时再判断是否共面。
    double angularTol    = 1e-12;
    // 退化三角形判据：|edge0 x edge1| <= degenerateTol * maxEdge^2。
    double degenerateTol = 1e-12;
    // 端点吸附半径：用于线段去重和折线缝合（把同一交点的重复结果合并）。
    double mergeTol      = 1e-6;

    bool includeCoplanar = true;   // 是否输出共面重叠区域的边界
    bool buildPolylines  = true;   // 是否把线段缝合成折线

    // --- 后处理 / 鲁棒性开关 ---------------------------------------------
    // 丢弃长度小于该值的折线（0 表示全部保留）。近相切处在剖分近似下容易出现
    // 短小碎片，调用方可用该阈值过滤。
    double minPolylineLength = 0.0;
    // 平面距在该范围内即按共面处理（0 表示沿用 distanceTol）。用于两侧面近乎
    // 重合、但三角剖分不同的情况。
    double coplanarBand      = 0.0;
};

// ---------------------------------------------------------------------------
// 网格级结果
// ---------------------------------------------------------------------------

// 一条去重后的交线线段（端点已焊接）。
struct MeshIntersectSegment
{
    gp_XYZ        p0;
    gp_XYZ        p1;
    std::uint32_t triA     = 0;     // 来源三角形在网格 A 中的下标
    std::uint32_t triB     = 0;     // 来源三角形在网格 B 中的下标
    bool          coplanar = false; // 是否由共面重叠多边形产生
};

// 缝合后的交线折线。closed=true 表示首尾闭合（形成环）。
struct MeshPolyline
{
    std::vector<gp_XYZ> points;
    bool                closed = false;
};

// 统计信息，用于性能分析/回归对比。
struct MeshIntersectStats
{
    std::size_t bvhPairs       = 0;  // 双树遍历访问的节点对数
    std::size_t candidatePairs = 0;  // 叶子层枚举出的三角形对总数
    std::size_t boxRejected    = 0;  // 被三角形 AABB 测试剔除的候选对
    std::size_t triPairs       = 0;  // 真正进入窄相位的三角形对数
    std::size_t degenerateTris = 0;  // A、B 两侧累计跳过的退化三角形数
    std::size_t coplanarPairs  = 0;  // 共面重叠的三角形对数
    std::size_t pointHits      = 0;  // 点接触次数（含吸附后退化的线段）
    std::size_t rawSegments    = 0;  // 去重前的原始线段数
    std::size_t duplicates     = 0;  // 被端点网格过滤掉的重复线段数
};

struct MeshIntersectResult
{
    std::vector<MeshIntersectSegment> segments;     // 去重后的线段
    std::vector<MeshPolyline>         polylines;    // 缝合后的折线
    MeshIntersectStats                stats;        // 统计
    bool                              bboxOverlap = false;  // 两网格包围盒是否重叠
};

// 完整网格/网格求交：宽相位 + 窄相位 + 缝合。
MeshIntersectResult IntersectMeshes(const TriMesh& mesh_a, const TriMesh& mesh_b,
                                    const MeshIntersectOptions& options = MeshIntersectOptions());

// 直接对两个 OCC 三角剖分求交的便捷重载（例如 IKernel::Tessellate 或
// BRep_Tool::Triangulation 的输出）。
MeshIntersectResult IntersectTriangulations(const Poly_Triangulation& tri_a,
                                            const TopLoc_Location&    loc_a,
                                            const Poly_Triangulation& tri_b,
                                            const TopLoc_Location&    loc_b,
                                            const MeshIntersectOptions& options = MeshIntersectOptions());

// ---------------------------------------------------------------------------
// 面级封装
// ---------------------------------------------------------------------------

struct FaceIntersectResult
{
    bool                      ok = false;   // 是否执行成功（不含“无交线”这种情况）
    std::string               error;        // 失败原因
    std::vector<MeshPolyline> curves;       // 缝合后的交线折线
    MeshIntersectResult       mesh;         // 底层网格求交的完整结果与统计
};

// 按给定弦高/转角误差剖分两张面，再对两套三角形求交，得到交线的网格近似。
// 与精确交线的偏差由剖分弦高（linear_deflection）决定，而不是 distanceTol。
//
// linear_deflection  : 剖分允许的最大弦高误差（模型单位）
// angular_deflection : 剖分允许的最大转角误差（弧度）
FaceIntersectResult IntersectFaces(const TopoDS_Face& face_a, const TopoDS_Face& face_b,
                                   double linear_deflection  = 0.05,
                                   double angular_deflection = 0.35,
                                   const MeshIntersectOptions& options = MeshIntersectOptions());

} // namespace mesh_inter
