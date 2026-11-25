// kernels/occ/mesh_inter.cpp — 三角形网格之间的快速求交实现。
//
// 宽相位通过 BVH 收集包围盒重叠的三角形对。窄相位采用“三角形-平面裁剪 + 共线区间求交”：
//
//   1. 若 B 的三个顶点全部位于 A 平面的同一侧（或反之），立即判为不相交；
//   2. 用 B 的平面裁剪 A，用 A 的平面裁剪 B；两段裁剪结果都落在两平面的
//      交线上；
//   3. 把两段裁剪结果投影到交线方向，得到两个一维区间并求交集。最终端点直接
//      取原始裁剪点，而不是用一维参数反算三维坐标，这样在近退化情形下更稳定。
//
// 共面（或近平行）的三角形对转到二维处理：选择与法向最对齐的坐标轴做投影，
// 丢弃该分量后用 Sutherland-Hodgman 裁剪两个凸三角形。
//
// 近共面情形：当任一侧顶点落在 distanceTol 带内时，“在带内”不再等价于“在平面上”，
// 此时改用未归一化有向体积的可靠符号（double 过滤 + 精确展开）构造真实
// 截面，避免把三个带内顶点截断成任意一条边（结果与顶点次序无关）。
//
// 性能相关约定：
//   * 每个三角形的单位法向、平面信息和最长边在预处理阶段算好，窄相位不再重复开方；
//   * BVH 建在按质心排序的紧凑数组上（缓存友好），两棵树同步遍历收集候选对，
//     两侧都是内部节点时优先展开 AABB 尺寸较大的一侧；
//   * 候选对先过三角形 AABB 测试，再进入窄相位；
//   * 窄相位按候选区间分块并行。命中按来源三角形编号排序后焊接、去重及缝合，
//     代表点与重复线段的来源独立于 BVH 遍历顺序和线程数。

#include "mesh_inter.h"

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace mesh_inter {
namespace {

// ---------------------------------------------------------------------------
// 轻量向量运算：只用 double 和栈上定长数组，不依赖 OCC、不做堆分配，
// 便于在窄相位里被调用上百万次。
// ---------------------------------------------------------------------------

struct V3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

inline V3 MakeV3(double x, double y, double z)
{
    V3 v;
    v.x = x; v.y = y; v.z = z;
    return v;
}

inline V3 ToV3(const gp_XYZ& p) { return MakeV3(p.X(), p.Y(), p.Z()); }
inline gp_XYZ ToGp(const V3& p)  { return gp_XYZ(p.x, p.y, p.z); }

inline V3    Add(const V3& a, const V3& b) { return MakeV3(a.x + b.x, a.y + b.y, a.z + b.z); }
inline V3    Sub(const V3& a, const V3& b) { return MakeV3(a.x - b.x, a.y - b.y, a.z - b.z); }
inline V3    Mul(const V3& a, double s)    { return MakeV3(a.x * s, a.y * s, a.z * s); }
inline double Dot(const V3& a, const V3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline V3    Cross(const V3& a, const V3& b)
{
    return MakeV3(a.y * b.z - a.z * b.y,
                  a.z * b.x - a.x * b.z,
                  a.x * b.y - a.y * b.x);
}
inline double Len2(const V3& a) { return Dot(a, a); }
inline double Len(const V3& a)  { return std::sqrt(Dot(a, a)); }
inline double Dist2(const V3& a, const V3& b) { return Len2(Sub(a, b)); }

inline double MaxEdgeLength(const V3 tri[3])
{
    const double e0 = Len2(Sub(tri[1], tri[0]));
    const double e1 = Len2(Sub(tri[2], tri[1]));
    const double e2 = Len2(Sub(tri[0], tri[2]));
    return std::sqrt(std::max(e0, std::max(e1, e2)));
}

// ---------------------------------------------------------------------------
// 单个三角形的预处理数据：顶点、包围盒、质心，以及预先算好的单位法向、
// 法向长度（= 2×面积）和最长边长，避免窄相位反复做叉积与开方。
// ---------------------------------------------------------------------------

struct TriData
{
    V3            v[3];                 // 三个顶点
    V3            bmin;                 // AABB 最小角
    V3            bmax;                 // AABB 最大角
    V3            cen;                  // 质心（BVH 排序键）
    V3            unit;                 // 单位法向
    double        normalLen = 0.0;      // |edge0 × edge1|，等于 2 倍面积
    double        maxEdge = 0.0;        // 最长边长（退化判据的尺度基准）
    std::uint32_t id = 0;               // 在调用方 TriMesh 中的三角形下标
};

// 计算单个三角形的预处理数据；面积过小（退化）时返回 false。
bool MakeTriData(const V3 v[3], std::uint32_t id, const MeshIntersectOptions& options, TriData& out)
{
    const V3 normal = Cross(Sub(v[1], v[0]), Sub(v[2], v[0]));
    const double normal_len = Len(normal);
    const double max_edge = MaxEdgeLength(v);
    if (normal_len <= options.degenerateTol * max_edge * max_edge)
    {
        return false;
    }

    out.v[0] = v[0];
    out.v[1] = v[1];
    out.v[2] = v[2];
    out.unit = Mul(normal, 1.0 / normal_len);
    out.normalLen = normal_len;
    out.maxEdge = max_edge;
    out.id = id;

    out.bmin = v[0];
    out.bmax = v[0];
    for (int i = 1; i < 3; ++i)
    {
        out.bmin.x = std::min(out.bmin.x, v[i].x);
        out.bmin.y = std::min(out.bmin.y, v[i].y);
        out.bmin.z = std::min(out.bmin.z, v[i].z);
        out.bmax.x = std::max(out.bmax.x, v[i].x);
        out.bmax.y = std::max(out.bmax.y, v[i].y);
        out.bmax.z = std::max(out.bmax.z, v[i].z);
    }
    out.cen = Mul(Add(Add(v[0], v[1]), v[2]), 1.0 / 3.0);
    return true;
}

// 两个三角形 AABB 的重叠测试（按容差膨胀），用于在进入窄相位前快速剔除。
inline bool TriBoxesOverlap(const TriData& a, const TriData& b, double tol)
{
    return (a.bmin.x - tol <= b.bmax.x) && (b.bmin.x - tol <= a.bmax.x)
        && (a.bmin.y - tol <= b.bmax.y) && (b.bmin.y - tol <= a.bmax.y)
        && (a.bmin.z - tol <= b.bmax.z) && (b.bmin.z - tol <= a.bmax.z);
}

// ---------------------------------------------------------------------------
// 窄相位
// ---------------------------------------------------------------------------

enum class TriTriType
{
    None,
    Point,
    Segment,
    Coplanar,
    Degenerate
};

struct TriTriResult
{
    TriTriType type = TriTriType::None;
    gp_XYZ p0;              // Point：接触点；Segment：起点
    gp_XYZ p1;              // Segment：终点
    gp_XYZ poly[6];         // Coplanar：相交凸多边形顶点
    int polyCount = 0;
};

// 用一组逐顶点有符号距离表示的平面裁剪一个三角形。
// out 最多写入两个点，返回有效点个数：
//   0 -> 三角形整体位于平面一侧（无交）
//   1 -> 只有一个顶点/切点落在平面上
//   2 -> 得到一段裁剪线段
int ClipTriangleByPlane(const V3 tri[3], const double dist[3], double tol, V3 out[2])
{
    V3 tmp[4];
    int count = 0;

    // 先收集体检落在平面上的顶点（容差内）。
    for (int i = 0; i < 3; ++i)
    {
        if (std::fabs(dist[i]) <= tol)
        {
            tmp[count++] = tri[i];
        }
    }

    // 严格跨越平面（两端符号相反且都超出容差）的边，会与平面产生一个交点。
    for (int i = 0; i < 3; ++i)
    {
        const int j = (i + 1) % 3;
        const bool plus_minus = (dist[i] > tol && dist[j] < -tol);
        const bool minus_plus = (dist[i] < -tol && dist[j] > tol);
        if (plus_minus || minus_plus)
        {
            const double s = dist[i] / (dist[i] - dist[j]);
            tmp[count++] = Add(tri[i], Mul(Sub(tri[j], tri[i]), s));
        }
    }

    // 几何上一个三角形最多只会被平面裁出两个点，这里再做一次防御性钳制。
    if (count > 2)
    {
        count = 2;
    }
    for (int i = 0; i < count; ++i)
    {
        out[i] = tmp[i];
    }
    return count;
}

// ---------------------------------------------------------------------------
// 可靠符号判定：区分“距离很小（仍在容差带内）”和“位于平面上”。
//
// 距离判定（单位法向点积）在容差带 |d| <= distanceTol 内无法分辨顶点究竟在平面
// 哪一侧：A1 样例中六个顶点全部落在带内，但确实存在一条交线段。因此这里改用
// 未归一化的有向体积 det(p-q0, q1-q0, q2-q0)（= 法向长度 × 有符号距离）求符号：
//   1. 先用 double 加与运算尺度相关的误差界过滤；
//   2. 误差过大时，对原始 double 输入做精确展开。只有精确消去才返回零。
// 误差界与无误差变换见 Shewchuk 的 orient3d 和浮点展开算法：
// https://www.cs.cmu.edu/~quake/robust.html
// ---------------------------------------------------------------------------

struct TwoTerm
{
    double hi;
    double lo;
};

inline TwoTerm TwoSum(double a, double b)
{
    const double sum = a + b;
    const double bb = sum - a;
    return { sum, (a - (sum - bb)) + (b - bb) };
}

inline TwoTerm TwoProd(double a, double b)
{
    const double product = a * b;
    return { product, std::fma(a, b, -product) };
}

// 6 个三重积，每个差分最多 2 项，每个标量三重积最多展开为 4 项。
// 按增量 grow-expansion 累加并消去零项，最大分量给出精确符号。
struct VolumeExpansion
{
    std::array<double, 6 * 2 * 2 * 2 * 4> components;
    std::size_t size = 0;

    void Add(double carry)
    {
        if (carry == 0.0)
        {
            return;
        }
        std::size_t count = 0;
        for (std::size_t i = 0; i < size; ++i)
        {
            const TwoTerm sum = TwoSum(carry, components[i]);
            if (sum.lo != 0.0)
            {
                components[count++] = sum.lo;
            }
            carry = sum.hi;
        }
        if (carry != 0.0)
        {
            components[count++] = carry;
        }
        size = count;
    }

    void AddTripleProduct(const TwoTerm& a, const TwoTerm& b, const TwoTerm& c, double sign)
    {
        for (double x : { a.hi, a.lo })
        {
            for (double y : { b.hi, b.lo })
            {
                if (x == 0.0 || y == 0.0)
                {
                    continue;
                }
                const TwoTerm xy = TwoProd(x, y);
                for (double z : { sign * c.hi, sign * c.lo })
                {
                    if (z == 0.0)
                    {
                        continue;
                    }
                    const TwoTerm high = TwoProd(xy.hi, z);
                    const TwoTerm low = TwoProd(xy.lo, z);
                    Add(low.lo);
                    Add(low.hi);
                    Add(high.lo);
                    Add(high.hi);
                }
            }
        }
    }
};

int ExactVolumeSign(const V3& p, const V3& q0, const V3& q1, const V3& q2, double& volume)
{
    const TwoTerm ax = TwoSum(p.x, -q0.x), ay = TwoSum(p.y, -q0.y), az = TwoSum(p.z, -q0.z);
    const TwoTerm bx = TwoSum(q1.x, -q0.x), by = TwoSum(q1.y, -q0.y), bz = TwoSum(q1.z, -q0.z);
    const TwoTerm cx = TwoSum(q2.x, -q0.x), cy = TwoSum(q2.y, -q0.y), cz = TwoSum(q2.z, -q0.z);
    VolumeExpansion result;
    result.AddTripleProduct(ax, by, cz, 1.0);
    result.AddTripleProduct(ax, bz, cy, -1.0);
    result.AddTripleProduct(ay, bz, cx, 1.0);
    result.AddTripleProduct(ay, bx, cz, -1.0);
    result.AddTripleProduct(az, bx, cy, 1.0);
    result.AddTripleProduct(az, by, cx, -1.0);

    volume = 0.0;
    for (std::size_t i = 0; i < result.size; ++i)
    {
        volume += result.components[i];
    }
    if (result.size == 0)
    {
        return 0;
    }
    return result.components[result.size - 1] > 0.0 ? 1 : -1;
}

// 求 p 相对平面 (q0,q1,q2) 的有向体积及其可靠符号（-1/0/+1）。
// volume 输出双精度体积，供插值比例使用。
inline int ReliableVolumeSign(const V3& p, const V3& q0, const V3& q1, const V3& q2,
                              double relative_error, double& volume)
{
    const V3 a = Sub(p, q0);
    const V3 b = Sub(q1, q0);
    const V3 c = Sub(q2, q0);

    const double by_cz = b.y * c.z, bz_cy = b.z * c.y;
    const double bz_cx = b.z * c.x, bx_cz = b.x * c.z;
    const double bx_cy = b.x * c.y, by_cx = b.y * c.x;
    const double det = a.x * (by_cz - bz_cy) + a.y * (bz_cx - bx_cz) + a.z * (bx_cy - by_cx);
    // 叉积相减前的各项绝对值覆盖消去误差；同时约束插值所需的体积精度。
    const double permanent = std::fabs(a.x) * (std::fabs(by_cz) + std::fabs(bz_cy))
                           + std::fabs(a.y) * (std::fabs(bz_cx) + std::fabs(bx_cz))
                           + std::fabs(a.z) * (std::fabs(bx_cy) + std::fabs(by_cx));
    const double error = (8.0 * std::numeric_limits<double>::epsilon()) * permanent;
    if (std::fabs(det) * relative_error > error)
    {
        volume = det;
        return (det > 0.0) ? 1 : -1;
    }
    // 轴对齐面上某一坐标精确相等是常见的零体积情形。
    if ((a.x == 0.0 && b.x == 0.0 && c.x == 0.0)
        || (a.y == 0.0 && b.y == 0.0 && c.y == 0.0)
        || (a.z == 0.0 && b.z == 0.0 && c.z == 0.0))
    {
        volume = 0.0;
        return 0;
    }
    return ExactVolumeSign(p, q0, q1, q2, volume);
}

// 求三角形与另一三角形所在平面的真实交集，只按可靠符号分类，不再用容差截断：
//   * 符号为零的顶点视为位于平面上，保留一次；
//   * 两端异号的边按 t = d0/(d0-d1) 插值出交点；
//   * 整条边（或整个三角形）在平面上时保留其端点。
// 返回 0/1/2 表示点或线段的端点数；返回 3 表示三个顶点全部落在平面上，
// 调用方应转入共面处理。
int BuildPlaneSection(const TriData& tri, const TriData& plane, double tol, V3 out[2])
{
    // 预留插值误差预算，使体积舍入造成的端点偏差远小于距离容差。
    const double relative_error = 0.125 * std::clamp(tol / tri.maxEdge, 0.0, 1.0);
    int    sign[3] = { 0, 0, 0 };
    double volume[3] = { 0.0, 0.0, 0.0 };
    for (int i = 0; i < 3; ++i)
    {
        sign[i] = ReliableVolumeSign(tri.v[i], plane.v[0], plane.v[1], plane.v[2], relative_error, volume[i]);
    }

    if (sign[0] == 0 && sign[1] == 0 && sign[2] == 0)
    {
        return 3;
    }

    int count = 0;
    for (int i = 0; i < 3; ++i)
    {
        if (sign[i] == 0)
        {
            out[count++] = tri.v[i];
        }
    }

    for (int i = 0; i < 3 && count < 2; ++i)
    {
        const int j = (i + 1) % 3;
        if (sign[i] == 0 || sign[j] == 0 || sign[i] == sign[j])
        {
            continue;
        }
        const double denom = volume[i] - volume[j];
        const double t = (denom != 0.0) ? (volume[i] / denom) : 0.5;
        out[count++] = Add(tri.v[i], Mul(Sub(tri.v[j], tri.v[i]), t));
    }
    return count;
}

inline bool IsFiniteV3(const V3& p)
{
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

// 共面情形的二维裁剪，返回相交多边形顶点数（0..6）。
int ClipCoplanar2D(const V3 tri_a[3], const V3 tri_b[3], const V3& normal, double tol, V3 out[6])
{
    // 丢弃与法向最对齐的坐标分量，把问题降到二维。
    int drop_axis = 0;
    double best = std::fabs(normal.x);
    if (std::fabs(normal.y) > best) { drop_axis = 1; best = std::fabs(normal.y); }
    if (std::fabs(normal.z) > best) { drop_axis = 2; best = std::fabs(normal.z); }

    int axis0 = 0;
    int axis1 = 1;
    if (drop_axis == 0)      { axis0 = 1; axis1 = 2; }
    else if (drop_axis == 1) { axis0 = 0; axis1 = 2; }
    else                     { axis0 = 0; axis1 = 1; }

    const auto comp = [](const V3& p, int axis) {
        return (axis == 0) ? p.x : (axis == 1 ? p.y : p.z);
    };

    // 把两个三角形投影到二维，并统一调整为逆时针方向，保证裁剪测试一致。
    double poly_a[3][2];
    double poly_b[3][2];
    for (int i = 0; i < 3; ++i)
    {
        poly_a[i][0] = comp(tri_a[i], axis0);
        poly_a[i][1] = comp(tri_a[i], axis1);
        poly_b[i][0] = comp(tri_b[i], axis0);
        poly_b[i][1] = comp(tri_b[i], axis1);
    }

    const auto signed_area = [](const double p[3][2]) {
        return 0.5 * ((p[1][0] - p[0][0]) * (p[2][1] - p[0][1])
                    - (p[2][0] - p[0][0]) * (p[1][1] - p[0][1]));
    };
    const auto swap_rows = [](double p[3][2]) {
        for (int c = 0; c < 2; ++c)
        {
            const double t = p[1][c];
            p[1][c] = p[2][c];
            p[2][c] = t;
        }
    };
    if (signed_area(poly_a) < 0.0) swap_rows(poly_a);
    if (signed_area(poly_b) < 0.0) swap_rows(poly_b);

    // Sutherland-Hodgman：用凸三角形 B 的每条边依次裁剪 A 的多边形。
    double cur[8][2];
    for (int i = 0; i < 3; ++i)
    {
        cur[i][0] = poly_a[i][0];
        cur[i][1] = poly_a[i][1];
    }
    int count = 3;

    double nxt[8][2];
    for (int edge = 0; edge < 3 && count > 0; ++edge)
    {
        const int next_edge = (edge + 1) % 3;
        const double ex = poly_b[next_edge][0] - poly_b[edge][0];
        const double ey = poly_b[next_edge][1] - poly_b[edge][1];

        int out_count = 0;
        for (int i = 0; i < count; ++i)
        {
            const int j = (i + 1) % count;
            const double di = ex * (cur[i][1] - poly_b[edge][1]) - ey * (cur[i][0] - poly_b[edge][0]);
            const double dj = ex * (cur[j][1] - poly_b[edge][1]) - ey * (cur[j][0] - poly_b[edge][0]);

            if (di >= 0.0 && out_count < 8)
            {
                nxt[out_count][0] = cur[i][0];
                nxt[out_count][1] = cur[i][1];
                ++out_count;
            }
            if (((di > 0.0 && dj < 0.0) || (di < 0.0 && dj > 0.0)) && out_count < 8)
            {
                const double s = di / (di - dj);
                nxt[out_count][0] = cur[i][0] + s * (cur[j][0] - cur[i][0]);
                nxt[out_count][1] = cur[i][1] + s * (cur[j][1] - cur[i][1]);
                ++out_count;
            }
        }

        count = out_count;
        for (int i = 0; i < count; ++i)
        {
            cur[i][0] = nxt[i][0];
            cur[i][1] = nxt[i][1];
        }
    }

    if (count < 2)
    {
        return 0;
    }

    // 还原到三维：被丢弃的坐标由平面方程反算得到。
    const double normal_drop = comp(normal, drop_axis);
    const double plane_d = Dot(normal, tri_a[0]);
    int out_count = 0;
    for (int i = 0; i < count; ++i)
    {
        V3 p;
        p.x = (drop_axis == 0) ? 0.0 : cur[i][0];
        p.y = (drop_axis == 1) ? 0.0 : (drop_axis == 0 ? cur[i][0] : cur[i][1]);
        p.z = (drop_axis == 2) ? 0.0 : cur[i][1];

        double rest = 0.0;
        if (drop_axis != 0) rest += normal.x * p.x;
        if (drop_axis != 1) rest += normal.y * p.y;
        if (drop_axis != 2) rest += normal.z * p.z;
        const double dropped = (plane_d - rest) / normal_drop;
        if (drop_axis == 0)      p.x = dropped;
        else if (drop_axis == 1) p.y = dropped;
        else                     p.z = dropped;

        // 裁剪过程可能产生重合的相邻顶点，这里按容差去重。
        if (out_count > 0 && Dist2(p, out[out_count - 1]) <= tol * tol)
        {
            continue;
        }
        if (out_count >= 6)
        {
            break;
        }
        out[out_count] = p;
        ++out_count;
    }

    // 首尾也可能是同一个点，闭合前去重。
    if (out_count > 1 && Dist2(out[0], out[out_count - 1]) <= tol * tol)
    {
        --out_count;
    }

    return out_count;
}

// 共面重叠：投影到主轴平面做二维凸多边形裁剪，并填写 TriTriResult。
TriTriResult CoplanarResult(const TriData& tri_a, const TriData& tri_b, double tol)
{
    TriTriResult result;
    V3 poly[6];
    const int count = ClipCoplanar2D(tri_a.v, tri_b.v, tri_a.unit, tol, poly);
    result.polyCount = count;
    for (int i = 0; i < count; ++i)
    {
        result.poly[i] = ToGp(poly[i]);
    }
    if (count >= 3)
    {
        result.type = TriTriType::Coplanar;
    }
    else if (count == 2)
    {
        result.type = TriTriType::Segment;
        result.p0 = result.poly[0];
        result.p1 = result.poly[1];
    }
    else
    {
        result.type = TriTriType::None;
        result.polyCount = 0;
    }
    return result;
}

TriTriResult NarrowPhase(const TriData& tri_a, const TriData& tri_b, const MeshIntersectOptions& options)
{
    TriTriResult result;
    const double tol = options.distanceTol;

    // 网格预处理已过滤退化三角形，这里也检查直接传入的三角形数据。
    if (tri_a.normalLen <= options.degenerateTol * tri_a.maxEdge * tri_a.maxEdge
        || tri_b.normalLen <= options.degenerateTol * tri_b.maxEdge * tri_b.maxEdge)
    {
        result.type = TriTriType::Degenerate;
        return result;
    }

    const V3& unit_a = tri_a.unit;
    const V3& unit_b = tri_b.unit;

    // --- 同侧快速剔除 -----------------------------------------------------
    // 计算 B 的三个顶点到 A 平面的有符号距离；同号说明 B 整体在平面一侧。
    double dist_b[3];
    bool b_all_pos = true;
    bool b_all_neg = true;
    for (int i = 0; i < 3; ++i)
    {
        dist_b[i] = Dot(unit_a, Sub(tri_b.v[i], tri_a.v[0]));
        b_all_pos = b_all_pos && (dist_b[i] > tol);
        b_all_neg = b_all_neg && (dist_b[i] < -tol);
    }
    if (b_all_pos || b_all_neg)
    {
        return result;   // B 完全位于平面 A 的一侧
    }

    double dist_a[3];
    bool a_all_pos = true;
    bool a_all_neg = true;
    for (int i = 0; i < 3; ++i)
    {
        dist_a[i] = Dot(unit_b, Sub(tri_a.v[i], tri_b.v[0]));
        a_all_pos = a_all_pos && (dist_a[i] > tol);
        a_all_neg = a_all_neg && (dist_a[i] < -tol);
    }
    if (a_all_pos || a_all_neg)
    {
        return result;   // A 完全位于平面 B 的一侧
    }

    // --- 平行平面：共面则做二维裁剪，否则无交 ------------------------------
    // 两个单位法向的叉积长度等于夹角正弦值，可同时作为“平行程度”的度量。
    V3 direction = Cross(unit_a, unit_b);
    double len_direction = Len(direction);
    if (len_direction <= options.angularTol)
    {
        const double band = (options.coplanarBand > 0.0) ? options.coplanarBand : tol;
        bool coplanar = true;
        for (int i = 0; i < 3 && coplanar; ++i)
        {
            coplanar = (std::fabs(dist_b[i]) <= band);
        }
        if (!coplanar)
        {
            return result;   // 两平面平行且不共面，无交
        }
        return CoplanarResult(tri_a, tri_b, tol);
    }
    direction = Mul(direction, 1.0 / len_direction);

    // --- 用对方平面分别裁剪两个三角形 -------------------------------------
    // 裁剪结果都落在两平面的交线上，后续只需在这个一维方向上比较区间。
    //
    // 只有当某一侧的顶点全部严格远离另一平面（超出 distanceTol）时，“在容差带内”
    // 才等价于“落在平面上”，此时沿用原有快路径；否则必须用可靠符号构造真实截面，
    // 否则三个带内顶点会被截断成任意一条边（A1 样例的根因）。
    bool band_hit = false;
    for (int i = 0; i < 3 && !band_hit; ++i)
    {
        band_hit = (std::fabs(dist_b[i]) <= tol) || (std::fabs(dist_a[i]) <= tol);
    }

    V3  clip_a[2];
    V3  clip_b[2];
    int n_clip_a = 0;
    int n_clip_b = 0;

    if (band_hit)
    {
        n_clip_a = BuildPlaneSection(tri_a, tri_b, tol, clip_a);
        n_clip_b = BuildPlaneSection(tri_b, tri_a, tol, clip_b);
        if (n_clip_a >= 3 || n_clip_b >= 3)
        {
            // 一侧三角形整体贴在对方平面上：按共面重叠处理。
            return CoplanarResult(tri_a, tri_b, tol);
        }
        if (n_clip_a == 0 || n_clip_b == 0)
        {
            return result;   // 严格同侧且没有零点：该三角形没有平面截面
        }
    }
    else
    {
        n_clip_a = ClipTriangleByPlane(tri_a.v, dist_a, tol, clip_a);
        if (n_clip_a == 0)
        {
            return result;
        }
        n_clip_b = ClipTriangleByPlane(tri_b.v, dist_b, tol, clip_b);
        if (n_clip_b == 0)
        {
            return result;
        }
    }

    // --- 在公共交线上做一维区间求交 ---------------------------------------
    // 端点直接取原始裁剪点（而不是由参数反算），因此不会引入额外的数值误差。
    const V3& origin = tri_a.v[0];
    const V3& a_last = clip_a[n_clip_a - 1];
    const V3& b_last = clip_b[n_clip_b - 1];
    const double ta0 = Dot(direction, Sub(clip_a[0], origin));
    const double ta1 = Dot(direction, Sub(a_last, origin));
    const V3     a_lo_point = (ta0 <= ta1) ? clip_a[0] : a_last;
    const V3     a_hi_point = (ta0 <= ta1) ? a_last : clip_a[0];
    const double a_lo = std::min(ta0, ta1);
    const double a_hi = std::max(ta0, ta1);

    const double tb0 = Dot(direction, Sub(clip_b[0], origin));
    const double tb1 = Dot(direction, Sub(b_last, origin));
    const V3     b_lo_point = (tb0 <= tb1) ? clip_b[0] : b_last;
    const V3     b_hi_point = (tb0 <= tb1) ? b_last : clip_b[0];
    const double b_lo = std::min(tb0, tb1);
    const double b_hi = std::max(tb0, tb1);

    const double lo = std::max(a_lo, b_lo);
    const double hi = std::min(a_hi, b_hi);
    if (lo - hi > tol)
    {
        return result;   // 两个一维区间不重叠
    }

    const V3 lo_point = (a_lo >= b_lo) ? a_lo_point : b_lo_point;
    const V3 hi_point = (a_hi <= b_hi) ? a_hi_point : b_hi_point;
    if (!IsFiniteV3(lo_point) || !IsFiniteV3(hi_point))
    {
        return result;   // 输入含非有限值时的防御性回退
    }

    if (hi - lo <= tol)
    {
        result.type = TriTriType::Point;
        result.p0 = ToGp(Mul(Add(lo_point, hi_point), 0.5));
        result.p1 = result.p0;
        return result;
    }

    result.type = TriTriType::Segment;
    result.p0 = ToGp(lo_point);
    result.p1 = ToGp(hi_point);
    return result;
}

// ---------------------------------------------------------------------------
// 宽相位：每侧网格各建一棵 BVH，然后做双树同步遍历
// ---------------------------------------------------------------------------

struct BvhNode
{
    V3  bmin;
    V3  bmax;
    int start = 0;
    int count = 0;
    int child[2] = { -1, -1 };
};

// 按质心排序的 BVH 条目。构建过程只访问这个紧凑数组（16 字节/项），
// 而不是反复跳读体积大得多的三角形记录，可显著减少缓存缺失。
struct BvhItem
{
    V3  cen;
    int tri = 0;
};

class MeshBvh
{
public:
    // 构造即完成建树；items_ 会在构建过程中按质心重排。
    explicit MeshBvh(const std::vector<TriData>& tris)
        : tris_(tris)
    {
        items_.resize(tris_.size());
        for (std::size_t i = 0; i < tris_.size(); ++i)
        {
            items_[i].cen = tris_[i].cen;
            items_[i].tri = static_cast<int>(i);
        }
        nodes_.reserve(tris_.size() / 2 + 1);
        if (!tris_.empty())
        {
            Build(0, static_cast<int>(tris_.size()), 0);
        }
    }

    const std::vector<BvhNode>& Nodes() const { return nodes_; }
    const std::vector<BvhItem>& Items() const { return items_; }

// 递归建树：返回当前节点在 nodes_ 中的下标。
// 叶子最多 8 个三角形；内部节点沿质心分布最长轴做中位数切分。
private:
    int Build(int lo, int hi, int depth)
    {
        const int node_index = static_cast<int>(nodes_.size());
        nodes_.push_back(BvhNode());

        const int leaf_size = 8;
        if (hi - lo <= leaf_size || depth >= 48)
        {
            const TriData& first = tris_[items_[lo].tri];
            V3 bmin = first.bmin;
            V3 bmax = first.bmax;
            for (int i = lo + 1; i < hi; ++i)
            {
                const TriData& tri = tris_[items_[i].tri];
                bmin.x = std::min(bmin.x, tri.bmin.x);
                bmin.y = std::min(bmin.y, tri.bmin.y);
                bmin.z = std::min(bmin.z, tri.bmin.z);
                bmax.x = std::max(bmax.x, tri.bmax.x);
                bmax.y = std::max(bmax.y, tri.bmax.y);
                bmax.z = std::max(bmax.z, tri.bmax.z);
            }
            nodes_[node_index].bmin = bmin;
            nodes_[node_index].bmax = bmax;
            nodes_[node_index].start = lo;
            nodes_[node_index].count = hi - lo;
            return node_index;
        }

        // 选轴只需要一个“有代表性”的质心分布：这里以固定步长抽样若干点求包围盒，
        // 避免每个节点都对整个区间做一次全量扫描。
        // 节点的包围盒则在递归返回后由两个子节点合并得到（自底向上），
        // 因此整个构建过程不存在逐节点全量扫描。
        const int sample_stride = std::max(1, (hi - lo) / 32);
        V3 cmin = items_[lo].cen;
        V3 cmax = cmin;
        for (int i = lo + sample_stride; i < hi; i += sample_stride)
        {
            const V3& c = items_[i].cen;
            cmin.x = std::min(cmin.x, c.x);
            cmin.y = std::min(cmin.y, c.y);
            cmin.z = std::min(cmin.z, c.z);
            cmax.x = std::max(cmax.x, c.x);
            cmax.y = std::max(cmax.y, c.y);
            cmax.z = std::max(cmax.z, c.z);
        }

        const double extent_x = cmax.x - cmin.x;
        const double extent_y = cmax.y - cmin.y;
        const double extent_z = cmax.z - cmin.z;

        int axis = 0;
        if (extent_y > extent_x && extent_y >= extent_z) axis = 1;
        else if (extent_z > extent_x && extent_z > extent_y) axis = 2;

        const int mid = lo + (hi - lo) / 2;
        std::nth_element(items_.begin() + lo, items_.begin() + mid, items_.begin() + hi,
                         [axis](const BvhItem& lhs, const BvhItem& rhs) {
                             const double a = (axis == 0) ? lhs.cen.x : (axis == 1 ? lhs.cen.y : lhs.cen.z);
                             const double b = (axis == 0) ? rhs.cen.x : (axis == 1 ? rhs.cen.y : rhs.cen.z);
                             return a < b;
                         });

        const int child0 = Build(lo, mid, depth + 1);
        const int child1 = Build(mid, hi, depth + 1);

        const BvhNode& node0 = nodes_[child0];
        const BvhNode& node1 = nodes_[child1];
        nodes_[node_index].bmin.x = std::min(node0.bmin.x, node1.bmin.x);
        nodes_[node_index].bmin.y = std::min(node0.bmin.y, node1.bmin.y);
        nodes_[node_index].bmin.z = std::min(node0.bmin.z, node1.bmin.z);
        nodes_[node_index].bmax.x = std::max(node0.bmax.x, node1.bmax.x);
        nodes_[node_index].bmax.y = std::max(node0.bmax.y, node1.bmax.y);
        nodes_[node_index].bmax.z = std::max(node0.bmax.z, node1.bmax.z);
        nodes_[node_index].child[0] = child0;
        nodes_[node_index].child[1] = child1;
        nodes_[node_index].count = 0;
        return node_index;
    }

    const std::vector<TriData>& tris_;
    std::vector<BvhNode>        nodes_;
    std::vector<BvhItem>        items_;
};

inline bool BoxesOverlap(const BvhNode& a, const BvhNode& b, double tol)
{
    return (a.bmin.x - tol <= b.bmax.x) && (b.bmin.x - tol <= a.bmax.x)
        && (a.bmin.y - tol <= b.bmax.y) && (b.bmin.y - tol <= a.bmax.y)
        && (a.bmin.z - tol <= b.bmax.z) && (b.bmin.z - tol <= a.bmax.z);
}

inline double BoxExtent(const BvhNode& node)
{
    return (node.bmax.x - node.bmin.x) + (node.bmax.y - node.bmin.y) + (node.bmax.z - node.bmin.z);
}

struct PairRef
{
    int a = 0;
    int b = 0;
};

// 双树同步遍历：每次取一个节点对，先做包围盒重叠测试；两侧都是叶子时枚举
// 三角形对，否则优先细分体积更大的那一侧，直到落到叶子。每个重叠的叶子对
// 只被收集一次，后续再统一做窄相位。
void CollectPairs(const MeshBvh& bvh_a, const MeshBvh& bvh_b, double tol,
                  const std::vector<TriData>& tris_a, const std::vector<TriData>& tris_b,
                  std::vector<PairRef>& pairs, MeshIntersectStats& stats)
{
    const std::vector<BvhNode>& nodes_a = bvh_a.Nodes();
    const std::vector<BvhNode>& nodes_b = bvh_b.Nodes();
    const std::vector<BvhItem>& items_a = bvh_a.Items();
    const std::vector<BvhItem>& items_b = bvh_b.Items();
    if (nodes_a.empty() || nodes_b.empty())
    {
        return;
    }

    struct NodePair
    {
        int a;
        int b;
    };

    std::vector<NodePair> stack;
    stack.reserve(64);
    stack.push_back(NodePair{ 0, 0 });

    while (!stack.empty())
    {
        const NodePair current = stack.back();
        stack.pop_back();
        ++stats.bvhPairs;

        const BvhNode& node_a = nodes_a[current.a];
        const BvhNode& node_b = nodes_b[current.b];
        if (!BoxesOverlap(node_a, node_b, tol))
        {
            continue;
        }

        const bool leaf_a = (node_a.child[0] < 0);
        const bool leaf_b = (node_b.child[0] < 0);

        if (leaf_a && leaf_b)
        {
            for (int i = 0; i < node_a.count; ++i)
            {
                const int ia = items_a[node_a.start + i].tri;
                for (int j = 0; j < node_b.count; ++j)
                {
                    const int ib = items_b[node_b.start + j].tri;
                    ++stats.candidatePairs;
                    if (!TriBoxesOverlap(tris_a[ia], tris_b[ib], tol))
                    {
                        ++stats.boxRejected;
                        continue;
                    }
                    pairs.push_back(PairRef{ ia, ib });
                }
            }
            continue;
        }

        // 两侧都是内部节点时展开尺寸（AABB 三轴长度之和）较大的一侧：这样每次
        // 展开都能尽快把大盒细分到与另一侧同量级，节点对访问数显著下降。
        // 尺寸相等时沿用基线的选择（展开 B）。
        const bool expand_b = leaf_a || (!leaf_b && BoxExtent(node_b) >= BoxExtent(node_a));
        if (!expand_b)
        {
            stack.push_back(NodePair{ node_a.child[0], current.b });
            stack.push_back(NodePair{ node_a.child[1], current.b });
        }
        else
        {
            stack.push_back(NodePair{ current.a, node_b.child[0] });
            stack.push_back(NodePair{ current.a, node_b.child[1] });
        }
    }
}

// 对收集到的候选对执行窄相位。候选对区间被切成固定的连续块分配给各线程，
// 合并时按块顺序拼接，因此输出结果与线程调度无关（确定性）。
// 候选对较少时直接走单线程，避免线程创建开销。
void ComputePairs(const std::vector<TriData>& tris_a, const std::vector<TriData>& tris_b,
                  const std::vector<PairRef>& pairs, const MeshIntersectOptions& options,
                  std::vector<MeshIntersectSegment>& raw, MeshIntersectStats& stats)
{
    const auto run = [&](std::size_t begin, std::size_t end,
                         std::vector<MeshIntersectSegment>& local,
                         MeshIntersectStats& local_stats) {
        for (std::size_t i = begin; i < end; ++i)
        {
            const TriData& tri_a = tris_a[pairs[i].a];
            const TriData& tri_b = tris_b[pairs[i].b];

            const TriTriResult hit = NarrowPhase(tri_a, tri_b, options);
            switch (hit.type)
            {
            case TriTriType::None:
                break;
            case TriTriType::Degenerate:
                ++local_stats.degenerateTris;
                break;
            case TriTriType::Point:
                ++local_stats.pointHits;
                break;
            case TriTriType::Segment:
            {
                ++local_stats.rawSegments;
                MeshIntersectSegment segment;
                segment.p0 = hit.p0;
                segment.p1 = hit.p1;
                segment.triA = tri_a.id;
                segment.triB = tri_b.id;
                segment.coplanar = false;
                local.push_back(segment);
                break;
            }
            case TriTriType::Coplanar:
            {
                if (!options.includeCoplanar || hit.polyCount < 3)
                {
                    break;
                }
                ++local_stats.coplanarPairs;
                for (int v = 0; v < hit.polyCount; ++v)
                {
                    ++local_stats.rawSegments;
                    MeshIntersectSegment segment;
                    segment.p0 = hit.poly[v];
                    segment.p1 = hit.poly[(v + 1) % hit.polyCount];
                    segment.triA = tri_a.id;
                    segment.triB = tri_b.id;
                    segment.coplanar = true;
                    local.push_back(segment);
                }
                break;
            }
            }
        }
    };

    unsigned hardware = std::thread::hardware_concurrency();
    if (hardware == 0)
    {
        hardware = 1;
    }
    int thread_count = static_cast<int>(std::min<unsigned>(hardware, 4));
    // 环境变量 MESH_INTER_THREADS 可强制线程数：设为 1 即走单线程路径
    // （便于调试，或调用方本身已在工作线程中、不希望再嵌套并行）。
    if (const char* env_threads = std::getenv("MESH_INTER_THREADS"))
    {
        const int requested = std::atoi(env_threads);
        if (requested > 0)
        {
            thread_count = requested;
        }
    }
    if (pairs.size() < 50000 || thread_count <= 1)
    {
        run(0, pairs.size(), raw, stats);
        return;
    }

    std::vector<std::vector<MeshIntersectSegment>> locals(static_cast<std::size_t>(thread_count));
    std::vector<MeshIntersectStats> local_stats(static_cast<std::size_t>(thread_count));
    std::vector<std::thread> threads;
    threads.reserve(static_cast<std::size_t>(thread_count));

    const std::size_t chunk = (pairs.size() + static_cast<std::size_t>(thread_count) - 1)
                            / static_cast<std::size_t>(thread_count);
    for (int t = 0; t < thread_count; ++t)
    {
        const std::size_t begin = std::min(pairs.size(), chunk * static_cast<std::size_t>(t));
        const std::size_t end = std::min(pairs.size(), begin + chunk);
        threads.emplace_back([&, t, begin, end]() {
            run(begin, end, locals[static_cast<std::size_t>(t)], local_stats[static_cast<std::size_t>(t)]);
        });
    }
    for (std::thread& thread : threads)
    {
        thread.join();
    }

    for (int t = 0; t < thread_count; ++t)
    {
        const std::size_t index = static_cast<std::size_t>(t);
        raw.insert(raw.end(), locals[index].begin(), locals[index].end());
        stats.degenerateTris += local_stats[index].degenerateTris;
        stats.coplanarPairs += local_stats[index].coplanarPairs;
        stats.pointHits += local_stats[index].pointHits;
        stats.rawSegments += local_stats[index].rawSegments;
    }
}

// 把 TriMesh 中的三角形转换为预处理数据；退化（面积过小）的三角形计入统计后跳过。
void BuildTriangleData(const TriMesh& mesh, const MeshIntersectOptions& options,
                       std::vector<TriData>& out, MeshIntersectStats& stats)
{
    out.clear();
    out.reserve(mesh.tris.size());
    const std::size_t node_count = mesh.nodes.size();

    for (std::size_t t = 0; t < mesh.tris.size(); ++t)
    {
        const TriMeshTri& tri = mesh.tris[t];
        if (tri.v0 >= node_count || tri.v1 >= node_count || tri.v2 >= node_count)
        {
            continue;   // 索引非法，直接跳过该三角形
        }

        const V3 corners[3] = { ToV3(mesh.nodes[tri.v0]),
                                ToV3(mesh.nodes[tri.v1]),
                                ToV3(mesh.nodes[tri.v2]) };
        TriData data;
        if (!MakeTriData(corners, static_cast<std::uint32_t>(t), options, data))
        {
            ++stats.degenerateTris;
            continue;
        }
        out.push_back(data);
    }
}

// ---------------------------------------------------------------------------
// 后处理：端点吸附、重复线段去除、折线缝合
// ---------------------------------------------------------------------------

struct CellKey
{
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;

    bool operator==(const CellKey& other) const
    {
        return x == other.x && y == other.y && z == other.z;
    }
};

inline std::uint64_t MixBits(std::uint64_t v)
{
    v += 0x9e3779b97f4a7c15ull;
    v = (v ^ (v >> 30)) * 0xbf58476d1ce4e5b9ull;
    v = (v ^ (v >> 27)) * 0x94d049bb133111ebull;
    return v ^ (v >> 31);
}

struct CellKeyHash
{
    std::size_t operator()(const CellKey& key) const
    {
        std::uint64_t h = MixBits(static_cast<std::uint64_t>(key.x));
        h ^= MixBits(static_cast<std::uint64_t>(key.y) + 0x165667b19e3779f9ull);
        h ^= MixBits(static_cast<std::uint64_t>(key.z) + 0x27d4eb2f165667c5ull);
        return static_cast<std::size_t>(h);
    }
};

// 端点吸附器：把落在同一网格单元邻域内（3×3×3 个格子、半径 mergeTol）的点
// 合并成同一个节点编号。网格哈希让查找保持在 O(1) 量级。
class NodeSnapper
{
public:
    explicit NodeSnapper(double tol)
        : tol_(tol > 0.0 ? tol : 1e-12)
        , inv_(1.0 / tol_)
        , tol2_(tol_ * tol_)
    {
    }

    int Snap(const V3& p)
    {
        const std::int64_t base_x = Quantize(p.x);
        const std::int64_t base_y = Quantize(p.y);
        const std::int64_t base_z = Quantize(p.z);

        int    best = -1;
        double best_dist2 = tol2_;
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                for (int dz = -1; dz <= 1; ++dz)
                {
                    const CellKey cell{ base_x + dx, base_y + dy, base_z + dz };
                    const auto it = cells_.find(cell);
                    if (it == cells_.end())
                    {
                        continue;
                    }
                    for (const int index : it->second)
                    {
                        const double dist2 = Dist2(p, points_[index]);
                        if (dist2 <= best_dist2)
                        {
                            best_dist2 = dist2;
                            best = index;
                        }
                    }
                }
            }
        }
        if (best >= 0)
        {
            return best;
        }

        const int id = static_cast<int>(points_.size());
        points_.push_back(p);
        cells_[CellKey{ base_x, base_y, base_z }].push_back(id);
        return id;
    }

    int FindOrAdd(const gp_XYZ& p)
    {
        return Snap(ToV3(p));
    }

    const std::vector<V3>& Points() const { return points_; }

private:
    std::int64_t Quantize(double value) const
    {
        return static_cast<std::int64_t>(std::llround(value * inv_));
    }

    double                            tol_;
    double                            inv_;
    double                            tol2_;
    std::vector<V3>                   points_;
    std::unordered_map<CellKey, std::vector<int>, CellKeyHash> cells_;
};

// 依据端点邻接关系把线段串成折线：先走开放链（从度数 != 2 的节点起步），
// 剩余的线段自然构成闭合环；首尾节点相同则标记为闭合折线。
std::vector<MeshPolyline> StitchFromNodes(const std::vector<V3>& points,
                                          const std::vector<int>& node_a,
                                          const std::vector<int>& node_b)
{
    std::vector<MeshPolyline> polylines;
    const std::size_t segment_count = node_a.size();
    if (segment_count == 0)
    {
        return polylines;
    }

    std::vector<std::vector<int>> adjacency(points.size());
    for (std::size_t i = 0; i < segment_count; ++i)
    {
        adjacency[node_a[i]].push_back(static_cast<int>(i));
        adjacency[node_b[i]].push_back(static_cast<int>(i));
    }

    std::vector<char> used(segment_count, 0);
    const auto next_unused = [&](int node) {
        for (const int segment : adjacency[node])
        {
            if (!used[segment])
            {
                return segment;
            }
        }
        return -1;
    };

    const auto walk = [&](int start_node, int start_segment) {
        MeshPolyline polyline;
        polyline.points.push_back(ToGp(points[start_node]));

        int at = start_node;
        int current = start_segment;
        while (current >= 0)
        {
            used[current] = 1;
            const int other = (node_a[current] == at) ? node_b[current] : node_a[current];
            polyline.points.push_back(ToGp(points[other]));
            at = other;
            current = next_unused(at);
        }

        polyline.closed = (at == start_node && polyline.points.size() > 2);
        if (polyline.points.size() < 2)
        {
            return;
        }
        polylines.push_back(std::move(polyline));
    };

    // 先处理开放链：从度数不等于 2 的节点出发，这样分叉/端点都能被逐条走完。
    for (int node = 0; node < static_cast<int>(adjacency.size()); ++node)
    {
        if (adjacency[node].size() == 2)
        {
            continue;
        }
        for (const int segment : adjacency[node])
        {
            if (!used[segment])
            {
                walk(node, segment);
            }
        }
    }

    // 剩下的线段一定属于闭合环（所有节点度数均为 2）。
    for (std::size_t i = 0; i < segment_count; ++i)
    {
        if (!used[i])
        {
            walk(node_a[i], static_cast<int>(i));
        }
    }

    return polylines;
}

// 用给定弦高/转角误差剖分一张面，并展平为 TriMesh。
bool MeshFace(const TopoDS_Face& face, double linear_deflection, double angular_deflection,
              TriMesh& mesh, std::string& error)
{
    if (face.IsNull())
    {
        error = "null face";
        return false;
    }

    try
    {
        BRepMesh_IncrementalMesh mesher(face, linear_deflection, Standard_False,
                                        angular_deflection, Standard_True);
        mesher.Perform();
    }
    catch (const Standard_Failure& failure)
    {
        const char* message = failure.GetMessageString();
        error = (message != nullptr && message[0] != '\0') ? message : "face tessellation failed";
        return false;
    }

    TopLoc_Location location;
    Handle(Poly_Triangulation) triangulation = BRep_Tool::Triangulation(face, location);
    if (triangulation.IsNull() || triangulation->NbTriangles() <= 0)
    {
        error = "face has no triangulation";
        return false;
    }

    mesh = TriMeshFromTriangulation(*triangulation, location);
    if (mesh.tris.empty())
    {
        error = "face triangulation is empty";
        return false;
    }
    return true;
}

} // namespace

// ---------------------------------------------------------------------------
// 对外接口实现
// ---------------------------------------------------------------------------

std::size_t TriMesh::NodeCount() const { return nodes.size(); }
std::size_t TriMesh::TriangleCount() const { return tris.size(); }
bool TriMesh::IsEmpty() const { return tris.empty(); }

void TriMesh::Clear()
{
    nodes.clear();
    tris.clear();
}

TriMesh TriMeshFromTriangulation(const Poly_Triangulation& triangulation,
                                 const TopLoc_Location&    location)
{
    TriMesh mesh;

    const int node_count = triangulation.NbNodes();
    const int tri_count = triangulation.NbTriangles();
    if (node_count <= 0 || tri_count <= 0)
    {
        return mesh;
    }

    mesh.nodes.resize(static_cast<std::size_t>(node_count));
    const bool identity = location.IsIdentity();
    const gp_Trsf trsf = identity ? gp_Trsf() : location.Transformation();

    for (int i = 1; i <= node_count; ++i)
    {
        const gp_Pnt& point = triangulation.Node(i);
        mesh.nodes[static_cast<std::size_t>(i - 1)] =
            identity ? point.XYZ() : point.Transformed(trsf).XYZ();
    }

    mesh.tris.reserve(static_cast<std::size_t>(tri_count));
    for (int i = 1; i <= tri_count; ++i)
    {
        int n1 = 0;
        int n2 = 0;
        int n3 = 0;
        triangulation.Triangle(i).Get(n1, n2, n3);
        if (n1 < 1 || n2 < 1 || n3 < 1 || n1 > node_count || n2 > node_count || n3 > node_count)
        {
            continue;
        }
        mesh.tris.push_back(TriMeshTri{ static_cast<std::uint32_t>(n1 - 1),
                                        static_cast<std::uint32_t>(n2 - 1),
                                        static_cast<std::uint32_t>(n3 - 1) });
    }

    return mesh;
}

MeshIntersectResult IntersectMeshes(const TriMesh& mesh_a, const TriMesh& mesh_b,
                                    const MeshIntersectOptions& options)
{
    MeshIntersectResult result;
    if (mesh_a.tris.empty() || mesh_b.tris.empty() || mesh_a.nodes.empty() || mesh_b.nodes.empty())
    {
        return result;
    }

    // --- 网格级包围盒快速剔除 --------------------------------------------
    // 两侧整体 AABB 都不重叠时，无需建树即可返回空结果。
    V3 a_min = ToV3(mesh_a.nodes.front());
    V3 a_max = a_min;
    for (const gp_XYZ& p : mesh_a.nodes)
    {
        a_min.x = std::min(a_min.x, p.X()); a_min.y = std::min(a_min.y, p.Y()); a_min.z = std::min(a_min.z, p.Z());
        a_max.x = std::max(a_max.x, p.X()); a_max.y = std::max(a_max.y, p.Y()); a_max.z = std::max(a_max.z, p.Z());
    }
    V3 b_min = ToV3(mesh_b.nodes.front());
    V3 b_max = b_min;
    for (const gp_XYZ& p : mesh_b.nodes)
    {
        b_min.x = std::min(b_min.x, p.X()); b_min.y = std::min(b_min.y, p.Y()); b_min.z = std::min(b_min.z, p.Z());
        b_max.x = std::max(b_max.x, p.X()); b_max.y = std::max(b_max.y, p.Y()); b_max.z = std::max(b_max.z, p.Z());
    }

    const BvhNode mesh_box_a{ a_min, a_max, 0, 0, { -1, -1 } };
    const BvhNode mesh_box_b{ b_min, b_max, 0, 0, { -1, -1 } };
    if (!BoxesOverlap(mesh_box_a, mesh_box_b, options.distanceTol))
    {
        return result;
    }
    result.bboxOverlap = true;

    // --- 宽相位：预处理三角形 + 建两棵 BVH + 双树遍历收集候选对 ----------
    std::vector<TriData> tris_a;
    std::vector<TriData> tris_b;
    BuildTriangleData(mesh_a, options, tris_a, result.stats);
    BuildTriangleData(mesh_b, options, tris_b, result.stats);
    if (tris_a.empty() || tris_b.empty())
    {
        return result;
    }

    const MeshBvh bvh_a(tris_a);
    const MeshBvh bvh_b(tris_b);

    std::vector<PairRef> pairs;
    CollectPairs(bvh_a, bvh_b, options.distanceTol, tris_a, tris_b, pairs, result.stats);
    result.stats.triPairs = pairs.size();

    std::vector<MeshIntersectSegment> raw;
    ComputePairs(tris_a, tris_b, pairs, options, raw, result.stats);

    // 焊接的代表点与重复边的来源都取决于输入顺序。按公开三角下标排序命中，
    // 保留同一三角对的多边形边次序，使装配与 BVH 遍历和窄相位线程数无关。
    std::stable_sort(raw.begin(), raw.end(), [](const auto& a, const auto& b) {
        return a.triA < b.triA || (a.triA == b.triA && a.triB < b.triB);
    });

    // --- 后处理：端点焊接 + 重复线段去除 ----------------------------------
    // 相邻三角形对经常上报同一条重叠线段（共享网格边），因此这里按“焊接后的
    // 节点编号”做键去重，而不是比较原始坐标；同时也会丢弃短于 mergeTol 的线段。
    NodeSnapper snapper(options.mergeTol);
    std::unordered_set<std::uint64_t> seen;
    seen.reserve(raw.size() * 2);

    std::vector<int>         node_a;
    std::vector<int>         node_b;
    std::vector<std::size_t> raw_index;
    node_a.reserve(raw.size());
    node_b.reserve(raw.size());
    raw_index.reserve(raw.size());

    for (std::size_t i = 0; i < raw.size(); ++i)
    {
        const int id_a = snapper.FindOrAdd(raw[i].p0);
        const int id_b = snapper.FindOrAdd(raw[i].p1);
        if (id_a == id_b)
        {
            ++result.stats.pointHits;
            continue;
        }
        const int lo = (id_a < id_b) ? id_a : id_b;
        const int hi = (id_a < id_b) ? id_b : id_a;
        const std::uint64_t key = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(lo)) << 32)
                                | static_cast<std::uint32_t>(hi);
        if (!seen.insert(key).second)
        {
            ++result.stats.duplicates;
            continue;
        }
        node_a.push_back(id_a);
        node_b.push_back(id_b);
        raw_index.push_back(i);
    }

    const std::vector<V3>& node_points = snapper.Points();
    result.segments.reserve(node_a.size());
    for (std::size_t i = 0; i < node_a.size(); ++i)
    {
        const MeshIntersectSegment& source = raw[raw_index[i]];
        MeshIntersectSegment segment;
        segment.p0 = ToGp(node_points[node_a[i]]);
        segment.p1 = ToGp(node_points[node_b[i]]);
        segment.triA = source.triA;
        segment.triB = source.triB;
        segment.coplanar = source.coplanar;
        result.segments.push_back(segment);
    }

    // --- 折线缝合 ----------------------------------------------------------
    if (options.buildPolylines)
    {
        result.polylines = StitchFromNodes(node_points, node_a, node_b);
        if (options.minPolylineLength > 0.0)
        {
            std::vector<MeshPolyline> kept;
            kept.reserve(result.polylines.size());
            for (MeshPolyline& polyline : result.polylines)
            {
                double length = 0.0;
                for (std::size_t i = 1; i < polyline.points.size(); ++i)
                {
                    length += (polyline.points[i] - polyline.points[i - 1]).Modulus();
                }
                if (length >= options.minPolylineLength)
                {
                    kept.push_back(std::move(polyline));
                }
            }
            result.polylines.swap(kept);
        }
    }

    return result;
}

MeshIntersectResult IntersectTriangulations(const Poly_Triangulation& tri_a,
                                            const TopLoc_Location&    loc_a,
                                            const Poly_Triangulation& tri_b,
                                            const TopLoc_Location&    loc_b,
                                            const MeshIntersectOptions& options)
{
    const TriMesh mesh_a = TriMeshFromTriangulation(tri_a, loc_a);
    const TriMesh mesh_b = TriMeshFromTriangulation(tri_b, loc_b);
    return IntersectMeshes(mesh_a, mesh_b, options);
}

FaceIntersectResult IntersectFaces(const TopoDS_Face& face_a, const TopoDS_Face& face_b,
                                   double linear_deflection, double angular_deflection,
                                   const MeshIntersectOptions& options)
{
    FaceIntersectResult result;

    TriMesh mesh_a;
    TriMesh mesh_b;
    if (!MeshFace(face_a, linear_deflection, angular_deflection, mesh_a, result.error))
    {
        return result;
    }
    if (!MeshFace(face_b, linear_deflection, angular_deflection, mesh_b, result.error))
    {
        return result;
    }

    result.mesh = IntersectMeshes(mesh_a, mesh_b, options);
    result.curves = result.mesh.polylines;
    result.ok = true;
    return result;
}

} // namespace mesh_inter
