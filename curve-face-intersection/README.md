# 线面求交验证插件

独立插件 `curve-face-intersection`，插件管理中显示为 `Curve-Face Intersection`。打开工程后，在 Ribbon「线面求交」中点击「线面求交验证」。可选择一条有限曲线边和一个裁剪面单次运行，也可浏览选择测例根目录，点击「运行全部测例」。

```text
测例根目录/
  case-001/
    edge.brep
    face.brep
  case-002/
    a-face.brep
    b-edge.brep
```

扫描根目录的直接子目录，每个目录恰有两个 `.brep` 文件，按模型内容自动识别边和面，文件名顺序不限。每个文件应包含一条边或一个面，允许单边/单面容器；不支持多边线框、多面体、无限边、无三维曲线或退化边。格式错误、类型错误、算法失败会写入消息区及最终消息框，继续后续测例。取消在当前测例结束后生效。

## 算法与输出

- OCC：`IntCurvesFace_ShapeIntersector` 计算有限曲线与裁剪面的交点，`BRepAlgoAPI_Common` 提取重合段；删除重复交点及已包含在重合段中的点。Common 单独使用会漏掉独立穿透交点，因此组合两者。
- SMLib：当前桥接输入不支持独立边根节点，曲线边先通过 OCC 的精确有理 NURBS 转换，再传递控制点、权重和完整节点向量到 `SmBSplineCurve`；包含边的位置变换，不进行采样拟合。面使用基于宿主 OCC 编译的 OccSmBridge 转换，关闭 healer。
- `SmSurface::GlobalCurveIntersect` 输出点解和重合区间。点解通过 `SmFace::Point3DClassify` 按实际裁剪边界/孔洞过滤；重合区间通过 `SmFace::CreateCurvesByTrimming` 裁剪，构造成 SMLib wire BRep 后由 OccSmBridge 转回 OCC。
- SMLib 不会调用 OCC 求交来补齐结果；不支持的解类型和转换失败明确报错。若 OCC 精确 NURBS 转换不支持某种曲线，也会报失败。
- 输出交点（顶点）和重合段（边）到模型树，按「批次 / 测例 / 算法」命名；OCC 为绿色，SMLib 为橙色。重复运行增加新批次；空交不创建空节点。

## 报告与计时

消息框详细信息及 QJCAM 消息区显示算法状态、顶点数、边数、重合总长度、拓扑有效性、长度/边数差及双向采样最大偏差。顶点数包含重合段的端点，并非只统计孤立交点。偏差检查每条边 33 个参数采样点加所有顶点，使用到另一结果的最近距离；这是近似检查，不是严格误差界。

单次 `steady_clock` 计时排除读文件、独立 OCC 输入副本、质量检查及模型树输出。OCC 耗时包含两个基准求交接口及交点整理；SMLib 总耗时包含输入转换、求交、结果构建和清理，另列内核（含原生面裁剪/分类）及转换/结果构建耗时。SMLib 调用期间限制 TBB 为单线程。耗时比为 OCC / SMLib 总耗时；不将一次测量作为稳定性能结论。容差、长度和偏差均采用模型长度单位。

## 构建与检查

构建后直接写入 `install-path.txt` 对应的 `plugins/curve-face-intersection/`，包含 DLL/PDB 和 SMLib/TBB 依赖，与面面插件使用相同 SDK 配置。

```powershell
# 先关闭 QJCAM。
.\scripts\build.ps1 -Plugins curve-face-intersection
# 几何与文件读取测试。
cmake -S . -B build -DQJCAM_BUILD_CFI_TESTS=ON
cmake --build build --config RelWithDebInfo --target cfi-validation-tests
$installation = (Get-Content .\install-path.txt -Raw).Trim()
$env:PATH = "$installation/bin;$env:PATH"
.\build\tests\RelWithDebInfo\cfi-validation-tests.exe
```

检查覆盖穿透、裁剪域外无交、有限边无交、端点、边界、孔洞排除、平面重合、孔洞分段、球面双交点、相切、圆弧、中文路径和两个面输入拒绝。QJCAM 内还需检查选择控件、批量报告、树节点颜色、工程关闭与重新打开行为。
