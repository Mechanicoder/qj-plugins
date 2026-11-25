# 面面求交验证插件

支持 BREP 测例批量读取、OCC 与 SMLib 面面求交、质量/效率对比及模型树输出。

## 第三方库

`cmake/QjcamSMLib.cmake` 统一接入：

- `3rdparty/occ-sm-bridge-2.0.0-source/occ_sm_bridge`：直接从源码构建静态 OccSmBridge，使用 `install-path.txt` 对应 QJCAM 的 OCC 头文件和库，构建产物留在 `build/thirdparty/OccSmBridge`。不使用之前与宿主 OCC ABI 不兼容的预编译 bridge。
- `3rdparty/SMLib-10.0.0-localfixes-win64-release`：SMLib 10 本地修复版 Release SDK，以 `/MD` 配合 Release 或 RelWithDebInfo 使用。
- 编译后自动把 `SMLib.dll`、`tbb12.dll`、`tbbmalloc.dll` 复制至插件 DLL 同目录；桥接静态链接，无需部署 OccSmBridge.dll。测试程序也自动获得同一套依赖。

`QJCAM_SMLIB_ROOT` 和 `QJCAM_BRIDGE_SOURCE_DIR` 可通过 CMake 缓存覆盖；替换库版本时重新构建。

- 四个与 QJCAM example 一致的 C 导出，独立 Ribbon 入口。
- 非模态验证窗口，面 A/B 分别使用宿主 `ModelSelector`，每个最多一个面。
- 正容差输入（模型长度单位，默认 `1e-6`），检查缺失、类型及相同面。
- 单窗口复用，关闭后销毁；工程关闭前销毁选择器，避免跨工程保留选择。
- 输入变化时清除旧状态。所选面和文件测例均使用同一条验证流程。

## 批量测例

在已打开工程的 QJCAM 中打开插件，输入或浏览选择测例根目录，点击「运行全部测例」。仅扫描根目录的直接子目录，按目录名排序，每个测例恰好包含两个扩展名为 `.brep` 的文件（大小写不限），按文件名排序作为 A/B。每个文件必须包含恰好一个拓扑有效的面，允许单面 compound；多面模型、损坏文件和文件数量不符会记为测例失败，并继续后续测例。支持中文路径。

```text
测例根目录/
  case-001/
    a.brep
    b.brep
  case-002/
    first.brep
    second.brep
```

点击取消会在当前测例结束后停止后续测例，已输出结果保留。单个 OCC 调用期间界面可能等待。重复运行会增加一个新的批次，不覆盖既有结果。

## 对比与输出

- SDK 7.9.2 不包含旧 `BRepAlgo_Section`，使用 `BRepAlgoAPI_Section`。基准禁用并行、开启非破坏模式与曲线近似，以输入容差设置 fuzzy value。
- 两个算法获得独立几何副本，避免输入修改影响另一方。单次 `steady_clock` 计时排除 OCC 输入副本、读文件、质量检查和模型树输出；SMLib 总耗时包含双向桥接和结果构建，另列 FaceIntersect 内核耗时及转换/结果构建耗时。这是单次测量，不代表稳定基准测试。
- SMLib 使用 `SmFace::FaceIntersect`，处理裁剪边界和孔洞，距离容差取用户输入，角度容差固定为 SDK 推荐的 20°。TBB 在调用期间限制为单线程。输入桥接关闭 healer；报告保留转换警告。
- 消息框「详细信息」以及 QJCAM 消息区输出每个测例的耗时、边/顶点数、交线总长度、拓扑有效性。双方成功时额外输出长度差、边数差、双向采样最大偏差及耗时比 OCC/SMLib（含转换）。
- 偏差检查每条边均匀取 33 个参数点并加入所有顶点，计算到另一结果的最近距离；它不是严格 Hausdorff 距离或完整正确性证明，可能遗漏采样间差异。长度和容差采用模型长度单位。
- 非空结果输出到模型树，名称为「批次 / 测例 / OCC」或「批次 / 测例 / SMLib」；绿色为 OCC，橙色为 SMLib。交线先封装为 SMLib wire BRep，再经 OccSmBridge 精确转回 OCC；孤立交点转为 OCC 顶点。空结果成功不会添加空节点；输出失败会在报告中注明。
- SMLib SDK 的 `CreateWireEdgeFromCurve` 在闭合圆交线的端点粘合阶段存在访问异常；适配层对闭合交线直接创建共享单个端点的边，保留原始曲线及参数范围，不进行离散近似。

`src/intersection_algorithm.cpp` 实现 OCC → OccSmBridge → SMLib FaceIntersect → OCC，负责借用模型与算法结果的生命周期。完成计算返回 `Success`，无交线时返回空集合；失败返回 `Failed`。

手动验收：

1. 按根 README 部署、启用插件，确认入口出现；重复点击仅激活已有窗口。
2. 未选面、只选一个面时点击验证，应提示各选一个有效面。
3. 两次选择同一个面，应拒绝验证。
4. 选择两个相交面，应执行 OCC/SMLib 并分别输出交线、报告质量与效率数据。
5. 重置应清空选择并恢复默认容差；改动选择/容差后状态应变为等待验证。
6. 窗口打开时关闭工程，再新建/打开工程，重新打开插件应无旧选择。
7. 关闭验证窗口后重新打开、退出 QJCAM，应正常释放 UI。

8. 批量运行多个测例，确认逐例报告、交线颜色、命名，以及异常测例不中断后续执行。

## 几何/文件检查

```powershell
cmake -S . -B build -DQJCAM_BUILD_FFI_TESTS=ON
cmake --build build --config RelWithDebInfo --target ffi-validation-tests
$installation = (Get-Content .\install-path.txt -Raw).Trim()
$env:PATH = "$installation/bin;$env:PATH"
.\build\tests\RelWithDebInfo\ffi-validation-tests.exe
```

检查覆盖相交/不相交面、裁剪域无交、带孔裁剪面、圆柱闭合交线、球面孤立切点、相同与偏移结果的采样比较、算法异常隔离、中文路径与 BREP 读取、损坏文件、多面模型和文件数量错误。本机编译与几何检查不能替代上述宿主交互验收。
