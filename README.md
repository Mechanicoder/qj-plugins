# QJCAM 插件集合

每个插件独立目录、独立 DLL。根目录统一管理 SDK 和构建规则，编译结果直接写入 QJCAM 的插件目录。

```text
install-path.txt               QJCAM 安装根目录（一行绝对路径）
cmake/                         公共 SDK 配置与插件构建函数
scripts/build.ps1              统一构建，直接输出至宿主
face-face-intersection/        面面求交验证插件
curve-face-intersection/       线面求交验证插件
mesh-mesh-intersection/        基于 mesh_inter 的网格面面求交插件（移动面实时刷新）
xyz-draw/                      XYZ 点集与闭合多边形绘图插件
3rdparty/                      OccSmBridge 源码与 SMLib Release SDK（不纳入版本库）
build/                         CMake 工程及编译中间文件（不纳入版本库）
```

## 构建与使用

需要 Windows x64、CMake 3.22+、Visual Studio C++ 工具链。包含目录和依赖库来自 `<安装路径>/api`，使用其中的 Qt 5.15.2 和 OpenCascade 7.9.2。支持 Release、RelWithDebInfo，均使用 `/MD`；Debug 与宿主 ABI 不兼容。

面面求交插件还依赖 `3rdparty` 下的 OccSmBridge 和 SMLib：桥接库随项目基于宿主 OCC 从源码编译，SMLib/TBB 运行 DLL 自动部署至对应插件目录。详见 [面面求交说明](face-face-intersection/README.md)。

线面插件使用同一套第三方依赖，支持有限曲线边与裁剪面求交，详见 [线面求交说明](curve-face-intersection/README.md)。可用 `-Plugins curve-face-intersection` 单独构建，两插件为独立 DLL，可分别启用。

网格面面插件直接复用 `mesh-mesh-intersection/mesh_inter.h`：选择两组面（均支持多选）求交线多边形，组内面两两求交后合并；求交后选择移动整组面并调整 X/Y/Z 位移，交线随位移实时刷新，移动面副本与交线显示在模型树中。可用 `-Plugins mesh-mesh-intersection` 单独构建。

在插件管理中启用 `XYZ Draw`，打开工程后从 Ribbon「XYZ 绘图」选择 `.xyz` 文件。每行填写 `x, y, z`，支持空行和可选的行尾反斜杠。点集和多边形可分别绘制，结果显示在模型树「调试」下。多边形按行序连线并闭合，至少需要三个不同顶点。可用 `-Plugins xyz-draw` 单独构建。

`install-path.txt` 当前内容：

```text
D:/Program Files/QJCAM/26Q1.REL.5.1.8_withAPI
```

先关闭 QJCAM，再在仓库根目录运行 PowerShell：

```powershell
# 默认构建全部已启用插件，配置为 RelWithDebInfo。
.\scripts\build.ps1
# 指定插件及发布配置。
.\scripts\build.ps1 -Plugins face-face-intersection -Configuration Release
# 本机 VS2026 首次配置时选择已安装的兼容工具集。
.\scripts\build.ps1 -Toolset v143
```

无需 `-Deploy`、复制或执行 `cmake --install`。DLL 直接输出到：

```text
<安装路径>/plugins/face-face-intersection/face-face-intersection.dll
<安装路径>/plugins/face-face-intersection/face-face-intersection.pdb  # RelWithDebInfo
```

目录名与 DLL 名必须一致，这是 QJCAM 示例中的加载约定。Release 和 RelWithDebInfo 使用同一部署位置，后一次构建会替换前一次 DLL。`.lib` 和其他中间文件留在 `build/`。Program Files 写入权限不足时，需使用有权限的终端。

脚本按 `-QjcamRoot` 参数 → `install-path.txt` → `QJCAM_ROOT` 环境变量的优先级读取安装路径，并检查 QJCAM 是否正在运行。优先使用示例推荐的 VS2019；本机 VS2026 默认 v145 删除了 Qt 5.15.2 依赖的 `stdext` 迭代器，因此使用 v143。工具集首次配置后会保存在 CMake 缓存中。

也可以直接使用 CMake（同样需先关闭 QJCAM）：

```powershell
# 首次配置自动读取 install-path.txt；本机 VS2026 使用 -T v143。
cmake -S . -B build -A x64 -T v143
cmake --build build --config RelWithDebInfo --target face-face-intersection
```

已有 CMake 缓存时，切换安装路径需显式更新 `-DQJCAM_ROOT=...`，或使用干净的 `build` 目录。更换 SDK、生成器或工具集也建议使用干净构建目录。`QJCAM_API_DIR`、`QJCAM_OCC_DIR`、`QJCAM_QT_DIR` 可通过 CMake 缓存变量覆盖以适应其他 SDK 布局；切换 SDK 时应同步更新这些覆盖项。

启动 QJCAM → 插件管理 → 勾选 `Face-Face Intersection` → Ribbon「面面求交」→「面面求交验证」。RelWithDebInfo 可通过 Visual Studio 附加 `qjcam.exe` 调试。SDK 示例只提供初始化入口，本框架沿用进程生命周期，更新 DLL 前须关闭宿主。

## 新增独立插件

1. 新建 `<插件名>/src/`，实现 `GetPluginName`、`GetPluginComment`、`GetPluginVersion`、`InitPlugin` 四个 C 导出，可参考首个插件的 `extern "C" __declspec(dllexport)` 声明。
2. 在插件目录的 `CMakeLists.txt` 调用 `qjcam_add_plugin(<插件名> SOURCES ... OCC_LIBRARIES ...)`。名称决定构建目标、DLL 名和宿主插件子目录。OCC 只列插件额外使用的库。
3. 在根 `CMakeLists.txt` 添加构建开关和 `add_subdirectory(<插件名>)`。脚本默认构建全部已启用插件；`-Plugins <名称1>,<名称2>` 可指定插件。

各插件自行管理 UI 和算法，插件之间无需运行时依赖。
