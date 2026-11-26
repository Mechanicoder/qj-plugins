// 临时复现程序：直接加载 BREP 测例并调用面面求交，捕获异常并打印调用栈。
#include "face_realtime.h"

#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include <windows.h>
#include <dbghelp.h>

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#pragma comment(lib, "dbghelp.lib")

namespace {

LONG WINAPI crashHandler(EXCEPTION_POINTERS* info)
{
    std::fprintf(stderr, "exception=0x%08lX address=%p\n",
                 info->ExceptionRecord->ExceptionCode,
                 info->ExceptionRecord->ExceptionAddress);
    std::fflush(stderr);

    HANDLE process = GetCurrentProcess();
    HANDLE thread = GetCurrentThread();
    SymSetOptions(SYMOPT_LOAD_LINES | SYMOPT_UNDNAME | SYMOPT_DEFERRED_LOADS);
    SymInitialize(process, nullptr, TRUE);

    CONTEXT context = *info->ContextRecord;
    STACKFRAME64 frame{};
    frame.AddrPC.Offset = context.Rip;
    frame.AddrPC.Mode = AddrModeFlat;
    frame.AddrFrame.Offset = context.Rbp;
    frame.AddrFrame.Mode = AddrModeFlat;
    frame.AddrStack.Offset = context.Rsp;
    frame.AddrStack.Mode = AddrModeFlat;

    for (int index = 0; index < 80; ++index) {
        if (!StackWalk64(IMAGE_FILE_MACHINE_AMD64, process, thread, &frame, &context, nullptr,
                         SymFunctionTableAccess64, SymGetModuleBase64, nullptr))
            break;
        char storage[sizeof(SYMBOL_INFO) + 256] = {};
        auto* symbol = reinterpret_cast<SYMBOL_INFO*>(storage);
        symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
        symbol->MaxNameLen = 255;
        DWORD64 displacement = 0;
        DWORD lineDisplacement = 0;
        IMAGEHLP_LINE64 line{};
        line.SizeOfStruct = sizeof(IMAGEHLP_LINE64);
        std::printf("#%02d %p", index, reinterpret_cast<void*>(frame.AddrPC.Offset));
        if (SymFromAddr(process, frame.AddrPC.Offset, &displacement, symbol))
            std::printf(" %s+0x%llx", symbol->Name,
                        static_cast<unsigned long long>(displacement));
        if (SymGetLineFromAddr64(process, frame.AddrPC.Offset, &lineDisplacement, &line))
            std::printf(" [%s:%lu]", line.FileName, line.LineNumber);
        std::printf("\n");
    }
    std::fflush(stdout);
    return EXCEPTION_EXECUTE_HANDLER;
}

std::vector<TopoDS_Face> readFaces(const char* path)
{
    TopoDS_Shape shape;
    BRep_Builder builder;
    if (!BRepTools::Read(shape, path, builder)) {
        std::fprintf(stderr, "读取失败：%s\n", path);
        return {};
    }
    std::vector<TopoDS_Face> faces;
    for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More(); explorer.Next())
        faces.push_back(TopoDS::Face(explorer.Current()));
    return faces;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::fprintf(stderr, "用法：mech1106-repro <face-a.brep> <face-b.brep> [tol] [angle]\n");
        return 2;
    }
    const double tolerance = argc > 3 ? std::atof(argv[3]) : 1.0e-6;
    const double angle = argc > 4 ? std::atof(argv[4]) : 20.0;

    const std::vector<TopoDS_Face> first = readFaces(argv[1]);
    const std::vector<TopoDS_Face> second = readFaces(argv[2]);
    std::printf("faces: A=%zu B=%zu tol=%g angle=%g\n", first.size(), second.size(),
                tolerance, angle);
    std::fflush(stdout);
    if (first.empty() || second.empty()) return 3;

    SetUnhandledExceptionFilter(crashHandler);

    sfi::FaceGroup group;
    std::string error;
    if (!sfi::prepare(first, second, tolerance, angle, group, error)) {
        std::fprintf(stderr, "prepare 失败：%s\n", error.c_str());
        return 4;
    }

    const sfi::FaceIntersectionRun run = sfi::intersect(group, false, 0.0, 0.0, 0.0);
    std::printf("done: edges=%d vertices=%d ok=%d failed=%d skipped=%d kernel=%.2fms total=%.2fms %s\n",
                run.edges, run.vertices, run.succeededPairs, run.failedPairs, run.skippedPairs,
                run.kernelMilliseconds, run.totalMilliseconds, run.message.c_str());
    return 0;
}
