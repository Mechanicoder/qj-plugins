#include "intersection_dialog.h"

#include <qjcam_api.h>

#include <QAction>
#include <QPointer>

// QJCAM 通过 C 导出名发现插件入口。
#define QJCAM_PLUGIN_EXPORT extern "C" __declspec(dllexport)

namespace {
QPointer<mmi::IntersectionDialog> dialog;
QPointer<QAction> action;

// 在宿主释放工程选中的形状之前删除对话框。
bool onProjectClosing()
{
    mmi::IntersectionDialog::setProjectClosing(true);
    delete dialog.data();
    return true;
}
}

QJCAM_PLUGIN_EXPORT const char* GetPluginName() { return "Mesh-Mesh Intersection"; }
QJCAM_PLUGIN_EXPORT const char* GetPluginComment() { return "Mesh-based face intersection with live face translation preview."; }
QJCAM_PLUGIN_EXPORT const char* GetPluginVersion() { return "0.1.0"; }

QJCAM_PLUGIN_EXPORT bool InitPlugin(const char* pluginId)
{
    if (action) {
        return true;
    }
    auto* mainWindow = qjcam::UI_GetMainWindow();
    if (!pluginId || !*pluginId || !mainWindow) {
        return false;
    }
    if (!qjcam::NT_RegisterProjectClosingNotifier(pluginId, &onProjectClosing)) {
        qjcam::QJ_OutputMessage("mesh-mesh-intersection: failed to register project closing notifier.");
        return false;
    }
    auto* page = qjcam::UI_AddRibbonPage(QStringLiteral("网格求交"));
    auto* group = qjcam::UI_AddRibbonGroup(QStringLiteral("算法验证"), page);
    action = new QAction(QStringLiteral("网格求交验证"), mainWindow);
    QObject::connect(action.data(), &QAction::triggered, action.data(), [mainWindow]() {
        if (!dialog) {
            dialog = new mmi::IntersectionDialog(mainWindow);
        }
        dialog->show();
        dialog->raise();
        dialog->activateWindow();
    });
    qjcam::UI_AddRibbonAction(action.data(), group, Qt::ToolButtonTextOnly);
    qjcam::QJ_OutputMessage("mesh-mesh-intersection: mesh face intersection loaded.");
    return true;
}
