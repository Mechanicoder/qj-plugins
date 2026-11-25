#include "intersection_dialog.h"
#include <qjcam_api.h>
#include <QAction>
#include <QPointer>

// QJCAM discovers the plugin entry points by their C export names.
#define QJCAM_PLUGIN_EXPORT extern "C" __declspec(dllexport)

namespace {
QPointer<cfi::IntersectionDialog> dialog;
QPointer<QAction> action;

// Delete immediately before the host releases the project's selected shapes.
bool onProjectClosing()
{
    delete dialog.data();
    return true;
}
}

QJCAM_PLUGIN_EXPORT const char* GetPluginName() { return "Curve-Face Intersection"; }
QJCAM_PLUGIN_EXPORT const char* GetPluginComment() { return "OCC / SMLib curve-face intersection validation using OccSmBridge."; }
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
        qjcam::QJ_OutputMessage("curve-face-intersection: failed to register project closing notifier.");
        return false;
    }
    // A per-plugin group avoids coupling independently loaded DLLs to one another.
    auto* page = qjcam::UI_AddRibbonPage(QStringLiteral("线面求交"));
    auto* group = qjcam::UI_AddRibbonGroup(QStringLiteral("算法验证"), page);
    action = new QAction(QStringLiteral("线面求交验证"), mainWindow);
    QObject::connect(action.data(), &QAction::triggered, action.data(), [mainWindow]() {
        if (!dialog) {
            dialog = new cfi::IntersectionDialog(mainWindow);
        }
        dialog->show();
        dialog->raise();
        dialog->activateWindow();
    });
    qjcam::UI_AddRibbonAction(action.data(), group, Qt::ToolButtonTextOnly);
    qjcam::QJ_OutputMessage("curve-face-intersection: OCC / SMLib validation loaded.");
    return true;
}

