#include "face_compare_dialog.h"
#include "face_dialog.h"
#include "mesh_dialog.h"

#include <qjcam_api.h>

#include <QAction>
#include <QIcon>
#include <QPointer>

// QJCAM 通过 C 导出名发现插件入口。
#define QJCAM_PLUGIN_EXPORT extern "C" __declspec(dllexport)

namespace {
QPointer<ffi::FaceCompareDialog> compareDialog;
QPointer<sfi::FaceIntersectionDialog> faceDialog;
QPointer<mmi::MeshIntersectionDialog> meshDialog;
QPointer<QAction> compareAction;
QPointer<QAction> faceAction;
QPointer<QAction> meshAction;

// 在宿主释放工程选中的形状之前删除对话框。
bool onProjectClosing()
{
    delete compareDialog.data();
    sfi::FaceIntersectionDialog::setProjectClosing(true);
    delete faceDialog.data();
    mmi::MeshIntersectionDialog::setProjectClosing(true);
    delete meshDialog.data();
    return true;
}

template <class Dialog>
void showDialog(QPointer<Dialog>& dialog, QWidget* parent)
{
    if (!dialog) dialog = new Dialog(parent);
    dialog->show();
    dialog->raise();
    dialog->activateWindow();
}
}

QJCAM_PLUGIN_EXPORT const char* GetPluginName() { return "求交-intersection"; }
QJCAM_PLUGIN_EXPORT const char* GetPluginComment()
{
    return "Face-face (SMLib / OCC comparison) and mesh face intersection with live translation.";
}
QJCAM_PLUGIN_EXPORT const char* GetPluginVersion() { return "0.4.0"; }

QJCAM_PLUGIN_EXPORT bool InitPlugin(const char* pluginId)
{
    if (faceAction) {
        return true;
    }
    auto* mainWindow = qjcam::UI_GetMainWindow();
    if (!pluginId || !*pluginId || !mainWindow) {
        return false;
    }
    if (!qjcam::NT_RegisterProjectClosingNotifier(pluginId, &onProjectClosing)) {
        qjcam::QJ_OutputMessage("intersection: failed to register project closing notifier.");
        return false;
    }

    auto* page = qjcam::UI_AddRibbonPage(QStringLiteral("求交"));
    auto* faceGroup = qjcam::UI_AddRibbonGroup(QStringLiteral("面面求交"), page);
    auto* meshGroup = qjcam::UI_AddRibbonGroup(QStringLiteral("网格求交"), page);

    faceAction = new QAction(QIcon(QStringLiteral(":/intersection/face_face.svg")),
                             QStringLiteral("面面求交"), mainWindow);
    QObject::connect(faceAction.data(), &QAction::triggered, faceAction.data(), [mainWindow]() {
        showDialog(faceDialog, mainWindow);
    });
    qjcam::UI_AddRibbonAction(faceAction.data(), faceGroup, Qt::ToolButtonTextUnderIcon);

    compareAction = new QAction(QIcon(QStringLiteral(":/intersection/face_compare.svg")),
                                QStringLiteral("面面求交对比测试"), mainWindow);
    QObject::connect(compareAction.data(), &QAction::triggered, compareAction.data(), [mainWindow]() {
        showDialog(compareDialog, mainWindow);
    });
    qjcam::UI_AddRibbonAction(compareAction.data(), faceGroup, Qt::ToolButtonTextUnderIcon);

    meshAction = new QAction(QIcon(QStringLiteral(":/intersection/mesh.svg")),
                             QStringLiteral("网格求交"), mainWindow);
    QObject::connect(meshAction.data(), &QAction::triggered, meshAction.data(), [mainWindow]() {
        showDialog(meshDialog, mainWindow);
    });
    qjcam::UI_AddRibbonAction(meshAction.data(), meshGroup, Qt::ToolButtonTextUnderIcon);

    qjcam::QJ_OutputMessage("intersection: face-face and mesh intersection loaded.");
    return true;
}
