#include "xyz_points.h"

#include <debug/debug_draw.h>
#include <Standard_Failure.hxx>
#include <QAction>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPointer>
#include <QPushButton>
#include <QVBoxLayout>
#include <exception>

#define QJCAM_PLUGIN_EXPORT extern "C" __declspec(dllexport)

namespace {
void openDrawingDialog(QWidget* parent)
{
    QDialog dialog(parent);
    dialog.setWindowTitle(QStringLiteral("XYZ 绘图"));
    dialog.resize(520, 0);

    auto* layout = new QVBoxLayout(&dialog);
    auto* fileRow = new QHBoxLayout;
    auto* path = new QLineEdit(&dialog);
    path->setPlaceholderText(QStringLiteral("XYZ 文件路径"));
    auto* browse = new QPushButton(QStringLiteral("浏览…"), &dialog);
    fileRow->addWidget(path);
    fileRow->addWidget(browse);
    layout->addLayout(fileRow);

    auto* status = new QLabel(&dialog);
    status->setTextFormat(Qt::PlainText);
    status->setWordWrap(true);
    layout->addWidget(status);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, &dialog);
    auto* points = buttons->addButton(QStringLiteral("绘制点集"), QDialogButtonBox::ActionRole);
    auto* polygon = buttons->addButton(QStringLiteral("绘制多边形"), QDialogButtonBox::ActionRole);
    layout->addWidget(buttons);

    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
    QObject::connect(browse, &QPushButton::clicked, &dialog, [&] {
        const auto filename = QFileDialog::getOpenFileName(&dialog, QStringLiteral("打开 XYZ 文件"),
            path->text(), QStringLiteral("XYZ 文件 (*.xyz);;所有文件 (*)"));
        if (!filename.isEmpty()) path->setText(filename);
    });
    QObject::connect(path, &QLineEdit::textChanged, status, &QLabel::clear);

    const auto draw = [&](bool closedPolygon) {
        status->clear();
        try {
            if (qjcam::VI_GetView().IsNull()) {
                QMessageBox::warning(&dialog, QStringLiteral("XYZ 绘图"), QStringLiteral("请先打开工程和模型视图。"));
                return;
            }
            const auto filename = path->text().trimmed();
            auto coordinates = xyz::readPoints(filename);
            if (closedPolygon) coordinates = xyz::polygonVertices(coordinates);

            const auto name = (QFileInfo(filename).fileName()
                + (closedPolygon ? QStringLiteral(" / 多边形") : QStringLiteral(" / 点集"))).toUtf8().toStdString();
            qjcam::DebugDraw::SetDebugDrawEnabled(true);
            if (closedPolygon) qjcam::DebugDraw::Polygon(false, coordinates, true, name, Quantity_NOC_ORANGE);
            else qjcam::DebugDraw::Pnts(coordinates, name, Quantity_NOC_CYAN1);
            qjcam::VI_UpdateView();
            status->setText((closedPolygon ? QStringLiteral("已绘制 %1 个顶点的闭合多边形。")
                                           : QStringLiteral("已绘制 %1 个点。"))
                                .arg(static_cast<qulonglong>(coordinates.size())));
        } catch (const Standard_Failure& error) {
            QMessageBox::warning(&dialog, QStringLiteral("XYZ 绘图失败"), QString::fromUtf8(error.GetMessageString()));
        } catch (const std::exception& error) {
            QMessageBox::warning(&dialog, QStringLiteral("XYZ 绘图失败"), QString::fromUtf8(error.what()));
        }
    };
    QObject::connect(points, &QPushButton::clicked, &dialog, [&] { draw(false); });
    QObject::connect(polygon, &QPushButton::clicked, &dialog, [&] { draw(true); });
    dialog.exec();
}
}

QJCAM_PLUGIN_EXPORT const char* GetPluginName() { return "XYZ Draw"; }
QJCAM_PLUGIN_EXPORT const char* GetPluginComment() { return "Draw XYZ point sets and closed polygons."; }
QJCAM_PLUGIN_EXPORT const char* GetPluginVersion() { return "0.1.0"; }

QJCAM_PLUGIN_EXPORT bool InitPlugin(const char* pluginId)
{
    static QPointer<QAction> action;
    if (action) return true;
    auto* mainWindow = qjcam::UI_GetMainWindow();
    if (!pluginId || !*pluginId || !mainWindow) return false;
    auto* page = qjcam::UI_AddRibbonPage(QStringLiteral("XYZ 绘图"));
    if (!page) return false;
    auto* group = qjcam::UI_AddRibbonGroup(QStringLiteral("导入与绘制"), page);
    if (!group) return false;
    action = new QAction(QStringLiteral("打开 XYZ 绘图"), mainWindow);
    QObject::connect(action.data(), &QAction::triggered, action.data(), [mainWindow] { openDrawingDialog(mainWindow); });
    qjcam::UI_AddRibbonAction(action.data(), group, Qt::ToolButtonTextOnly);
    return true;
}
