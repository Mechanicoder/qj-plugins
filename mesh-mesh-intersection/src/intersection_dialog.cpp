#include "intersection_dialog.h"

#include <selector/model_selector.h>

#include <BRepBuilderAPI_Transform.hxx>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <exception>
#include <numeric>

namespace mmi {
namespace {

constexpr double kPi = 3.14159265358979323846;
// 位移小于该值时不显示移动面副本，避免与原面完全重合闪烁。
constexpr double kPreviewEpsilon = 1e-12;

QDoubleSpinBox* makeSpin(QWidget* parent, double minimum, double maximum, int decimals,
                         double step, double value, const QString& suffix = QString())
{
    auto* spin = new QDoubleSpinBox(parent);
    spin->setDecimals(decimals);
    spin->setRange(minimum, maximum);
    spin->setSingleStep(step);
    spin->setValue(value);
    if (!suffix.isEmpty()) spin->setSuffix(suffix);
    spin->setKeyboardTracking(false);
    return spin;
}

} // namespace

bool IntersectionDialog::projectClosing_ = false;

IntersectionDialog::IntersectionDialog(QWidget* parent) : QDialog(parent)
{
    projectClosing_ = false;
    setWindowTitle(QStringLiteral("网格面面求交"));
    setWindowIcon(QIcon(QStringLiteral(":/mesh-mesh-intersection/ribbon_icon.svg")));
    setAttribute(Qt::WA_DeleteOnClose);
    setModal(false); // 模型视图需要保持可拾取。
    resize(560, 0);

    auto* layout = new QVBoxLayout(this);
    auto* hint = new QLabel(QStringLiteral("面 A、面 B 均可多选，同一组面也可同时选入两侧；求交时按两组面逐对计算并合并交线。"
                                           "求交后调整位移可实时刷新交线；交线与移动面副本显示在模型树中。"), this);
    hint->setWordWrap(true);
    layout->addWidget(hint);

    auto* selectionBox = new QGroupBox(QStringLiteral("选择面"), this);
    auto* selectionForm = new QFormLayout(selectionBox);
    using Pickup = qjcam::ModelSelector::PickupType;
    first_ = new qjcam::ModelSelector(selectionBox, Pickup::PT_PickupFace);
    second_ = new qjcam::ModelSelector(selectionBox, Pickup::PT_PickupFace);
    selectionForm->addRow(QStringLiteral("面 A（可多选）"), first_);
    selectionForm->addRow(QStringLiteral("面 B（可多选）"), second_);
    layout->addWidget(selectionBox);

    auto* tessellationBox = new QGroupBox(QStringLiteral("剖分与容差"), this);
    auto* tessellationRow = new QHBoxLayout(tessellationBox);
    tessellationRow->addWidget(new QLabel(QStringLiteral("弦高"), tessellationBox));
    linear_ = makeSpin(tessellationBox, 1.0e-4, 10.0, 4, 0.01, 0.05);
    tessellationRow->addWidget(linear_);
    tessellationRow->addWidget(new QLabel(QStringLiteral("角度"), tessellationBox));
    angular_ = makeSpin(tessellationBox, 1.0, 90.0, 1, 1.0, 20.0, QStringLiteral(" °"));
    tessellationRow->addWidget(angular_);
    tessellationRow->addWidget(new QLabel(QStringLiteral("距离容差"), tessellationBox));
    tolerance_ = makeSpin(tessellationBox, 1.0e-12, 1.0, 9, 1.0e-6, 1.0e-6);
    tessellationRow->addWidget(tolerance_);
    layout->addWidget(tessellationBox);

    auto* moveBox = new QGroupBox(QStringLiteral("移动面与实时求交"), this);
    auto* moveLayout = new QVBoxLayout(moveBox);
    auto* moveRow = new QHBoxLayout;
    moveRow->addWidget(new QLabel(QStringLiteral("移动面"), moveBox));
    moveTarget_ = new QComboBox(moveBox);
    moveTarget_->addItem(QStringLiteral("面 A（整组）"));
    moveTarget_->addItem(QStringLiteral("面 B（整组）"));
    moveTarget_->setCurrentIndex(1);
    moveRow->addWidget(moveTarget_);
    moveRow->addSpacing(12);
    realtime_ = new QCheckBox(QStringLiteral("实时求交"), moveBox);
    realtime_->setChecked(true);
    moveRow->addWidget(realtime_);
    showPreview_ = new QCheckBox(QStringLiteral("显示移动面"), moveBox);
    showPreview_->setChecked(true);
    moveRow->addWidget(showPreview_);
    moveRow->addStretch();
    auto* resetMove = new QPushButton(QStringLiteral("重置位移"), moveBox);
    moveRow->addWidget(resetMove);
    moveLayout->addLayout(moveRow);

    auto* axisRow = new QHBoxLayout;
    auto addAxis = [&](const QString& label, QDoubleSpinBox*& spin) {
        axisRow->addWidget(new QLabel(label, moveBox));
        spin = makeSpin(moveBox, -1.0e6, 1.0e6, 4, 0.5, 0.0);
        axisRow->addWidget(spin);
    };
    addAxis(QStringLiteral("X"), translateX_);
    addAxis(QStringLiteral("Y"), translateY_);
    addAxis(QStringLiteral("Z"), translateZ_);
    moveLayout->addLayout(axisRow);
    layout->addWidget(moveBox);

    status_ = new QLabel(this);
    status_->setWordWrap(true);
    status_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(status_);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
    auto* runButton = buttons->addButton(QStringLiteral("求交"), QDialogButtonBox::ActionRole);
    auto* clearButton = buttons->addButton(QStringLiteral("清除"), QDialogButtonBox::ResetRole);
    layout->addWidget(buttons);

    updateTimer_ = new QTimer(this);
    updateTimer_->setSingleShot(true);
    updateTimer_->setInterval(30);

    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(runButton, &QPushButton::clicked, this, &IntersectionDialog::run);
    connect(clearButton, &QPushButton::clicked, this, &IntersectionDialog::reset);
    connect(resetMove, &QPushButton::clicked, this, [this] {
        suppressUpdates_ = true;
        translateX_->setValue(0.0);
        translateY_->setValue(0.0);
        translateZ_->setValue(0.0);
        suppressUpdates_ = false;
        scheduleUpdate();
    });
    connect(updateTimer_, &QTimer::timeout, this, &IntersectionDialog::run);

    connect(first_, &qjcam::ModelSelector::ShapeChanged, this, &IntersectionDialog::invalidate);
    connect(second_, &qjcam::ModelSelector::ShapeChanged, this, &IntersectionDialog::invalidate);
    connect(moveTarget_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &IntersectionDialog::scheduleUpdate);
    connect(realtime_, &QCheckBox::toggled, this, &IntersectionDialog::scheduleUpdate);
    connect(showPreview_, &QCheckBox::toggled, this, &IntersectionDialog::scheduleUpdate);
    for (QDoubleSpinBox* spin : {linear_, angular_, tolerance_, translateX_, translateY_, translateZ_}) {
        connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &IntersectionDialog::scheduleUpdate);
    }

    setStatus(QStringLiteral("请选择面 A、面 B（可多选）后点击“求交”。"));
}

IntersectionDialog::~IntersectionDialog()
{
    if (updateTimer_) updateTimer_->stop();
    if (!projectClosing_) clearOutput();
}

void IntersectionDialog::setProjectClosing(bool closing)
{
    projectClosing_ = closing;
}

bool IntersectionDialog::readFaces(std::vector<TopoDS_Face>& first, std::vector<TopoDS_Face>& second) const
{
    first.clear();
    second.clear();
    for (const TopoDS_Shape& shape : first_->GetSelectedModels()) {
        if (!shape.IsNull() && shape.ShapeType() == TopAbs_FACE) first.push_back(TopoDS::Face(shape));
    }
    for (const TopoDS_Shape& shape : second_->GetSelectedModels()) {
        if (!shape.IsNull() && shape.ShapeType() == TopAbs_FACE) second.push_back(TopoDS::Face(shape));
    }
    return !first.empty() && !second.empty();
}

bool IntersectionDialog::hasOffset() const
{
    return std::abs(translateX_->value()) > kPreviewEpsilon
        || std::abs(translateY_->value()) > kPreviewEpsilon
        || std::abs(translateZ_->value()) > kPreviewEpsilon;
}

void IntersectionDialog::setStatus(const QString& text)
{
    status_->setText(text);
}

void IntersectionDialog::invalidate()
{
    updateTimer_->stop();
    clearOutput();
    clear(meshes_);
    setStatus(QStringLiteral("面已更改，请重新求交。"));
}

void IntersectionDialog::reset()
{
    suppressUpdates_ = true;
    first_->Clear();
    second_->Clear();
    linear_->setValue(0.05);
    angular_->setValue(20.0);
    tolerance_->setValue(1.0e-6);
    translateX_->setValue(0.0);
    translateY_->setValue(0.0);
    translateZ_->setValue(0.0);
    moveTarget_->setCurrentIndex(1);
    realtime_->setChecked(true);
    showPreview_->setChecked(true);
    suppressUpdates_ = false;
    updateTimer_->stop();
    clearOutput();
    clear(meshes_);
    setStatus(QStringLiteral("请选择面 A、面 B（可多选）后点击“求交”。"));
}

void IntersectionDialog::scheduleUpdate()
{
    if (suppressUpdates_) return;
    if (!meshes_.ready) return;
    if (!realtime_->isChecked()) {
        updatePreview();
        qjcam::VI_UpdateView();
        setStatus(QStringLiteral("实时求交已关闭；点击“求交”刷新交线。"));
        return;
    }
    updateTimer_->start();
}

void IntersectionDialog::run()
{
    if (suppressUpdates_) return;
    std::vector<TopoDS_Face> first;
    std::vector<TopoDS_Face> second;
    if (!readFaces(first, second)) {
        setStatus(QStringLiteral("请为面 A、面 B 至少各选择一个有效面。检查后重新求交。"));
        return;
    }

    try {
        const double linear = linear_->value();
        const double angular = angular_->value() * kPi / 180.0;
        if (!meshes_.ready || !sameFaceLists(meshes_.firstFaces, first)
            || !sameFaceLists(meshes_.secondFaces, second)
            || meshes_.linearDeflection != linear || meshes_.angularDeflection != angular) {
            std::string error;
            if (!tessellate(first, second, linear, angular, meshes_, error)) {
                clearOutput();
                setStatus(QStringLiteral("剖分失败：%1").arg(QString::fromUtf8(error.c_str())));
                return;
            }
        }

        updatePreview();
        const auto result = intersect(meshes_, moveTarget_->currentIndex() == 1,
                                      translateX_->value(), translateY_->value(), translateZ_->value(),
                                      tolerance_->value());
        updateResult(result.result);

        const auto closed = std::count_if(result.result.polylines.begin(), result.result.polylines.end(),
                                          [](const mesh_inter::MeshPolyline& line) { return line.closed; });
        const auto points = std::accumulate(result.result.polylines.begin(), result.result.polylines.end(),
                                            std::size_t{0}, [](std::size_t sum, const mesh_inter::MeshPolyline& line) {
                                                return sum + line.points.size();
                                            });
        QString text = QStringLiteral("面 A %1 个 / 面 B %2 个；交线：%3 条折线（闭合 %4），%5 段，%6 个顶点；%7 ms。网格 %8/%9 个三角形。")
            .arg(static_cast<int>(meshes_.firstFaces.size()))
            .arg(static_cast<int>(meshes_.secondFaces.size()))
            .arg(static_cast<int>(result.result.polylines.size()))
            .arg(static_cast<int>(closed))
            .arg(static_cast<int>(result.result.segments.size()))
            .arg(static_cast<qulonglong>(points))
            .arg(result.milliseconds, 0, 'f', 1)
            .arg(static_cast<qulonglong>(triangleCount(meshes_.firstMeshes)))
            .arg(static_cast<qulonglong>(triangleCount(meshes_.secondMeshes)));
        if (result.result.polylines.empty()) text += QStringLiteral(" 当前两组面无交线。");
        if (!resultNode_ && !result.result.polylines.empty())
            text += QStringLiteral(" 交线输出失败（请确认已打开工程）。");
        setStatus(text);
        qjcam::VI_UpdateView();
    } catch (const Standard_Failure& error) {
        setStatus(QStringLiteral("几何异常：%1").arg(QString::fromUtf8(error.GetMessageString())));
    } catch (const std::exception& error) {
        setStatus(QStringLiteral("求交失败：%1").arg(QString::fromUtf8(error.what())));
    } catch (...) {
        setStatus(QStringLiteral("求交失败：未知异常。"));
    }
}

void IntersectionDialog::updatePreview()
{
    if (!showPreview_->isChecked() || !meshes_.ready || !hasOffset()) {
        removePreview();
        return;
    }
    const TopoDS_Shape& base = moveTarget_->currentIndex() == 1 ? meshes_.secondCompound : meshes_.firstCompound;
    if (base.IsNull()) {
        removePreview();
        return;
    }
    gp_Trsf translation;
    translation.SetTranslation(gp_Vec(translateX_->value(), translateY_->value(), translateZ_->value()));
    const TopoDS_Shape moved = BRepBuilderAPI_Transform(base, translation, Standard_False).Shape();
    if (previewNode_ && !qjcam::MO_SetShape(previewNode_, moved)) previewNode_ = nullptr;
    if (!previewNode_) previewNode_ = qjcam::MO_AddShape("网格求交 / 移动面", moved);
    if (previewNode_) {
        qjcam::MO_SetShapeColor(previewNode_, 0x3399ff);
        qjcam::MO_SetShapeTransparency(previewNode_, 0.4);
    }
}

void IntersectionDialog::updateResult(const mesh_inter::MeshIntersectResult& result)
{
    if (result.polylines.empty()) {
        removeResult();
        return;
    }
    const TopoDS_Shape shape = polylinesShape(result.polylines);
    if (resultNode_ && !qjcam::MO_SetShape(resultNode_, shape)) resultNode_ = nullptr;
    if (!resultNode_) resultNode_ = qjcam::MO_AddShape("网格求交 / 交线", shape);
    if (resultNode_) {
        qjcam::MO_SetShapeColor(resultNode_, 0xff8800);
        qjcam::MO_SetShapeTransparency(resultNode_, 0.0);
    }
}

void IntersectionDialog::removePreview()
{
    if (!previewNode_) return;
    qjcam::MO_DeleteModel(previewNode_);
    previewNode_ = nullptr;
}

void IntersectionDialog::removeResult()
{
    if (!resultNode_) return;
    qjcam::MO_DeleteModel(resultNode_);
    resultNode_ = nullptr;
}

void IntersectionDialog::clearOutput()
{
    removePreview();
    removeResult();
}

} // namespace mmi
