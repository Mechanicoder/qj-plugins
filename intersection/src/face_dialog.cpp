#include "face_dialog.h"

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

#include <cmath>
#include <exception>

namespace sfi {
namespace {

constexpr double kOffsetEpsilon = 1e-12;

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

bool FaceIntersectionDialog::projectClosing_ = false;

FaceIntersectionDialog::FaceIntersectionDialog(QWidget* parent) : QDialog(parent)
{
    projectClosing_ = false;
    setWindowTitle(QStringLiteral("面面求交"));
    setWindowIcon(QIcon(QStringLiteral(":/intersection/face_face.svg")));
    setAttribute(Qt::WA_DeleteOnClose);
    setModal(false); // 模型视图需要保持可拾取。
    resize(560, 0);

    auto* layout = new QVBoxLayout(this);
    auto* hint = new QLabel(QStringLiteral("面 A、面 B 均可多选；使用 SMLib FaceIntersect 对两组面逐对求精确交线。"
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

    auto* toleranceBox = new QGroupBox(QStringLiteral("容差"), this);
    auto* toleranceRow = new QHBoxLayout(toleranceBox);
    toleranceRow->addWidget(new QLabel(QStringLiteral("距离容差"), toleranceBox));
    tolerance_ = makeSpin(toleranceBox, 1.0e-12, 1.0, 9, 1.0e-6, 1.0e-6);
    toleranceRow->addWidget(tolerance_);
    toleranceRow->addWidget(new QLabel(QStringLiteral("角度容差"), toleranceBox));
    angle_ = makeSpin(toleranceBox, 1.0, 90.0, 1, 1.0, 20.0, QStringLiteral(" °"));
    toleranceRow->addWidget(angle_);
    toleranceRow->addStretch();
    layout->addWidget(toleranceBox);

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
    auto* saveButton = buttons->addButton(QStringLiteral("保存交线"), QDialogButtonBox::ActionRole);
    auto* caseButton = buttons->addButton(QStringLiteral("生成测例"), QDialogButtonBox::ActionRole);
    auto* clearButton = buttons->addButton(QStringLiteral("清除"), QDialogButtonBox::ResetRole);
    layout->addWidget(buttons);

    updateTimer_ = new QTimer(this);
    updateTimer_->setSingleShot(true);
    updateTimer_->setInterval(30);

    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(runButton, &QPushButton::clicked, this, &FaceIntersectionDialog::run);
    connect(saveButton, &QPushButton::clicked, this, &FaceIntersectionDialog::saveResult);
    connect(caseButton, &QPushButton::clicked, this, &FaceIntersectionDialog::saveTestCase);
    connect(clearButton, &QPushButton::clicked, this, &FaceIntersectionDialog::reset);
    connect(resetMove, &QPushButton::clicked, this, [this] {
        suppressUpdates_ = true;
        translateX_->setValue(0.0);
        translateY_->setValue(0.0);
        translateZ_->setValue(0.0);
        suppressUpdates_ = false;
        scheduleUpdate();
    });
    connect(updateTimer_, &QTimer::timeout, this, &FaceIntersectionDialog::run);

    connect(first_, &qjcam::ModelSelector::ShapeChanged, this, &FaceIntersectionDialog::invalidate);
    connect(second_, &qjcam::ModelSelector::ShapeChanged, this, &FaceIntersectionDialog::invalidate);
    connect(moveTarget_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &FaceIntersectionDialog::scheduleUpdate);
    connect(realtime_, &QCheckBox::toggled, this, &FaceIntersectionDialog::scheduleUpdate);
    connect(showPreview_, &QCheckBox::toggled, this, &FaceIntersectionDialog::scheduleUpdate);
    for (QDoubleSpinBox* spin : {tolerance_, angle_, translateX_, translateY_, translateZ_}) {
        connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
                this, &FaceIntersectionDialog::scheduleUpdate);
    }

    setStatus(QStringLiteral("请选择面 A、面 B（可多选）后点击“求交”。"));
}

FaceIntersectionDialog::~FaceIntersectionDialog()
{
    if (updateTimer_) updateTimer_->stop();
    if (!projectClosing_) clearOutput();
}

void FaceIntersectionDialog::setProjectClosing(bool closing)
{
    projectClosing_ = closing;
}

bool FaceIntersectionDialog::readFaces(std::vector<TopoDS_Face>& first, std::vector<TopoDS_Face>& second) const
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

bool FaceIntersectionDialog::hasOffset() const
{
    return std::abs(translateX_->value()) > kOffsetEpsilon
        || std::abs(translateY_->value()) > kOffsetEpsilon
        || std::abs(translateZ_->value()) > kOffsetEpsilon;
}

void FaceIntersectionDialog::setStatus(const QString& text)
{
    status_->setText(text);
}

void FaceIntersectionDialog::invalidate()
{
    updateTimer_->stop();
    clearOutput();
    clear(group_);
    setStatus(QStringLiteral("面已更改，请重新求交。"));
}

void FaceIntersectionDialog::reset()
{
    suppressUpdates_ = true;
    first_->Clear();
    second_->Clear();
    tolerance_->setValue(1.0e-6);
    angle_->setValue(20.0);
    translateX_->setValue(0.0);
    translateY_->setValue(0.0);
    translateZ_->setValue(0.0);
    moveTarget_->setCurrentIndex(1);
    realtime_->setChecked(true);
    showPreview_->setChecked(true);
    suppressUpdates_ = false;
    updateTimer_->stop();
    clearOutput();
    clear(group_);
    setStatus(QStringLiteral("请选择面 A、面 B（可多选）后点击“求交”。"));
}

void FaceIntersectionDialog::saveResult()
{
    if (!resultNode_) {
        setStatus(QStringLiteral("当前没有可保存的交线，请先求交。"));
        return;
    }
    const TopoDS_Shape shape = qjcam::MO_GetShape(resultNode_);
    if (shape.IsNull()) {
        setStatus(QStringLiteral("当前交线为空，无法保存。"));
        return;
    }
    static int serial = 0;
    const QString name = QStringLiteral("面面求交交线 %1").arg(++serial);
    qjcam::ModelHandle saved = qjcam::MO_AddShape(name.toUtf8().constData(), shape);
    if (!saved) {
        setStatus(QStringLiteral("保存失败：请确认已打开工程。"));
        return;
    }
    qjcam::MO_SetShapeColor(saved, 0xff8800);
    qjcam::MO_SetShapeTransparency(saved, 0.0);
    removeResult(); // 保存后删除临时节点，避免重复显示。
    setStatus(QStringLiteral("已保存到模型树：%1").arg(name));
    qjcam::VI_UpdateView();
}

void FaceIntersectionDialog::saveTestCase()
{
    std::vector<TopoDS_Face> first;
    std::vector<TopoDS_Face> second;
    if (!readFaces(first, second)) {
        setStatus(QStringLiteral("请为面 A、面 B 至少各选择一个有效面。"));
        return;
    }

    const double tolerance = tolerance_->value();
    const double angle = angle_->value();
    if (!group_.ready || !sameFaceLists(group_.firstFaces, first)
        || !sameFaceLists(group_.secondFaces, second)
        || group_.tolerance != tolerance || group_.angleTolerance != angle) {
        std::string error;
        if (!prepare(first, second, tolerance, angle, group_, error)) {
            setStatus(QStringLiteral("生成测例失败：%1").arg(QString::fromUtf8(error.c_str())));
            return;
        }
    }

    const bool moveSecond = moveTarget_->currentIndex() == 1;
    gp_Trsf translation;
    if (hasOffset())
        translation.SetTranslation(gp_Vec(translateX_->value(), translateY_->value(), translateZ_->value()));
    auto placed = [&](const TopoDS_Shape& shape, bool move) {
        return move ? BRepBuilderAPI_Transform(shape, translation, Standard_False).Shape() : shape;
    };

    static int serial = 0;
    const int index = ++serial;
    const QString params = QStringLiteral("tol=%1，ang=%2°，%3 位移 %4, %5, %6")
        .arg(tolerance, 0, 'g', 6)
        .arg(angle, 0, 'g', 6)
        .arg(moveSecond ? QStringLiteral("B") : QStringLiteral("A"))
        .arg(translateX_->value(), 0, 'g', 6)
        .arg(translateY_->value(), 0, 'g', 6)
        .arg(translateZ_->value(), 0, 'g', 6);
    const QString prefix = QStringLiteral("面面求交测例 %1").arg(index);

    bool ok = true;
    auto addShape = [&](const QString& label, const TopoDS_Shape& shape,
                        unsigned int color, double transparency) {
        const QString name = QStringLiteral("%1 / %2（%3）").arg(prefix, label, params);
        qjcam::ModelHandle node = qjcam::MO_AddShape(name.toUtf8().constData(), shape);
        if (!node) {
            ok = false;
            return;
        }
        qjcam::MO_SetShapeColor(node, color);
        qjcam::MO_SetShapeTransparency(node, transparency);
    };
    addShape(QStringLiteral("面 A"), placed(group_.firstCompound, !moveSecond), 0x3399ff, 0.35);
    addShape(QStringLiteral("面 B"), placed(group_.secondCompound, moveSecond), 0xffaa33, 0.35);

    setStatus(ok ? QStringLiteral("已生成测例：%1（%2）").arg(prefix, params)
                 : QStringLiteral("生成测例失败：请确认已打开工程。"));
    qjcam::VI_UpdateView();
}

void FaceIntersectionDialog::scheduleUpdate()
{
    if (suppressUpdates_) return;
    if (!group_.ready) return;
    if (!realtime_->isChecked()) {
        updatePreview();
        qjcam::VI_UpdateView();
        setStatus(QStringLiteral("实时求交已关闭；点击“求交”刷新交线。"));
        return;
    }
    updateTimer_->start();
}

void FaceIntersectionDialog::run()
{
    if (suppressUpdates_) return;
    std::vector<TopoDS_Face> first;
    std::vector<TopoDS_Face> second;
    if (!readFaces(first, second)) {
        setStatus(QStringLiteral("请为面 A、面 B 至少各选择一个有效面。检查后重新求交。"));
        return;
    }

    try {
        const double tolerance = tolerance_->value();
        const double angle = angle_->value();
        if (!group_.ready || !sameFaceLists(group_.firstFaces, first)
            || !sameFaceLists(group_.secondFaces, second)
            || group_.tolerance != tolerance || group_.angleTolerance != angle) {
            std::string error;
            if (!prepare(first, second, tolerance, angle, group_, error)) {
                clearOutput();
                setStatus(QStringLiteral("输入失败：%1").arg(QString::fromUtf8(error.c_str())));
                return;
            }
        }

        updatePreview();
        const auto result = intersect(group_, moveTarget_->currentIndex() == 1,
                                      translateX_->value(), translateY_->value(), translateZ_->value());
        updateResult(result);

        QString text = QStringLiteral("面 A %1 个 / 面 B %2 个；交线：%3 条边，%4 个孤立交点；"
                                      "成功配对 %5，失败 %6，跳过 %7；总耗时 %8 ms（内核 %9 ms）。")
            .arg(static_cast<int>(group_.firstFaces.size()))
            .arg(static_cast<int>(group_.secondFaces.size()))
            .arg(result.edges)
            .arg(result.vertices)
            .arg(result.succeededPairs)
            .arg(result.failedPairs)
            .arg(result.skippedPairs)
            .arg(result.totalMilliseconds, 0, 'f', 1)
            .arg(result.kernelMilliseconds, 0, 'f', 1);
        if (result.edges == 0 && result.vertices == 0) text += QStringLiteral(" 当前两组面无交线。");
        if (result.failedPairs > 0 && !result.message.empty())
            text += QStringLiteral(" 失败原因：%1").arg(QString::fromUtf8(result.message.c_str()));
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

void FaceIntersectionDialog::updatePreview()
{
    if (!showPreview_->isChecked() || !group_.ready || !hasOffset()) {
        removePreview();
        return;
    }
    const TopoDS_Shape& base = moveTarget_->currentIndex() == 1 ? group_.secondCompound : group_.firstCompound;
    if (base.IsNull()) {
        removePreview();
        return;
    }
    gp_Trsf translation;
    translation.SetTranslation(gp_Vec(translateX_->value(), translateY_->value(), translateZ_->value()));
    const TopoDS_Shape moved = BRepBuilderAPI_Transform(base, translation, Standard_False).Shape();
    if (previewNode_ && !qjcam::MO_SetShape(previewNode_, moved)) previewNode_ = nullptr;
    if (!previewNode_) previewNode_ = qjcam::MO_AddShape("面面求交 / 移动面", moved);
    if (previewNode_) {
        qjcam::MO_SetShapeColor(previewNode_, 0x3399ff);
        qjcam::MO_SetShapeTransparency(previewNode_, 0.4);
    }
}

void FaceIntersectionDialog::updateResult(const FaceIntersectionRun& result)
{
    if (result.shape.IsNull()) {
        removeResult();
        return;
    }
    if (resultNode_ && !qjcam::MO_SetShape(resultNode_, result.shape)) resultNode_ = nullptr;
    if (!resultNode_) resultNode_ = qjcam::MO_AddShape("面面求交 / 交线", result.shape);
    if (resultNode_) {
        qjcam::MO_SetShapeColor(resultNode_, 0xff8800);
        qjcam::MO_SetShapeTransparency(resultNode_, 0.0);
    }
}

void FaceIntersectionDialog::removePreview()
{
    if (!previewNode_) return;
    qjcam::MO_DeleteModel(previewNode_);
    previewNode_ = nullptr;
}

void FaceIntersectionDialog::removeResult()
{
    if (!resultNode_) return;
    qjcam::MO_DeleteModel(resultNode_);
    resultNode_ = nullptr;
}

void FaceIntersectionDialog::clearOutput()
{
    removePreview();
    removeResult();
}

} // namespace sfi
