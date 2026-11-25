#include "intersection_dialog.h"
#include "intersection_algorithm.h"
#include "validation.h"

#include <qjcam_api.h>
#include <selector/model_selector.h>
#include <Standard_Failure.hxx>
#include <TopoDS.hxx>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLineEdit>
#include <QFileDialog>
#include <QDir>
#include <QMessageBox>
#include <QProgressDialog>
#include <QPointer>
#include <QDateTime>
#include <exception>

namespace ffi {
namespace {
QString publish(const ValidationReport& report, const QString& label)
{
    QStringList failures;
    auto add = [&](const AlgorithmRun& run, const ShapeMetrics& metrics, const QString& suffix, unsigned int color) {
        if (run.result.status != IntersectionStatus::Success || (metrics.edges == 0 && metrics.vertices == 0)) return;
        const auto name = (label + suffix).toUtf8();
        const auto handle = qjcam::MO_AddShape(name.constData(), combineResults(run.result));
        if (handle) qjcam::MO_SetShapeColor(handle, color);
        else failures << QStringLiteral("%1 模型树输出失败（请确认已打开工程）").arg(suffix);
    };
    add(report.occ, report.occMetrics, QStringLiteral(" / OCC"), 0x00cc00);
    add(report.custom, report.customMetrics, QStringLiteral(" / SMLib"), 0xff8800);
    return failures.join('\n');
}

void showReport(QWidget* parent, const QString& summary, const QString& details)
{
    QMessageBox box(QMessageBox::Information, QStringLiteral("面面求交验证报告"), summary, QMessageBox::Ok, parent);
    box.setDetailedText(details);
    box.exec();
}
}

IntersectionDialog::IntersectionDialog(QWidget* parent) : QDialog(parent)
{
    setWindowTitle(QStringLiteral("面面求交验证"));
    setAttribute(Qt::WA_DeleteOnClose);
    setModal(false); // The model view must remain available for picking.
    resize(520, 260);

    auto* layout = new QVBoxLayout(this);
    auto* hint = new QLabel(QStringLiteral("可选择面 A/B 单次验证，或批量运行根目录下的测例。每个测例目录恰含两个 BREP 文件，每个文件恰含一个面。对比 OCC 与 SMLib 面面求交。"), this);
    hint->setWordWrap(true);
    layout->addWidget(hint);
    auto* pathRow = new QHBoxLayout;
    caseRoot_ = new QLineEdit(this);
    caseRoot_->setPlaceholderText(QStringLiteral("测例根目录（直接子目录为各测例）"));
    auto* browse = new QPushButton(QStringLiteral("浏览…"), this);
    auto* batch = new QPushButton(QStringLiteral("运行全部测例"), this);
    pathRow->addWidget(caseRoot_);
    pathRow->addWidget(browse);
    pathRow->addWidget(batch);
    layout->addLayout(pathRow);
    connect(browse, &QPushButton::clicked, this, [this] {
        const auto directory = QFileDialog::getExistingDirectory(this, QStringLiteral("选择测例根目录"), caseRoot_->text());
        if (!directory.isEmpty()) caseRoot_->setText(directory);
    });
    connect(batch, &QPushButton::clicked, this, &IntersectionDialog::runBatch);
    auto* form = new QFormLayout;
    using Pickup = qjcam::ModelSelector::PickupType;
    first_ = new qjcam::ModelSelector(this, Pickup::PT_PickupFace, 1);
    second_ = new qjcam::ModelSelector(this, Pickup::PT_PickupFace, 1);
    tolerance_ = new QDoubleSpinBox(this);
    tolerance_->setDecimals(9);
    tolerance_->setRange(1.0e-9, 1.0);
    tolerance_->setSingleStep(1.0e-6);
    tolerance_->setValue(1.0e-6);
    form->addRow(QStringLiteral("面 A"), first_);
    form->addRow(QStringLiteral("面 B"), second_);
    form->addRow(QStringLiteral("容差（模型长度单位）"), tolerance_);
    layout->addLayout(form);
    result_ = new QLabel(this);
    result_->setWordWrap(true);
    result_->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(result_);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
    auto* runButton = buttons->addButton(QStringLiteral("验证所选面"), QDialogButtonBox::ActionRole);
    auto* resetButton = buttons->addButton(QStringLiteral("重置"), QDialogButtonBox::ResetRole);
    layout->addWidget(buttons);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(runButton, &QPushButton::clicked, this, &IntersectionDialog::run);
    connect(resetButton, &QPushButton::clicked, this, &IntersectionDialog::reset);
    connect(first_, &qjcam::ModelSelector::ShapeChanged, this, &IntersectionDialog::invalidateResult);
    connect(second_, &qjcam::ModelSelector::ShapeChanged, this, &IntersectionDialog::invalidateResult);
    connect(tolerance_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &IntersectionDialog::invalidateResult);
    invalidateResult();
}

void IntersectionDialog::invalidateResult()
{
    result_->setText(QStringLiteral("等待验证。"));
}

void IntersectionDialog::reset()
{
    first_->Clear();
    second_->Clear();
    tolerance_->setValue(1.0e-6);
    invalidateResult();
}

void IntersectionDialog::run()
{
    try {
        const auto first = first_->GetSelectedModels();
        const auto second = second_->GetSelectedModels();
        if (first.size() != 1 || second.size() != 1
            || first[0].IsNull() || second[0].IsNull()
            || first[0].ShapeType() != TopAbs_FACE || second[0].ShapeType() != TopAbs_FACE) {
            result_->setText(QStringLiteral("请为面 A、面 B 各选择一个有效面。"));
            return;
        }
        const auto report = validate({TopoDS::Face(first[0]), TopoDS::Face(second[0]), tolerance_->value()});
        const auto output = publish(report, QStringLiteral("所选面 %1").arg(QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss-zzz")));
        const auto text = report.text + '\n' + output;
        result_->setText(text);
        qjcam::QJ_OutputMessage(text.toUtf8().constData());
        qjcam::VI_UpdateView();
        showReport(this, report.text, text);
    } catch (const Standard_Failure& error) {
        result_->setText(QStringLiteral("几何输入异常：%1").arg(QString::fromUtf8(error.GetMessageString())));
    } catch (const std::exception& error) {
        result_->setText(QStringLiteral("验证失败：%1").arg(QString::fromUtf8(error.what())));
    } catch (...) {
        result_->setText(QStringLiteral("验证失败：未知异常。"));
    }
}

void IntersectionDialog::runBatch()
{
    const QDir root(caseRoot_->text().trimmed());
    if (caseRoot_->text().trimmed().isEmpty() || !root.exists()) {
        QMessageBox::warning(this, QStringLiteral("测例目录"), QStringLiteral("请输入有效的测例根目录。"));
        return;
    }
    const auto directories = root.entryInfoList(QDir::Dirs | QDir::NoDotAndDotDot, QDir::Name);
    if (directories.isEmpty()) {
        QMessageBox::warning(this, QStringLiteral("测例目录"), QStringLiteral("根目录中没有测例子目录。"));
        return;
    }
    const double tolerance = tolerance_->value();
    const auto batchId = QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss-zzz");
    QPointer<IntersectionDialog> guard(this);
    // No parent: a project-closing notifier may destroy this dialog while
    // setValue processes Qt events. Keep the progress object's lifetime local.
    QProgressDialog progress(QStringLiteral("读取并验证测例…"), QStringLiteral("取消后续测例"), 0, directories.size(), nullptr);
    progress.setWindowModality(Qt::ApplicationModal);
    progress.setMinimumDuration(0);
    progress.setAutoClose(false);
    progress.show();
    QStringList details;
    int processed = 0, failed = 0, pending = 0;
    for (const auto& directory : directories) {
        progress.setValue(processed);
        if (!guard) return;
        if (progress.wasCanceled()) break;
        progress.setLabelText(directory.fileName());
        QString text = QStringLiteral("[%1]\n").arg(directory.fileName());
        try {
            const auto request = readCase(directory.absoluteFilePath(), tolerance);
            const auto report = validate(request);
            text += report.text;
            bool caseFailed = report.occ.result.status != IntersectionStatus::Success
                || (report.custom.result.status != IntersectionStatus::Success && report.custom.result.status != IntersectionStatus::NotImplemented);
            if (report.custom.result.status == IntersectionStatus::NotImplemented) ++pending;
            const auto output = publish(report, batchId + " / " + directory.fileName());
            if (!output.isEmpty()) { text += '\n' + output; caseFailed = true; }
            if (caseFailed) ++failed;
        } catch (const Standard_Failure& e) {
            ++failed;
            text += QStringLiteral("测例失败：%1").arg(QString::fromUtf8(e.GetMessageString()));
        } catch (const std::exception& e) {
            ++failed;
            text += QStringLiteral("测例失败：%1").arg(QString::fromUtf8(e.what()));
        } catch (...) { ++failed; text += QStringLiteral("测例失败：未知异常"); }
        ++processed;
        details << text;
        qjcam::QJ_OutputMessage(text.toUtf8().constData());
    }
    progress.hide();
    if (!guard) return;
    qjcam::VI_UpdateView();
    const auto summary = QStringLiteral("已处理 %1/%2；失败 %3；算法未实现 %4。%5\n绿色：OCC；橙色：SMLib。展开详细信息查看质量和效率数据。")
        .arg(processed).arg(directories.size()).arg(failed).arg(pending)
        .arg(processed < directories.size() ? QStringLiteral("已取消后续测例。") : QString());
    result_->setText(summary);
    showReport(this, summary, details.join(QStringLiteral("\n\n")));
}

} // namespace ffi
