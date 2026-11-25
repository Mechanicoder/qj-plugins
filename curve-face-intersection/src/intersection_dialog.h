#pragma once

#include <QDialog>

namespace qjcam { class ModelSelector; }
class QDoubleSpinBox;
class QLabel;
class QLineEdit;

namespace cfi {

class IntersectionDialog final : public QDialog {
public:
    explicit IntersectionDialog(QWidget* parent);

private:
    void run();
    void runBatch();
    void reset();
    void invalidateResult();
    qjcam::ModelSelector* first_;
    qjcam::ModelSelector* second_;
    QDoubleSpinBox* tolerance_;
    QLabel* result_;
    QLineEdit* caseRoot_;
};

} // namespace cfi

