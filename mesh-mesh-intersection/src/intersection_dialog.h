#pragma once

#include "mesh_pair.h"

#include <qjcam_api.h>
#include <QDialog>

namespace qjcam { class ModelSelector; }
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QTimer;

namespace mmi {

class IntersectionDialog final : public QDialog
{
public:
    explicit IntersectionDialog(QWidget* parent);
    ~IntersectionDialog() override;

    // 工程即将关闭时，宿主会先释放模型树中的形状，此时不能删除结果节点。
    static void setProjectClosing(bool closing);

private:
    void run();
    void scheduleUpdate();
    void invalidate();
    void reset();
    void updatePreview();
    void updateResult(const mesh_inter::MeshIntersectResult& result);
    void removePreview();
    void removeResult();
    void clearOutput();
    void setStatus(const QString& text);
    bool readFaces(std::vector<TopoDS_Face>& first, std::vector<TopoDS_Face>& second) const;
    bool hasOffset() const;

    static bool projectClosing_;

    qjcam::ModelSelector* first_ = nullptr;
    qjcam::ModelSelector* second_ = nullptr;
    QDoubleSpinBox* linear_ = nullptr;
    QDoubleSpinBox* angular_ = nullptr;
    QDoubleSpinBox* tolerance_ = nullptr;
    QComboBox* moveTarget_ = nullptr;
    QDoubleSpinBox* translateX_ = nullptr;
    QDoubleSpinBox* translateY_ = nullptr;
    QDoubleSpinBox* translateZ_ = nullptr;
    QCheckBox* realtime_ = nullptr;
    QCheckBox* showPreview_ = nullptr;
    QLabel* status_ = nullptr;
    QTimer* updateTimer_ = nullptr;

    FacePairMeshes meshes_;
    qjcam::ModelHandle previewNode_ = nullptr;
    qjcam::ModelHandle resultNode_ = nullptr;
    bool suppressUpdates_ = false;
};

} // namespace mmi
