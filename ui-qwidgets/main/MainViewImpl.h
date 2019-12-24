#pragma once

#include <cassert>
#include <cmath>
#include <cstdint>
#include <functional>
#include <random>

#include <QtCore/QThread>
#include <QtGui/qevent.h>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>

#include <MainPresenter.h>
#include <MainView.h>

#include <CameraWidget.h>
#include <DistanceWidget.h>
#include <InclioWidget.h>
#include <SliceWidget.h>

class MainViewImplTest;

namespace ros { namespace pike { namespace ui {

// Реализация вида главного окна
class MainViewImpl :
    public QWidget,
    public ros::pike::modules::MainView
{
    Q_OBJECT

    friend class ::MainViewImplTest;

public:
    MainViewImpl() = delete;

    MainViewImpl(ros::pike::modules::MainPresenter* presenter, double_t object_length);

    ~MainViewImpl() override;

    // MainView

    void RunOnUiThread(const std::function<void()>& f) override;

    void SetDistance(double_t distance) override;

    void SetAngle(double_t angle) override;

    void SetDepth(int16_t depth) override;

    void UpdateSliceDepth(double_t angle, int16_t depth) override;

    void SetSliceMsr(const std::vector<double_t>& angles, const std::vector<int16_t>& depths) override;

    void SetEnders(bool ender1, bool ender2) override;

    void SetAdcChannels(const std::vector<uint16_t>& channels, const int16_t* values, size_t values_count,
            double_t adc_to_volt) override;

    void SetMoveForwardEnabled(bool enabled) override;

    void SetMoveBackwardEnabled(bool enabled) override;

    void SetRotateCcwEnabled(bool enabled) override;

    void SetRotateCwEnabled(bool enabled) override;

    void SetSliceEnabled(bool enabled) override;

    void SliceCompleted() override;

    void SetCamera1Enabled(bool enabled) override;

    void SetCamera2Enabled(bool enabled) override;

    void SetRecEnabled(bool enabled) override;

    void SetPhotoEnabled(bool enabled) override;

    void SetDestPathEnabled(bool enabled) override;

protected:
    void resizeEvent(QResizeEvent* event) override;

private:
    static void RunOnUiThread(QWidget* target, const std::function<void()>& f)
    {
        assert(target != nullptr);

        if (target->thread() == QThread::currentThread()) {
            f();
        } else {
            QObject dummy;
            QObject::connect(&dummy, &QObject::destroyed, target, f, Qt::BlockingQueuedConnection);
            // Из-за BlockingQueuedConnection функция f будет вызвана в GUI-потоке и мы дождёмся её завершения в этом
            // потоке. Разрушение dummy заблокирует текущий поток до завершения f в GUI-потоке.
        }
    }

    ros::pike::modules::MainPresenter* presenter_{nullptr};

    CameraWidget* camera_viewport_{nullptr};

    DistanceWidget* distance_viewport_{nullptr};

    InclioWidget* inclio_viewport_{nullptr};

    SliceWidget* slice_viewport_{nullptr};
    QLabel* depth_label_{nullptr};
    QLabel* ender1_label_{nullptr};
    QLabel* ender2_label_{nullptr};

    QPushButton* move_forward_button_{nullptr};
    QPushButton* move_backward_button_{nullptr};
    QPushButton* rotate_ccw_button_{nullptr};
    QPushButton* rotate_cw_button_{nullptr};

    QPushButton* slice_button_{nullptr};
    QPushButton* camera1_button_{nullptr};
    QPushButton* camera2_button_{nullptr};
    QPushButton* rec_button_{nullptr};
    QPushButton* photo_button_{nullptr};
    QLineEdit* dest_path_edit_{nullptr};

    std::random_device rd;
    std::mt19937 gen{rd()};
};

}}}
