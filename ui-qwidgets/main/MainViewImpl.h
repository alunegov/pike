#pragma once

#include <cmath>
#include <cstdint>
#include <random>

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

    explicit MainViewImpl(ros::pike::modules::MainPresenter* presenter);

    ~MainViewImpl() override;

    // MainView

    void SetDistance(double_t distance) override;

    void SetAngle(double_t angle) override;

    void SetDepth(int16_t depth) override;

    void UpdateSliceDepth(double_t angle, int16_t depth) override;

    void SetEnders(bool ender1, bool ender2) override;

    void SetAdcChannels(const std::vector<uint16_t>& channels, const int16_t* values, size_t values_count,
            double_t adc_to_volt) override;

private:
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
