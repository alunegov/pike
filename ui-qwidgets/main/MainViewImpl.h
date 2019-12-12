#pragma once

#include <cstdint>

#include <QtWidgets/QPushButton>
#include <QtWidgets/QLabel>
#include <QtWidgets/QWidget>

#include <MainPresenter.h>
#include <MainView.h>

namespace ros { namespace pike { namespace ui {

class MainViewImpl :
    public QWidget,
    public ros::pike::modules::MainView
{
    Q_OBJECT

public:
    MainViewImpl() = delete;

    explicit MainViewImpl(ros::pike::modules::MainPresenter* presenter);

    ~MainViewImpl() override;

    // MainView
    void SetDistance(int32_t value) override;

    void SetAngle(double_t value) override;

    void SetDepth(int16_t value) override;

    void UpdateSliceDepth(double_t angle, int16_t depth) override;

private:
    ros::pike::modules::MainPresenter* presenter_{nullptr};

    QLabel* distance_label_{nullptr};
    QLabel* angle_label_{nullptr};
    QLabel* depth_label_{nullptr};
    QPushButton* move_forward_button_{nullptr};
    QPushButton* move_backward_button_{nullptr};
    QPushButton* rotate_ccw_button_{nullptr};
    QPushButton* rotate_cw_button_{nullptr};
    QPushButton* slice_button_{nullptr};
};

}}}
