#pragma once

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

    // MainView
    void SetDepth(int16_t value) override;

private:
    ros::pike::modules::MainPresenter* presenter_{nullptr};

    QLabel* label_{nullptr};
};

}}}
