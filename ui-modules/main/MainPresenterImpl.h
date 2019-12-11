#pragma once

#include <CD22.h>
#include <Pike.h>

#include <MainPresenter.h>
#include <MainView.h>

namespace ros { namespace pike { namespace modules {

class MainPresenterImpl : public MainPresenter {
public:
    MainPresenterImpl() = delete;

    explicit MainPresenterImpl(ros::devices::Pike* pike) :
        view_{nullptr},
        pike_{pike}
    {}

    ~MainPresenterImpl();

    // MainPresenter
    void SetView(ros::pike::modules::MainView* view) override;

    void OnShow() override;

private:
    ros::pike::modules::MainView* view_{nullptr};

    ros::devices::Pike* pike_{nullptr};
};

}}}
