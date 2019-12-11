#pragma once

#include <MainView.h>

namespace ros { namespace pike { namespace modules {

class MainPresenter {
public:
    virtual void SetView(ros::pike::modules::MainView* view) = 0;

    virtual void OnShow() = 0;
};

}}}
