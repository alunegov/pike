#pragma once

#include <Mover.h>
#include <Rotator.h>

#include <MainView.h>

namespace ros { namespace pike { namespace modules {

// Презентер главного окна
class MainPresenter
{
public:
    virtual ~MainPresenter() = default;

    virtual void SetView(ros::pike::modules::MainView* view) = 0;

    virtual void OnShow() = 0;

    virtual void StartMoving(ros::devices::MoverDirection dir) = 0;

    virtual void StopMoving() = 0;

    virtual void StartRotation(ros::devices::RotatorDirection dir) = 0;

    virtual void StopRotation() = 0;

    virtual void SliceClicked() = 0;

    virtual void ResetDistance() = 0;
};

}}}
