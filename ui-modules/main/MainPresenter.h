#pragma once

#include <string>

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
    
    virtual void ResetDistanceClicked() = 0;

    virtual void StartSlice(std::string dest_path) = 0;

    virtual void StopSlice() = 0;

    virtual void Camera1Clicked() = 0;

    virtual void Camera2Clicked() = 0;

    virtual void StartRec(std::string dest_path) = 0;

    virtual void StopRec() = 0;

    virtual void PhotoClicked(std::string dest_path) = 0;
};

}}}
