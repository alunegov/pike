#pragma once

#include <atomic>
#include <thread>

#include <Pike.h>

#include <MovementAndOrientationReader.h>
#include <Slicer.h>

#include <MainPresenter.h>
#include <MainView.h>

namespace ros { namespace pike { namespace modules {

class MainPresenterImpl : public MainPresenter {
public:
    MainPresenterImpl() = delete;

    MainPresenterImpl(ros::devices::Pike* pike, ros::pike::logic::MovementAndOrientationReader* movementAndOrientationReader,
            ros::pike::logic::Slicer* slicer) :
        pike_{pike},
        movementAndOrientationReader_{movementAndOrientationReader},
        slicer_(slicer)
    {}

    ~MainPresenterImpl() override;

    // MainPresenter
    void SetView(ros::pike::modules::MainView* view) override;

    void OnShow() override;

    void StartMoving(ros::devices::MoverDirection dir) override;

    void StopMoving() override;

    void StartRotation(ros::devices::RotatorDirection dir) override;

    void StopRotation() override;

    void SliceClicked() override;

    void ResetDistance() override;

private:
    ros::pike::modules::MainView* view_{nullptr};

    ros::devices::Pike* pike_{nullptr};

    ros::pike::logic::MovementAndOrientationReader* movementAndOrientationReader_{nullptr};
    ros::pike::logic::Slicer* slicer_{nullptr};

    std::thread slice_thread_;
    std::atomic_bool slice_cancel_token_{false};
};

}}}
