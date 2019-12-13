#pragma once

#include <atomic>
#include <thread>

#include <Pike.h>

#include <OngoingReader.h>
#include <Slicer.h>

#include <MainPresenter.h>
#include <MainView.h>

namespace ros { namespace pike { namespace modules {

// Реализация презентера главного окна
class MainPresenterImpl : public MainPresenter {
public:
    MainPresenterImpl() = delete;

    MainPresenterImpl(ros::devices::Pike* pike, ros::pike::logic::OngoingReader* ongoingReader,
            ros::pike::logic::Slicer* slicer) :
        pike_{pike},
        ongoingReader_{ongoingReader},
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

    ros::pike::logic::OngoingReader* ongoingReader_{nullptr};
    ros::pike::logic::Slicer* slicer_{nullptr};

    std::thread slice_thread_;
    std::atomic_bool slice_cancel_token_{false};
};

}}}
