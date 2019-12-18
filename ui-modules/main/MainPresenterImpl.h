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
class MainPresenterImpl :
    public MainPresenter,
    public ros::pike::logic::OngoingReaderOutput,
    public ros::pike::logic::SlicerReadOutput
{
public:
    MainPresenterImpl() = delete;

    MainPresenterImpl(ros::devices::Pike* pike, ros::pike::logic::OngoingReader* ongoingReader,
            ros::pike::logic::Slicer* slicer);

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

    // OngoingReaderOutput

    void AdcTick(double_t distance, double_t angle, int16_t depth) override;

    void AdcTick_Values(const std::vector<uint16_t>& channels, const int16_t* values, size_t values_count,
            double_t adc_to_volt) override;

    void TtlInTick(bool ender1, bool ender2) override;

    // SlicerReadOutput

    void SliceTick(double_t angle, int16_t depth) override;

    //void TtlInTick(bool ender1, bool ender2) override;

private:
    ros::pike::modules::MainView* view_{nullptr};

    ros::devices::Pike* pike_{nullptr};

    ros::pike::logic::OngoingReader* ongoingReader_{nullptr};
    ros::pike::logic::Slicer* slicer_{nullptr};

    std::thread slice_thread_;
    std::atomic_bool slice_cancel_token_{false};
};

}}}
