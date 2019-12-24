#pragma once

#include <atomic>
#include <chrono>
#include <thread>

#include <Pike.h>

#include <OngoingReader.h>
#include <SliceMsrMapper.h>
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
            ros::pike::logic::Slicer* slicer, ros::pike::logic::SliceMsrMapper* sliceMsrMapper);

    ~MainPresenterImpl() override;

    // MainPresenter

    void SetView(ros::pike::modules::MainView* view) override;

    void OnShow() override;

    void StartMoving(ros::devices::MoverDirection dir) override;

    void StopMoving() override;

    void StartRotation(ros::devices::RotatorDirection dir) override;

    void StopRotation() override;

    void ResetDistanceClicked() override;

    void StartSlice(std::string dest_path) override;

    void StopSlice() override;

    void Camera1Clicked() override;

    void Camera2Clicked() override;

    void StartRec(std::string dest_path) override;

    void StopRec() override;

    void PhotoClicked(std::string dest_path) override;

    // OngoingReaderOutput

    void AdcTick(double_t distance, double_t angle, int16_t depth) override;

    void AdcTick_Values(const std::vector<uint16_t>& channels, const int16_t* values, size_t values_count,
            double_t adc_to_volt) override;

    void TtlInTick(bool ender1, bool ender2) override;

    // SlicerReadOutput

    void SliceTick(double_t angle, int16_t depth) override;

    //void TtlInTick(bool ender1, bool ender2) override;

private:
    void SetMotionEnabled(bool enabled);

    void SetMotionEnabled(bool forward_enabled, bool backward_enabled, bool ccw_enabled, bool cw_enabled);

    ros::pike::modules::MainView* view_{nullptr};

    ros::devices::Pike* pike_{nullptr};

    ros::pike::logic::OngoingReader* ongoingReader_{nullptr};
    ros::pike::logic::Slicer* slicer_{nullptr};
    ros::pike::logic::SliceMsrMapper* sliceMsrMapper_{nullptr};

    std::thread slice_thread_;
    std::atomic_bool slice_cancel_token_{false};

    uint8_t selected_camera_{1};

    std::string slice_dest_path_;

    std::string rec_dest_path_;
    std::chrono::system_clock::time_point rec_start_time_;
    double_t rec_start_distance_{0};
    double_t rec_start_angle_{0};
};

}}}
