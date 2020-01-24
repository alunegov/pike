#pragma once

#include <atomic>
#include <chrono>
#include <system_error>
#include <thread>

#include <OngoingReader.h>
#include <Pike.h>
#include <RemoteServer.h>
#include <SliceMsrMapper.h>
#include <Slicer.h>

#include <MainPresenter.h>
#include <MainView.h>

namespace ros { namespace pike { namespace modules {

// Реализация презентера главного окна
class MainPresenterImpl :
    public MainPresenter,
    public ros::pike::logic::OngoingReaderOutput,
    public ros::pike::logic::SlicerReadOutput,
    public ros::pike::logic::RemoteServerOutput,
    public ros::devices::RotatorOutput
{
public:
    MainPresenterImpl() = delete;

    MainPresenterImpl(ros::pike::logic::Pike* pike, ros::pike::logic::OngoingReader* ongoingReader,
            ros::pike::logic::Slicer* slicer, ros::pike::logic::SliceMsrMapper* sliceMsrMapper,
            ros::pike::logic::RemoteServer* remote);

    ~MainPresenterImpl() override;

    // MainPresenter

    void SetView(ros::pike::modules::MainView* view) override;

    void OnShow() override;

    void StartMovement(ros::devices::MoverDirection dir) override;

    void StopMovement() override;

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

    void AdcTick(double_t distance, double_t angle) override;

    void AdcTick_Values(const std::vector<uint16_t>& channels, const int16_t* values, size_t values_count,
            double_t adc_to_volt) override;

    void AdcFinish(bool canceled) override;

    void DepthTick(int16_t depth) override;

    // also for SlicerReadOutput
    void TtlInTick(bool ender1, bool ender2) override;

    // SlicerReadOutput

    void SliceTick(double_t angle, int16_t depth) override;

    //void TtlInTick(bool ender1, bool ender2) override;

    // RemoteOutput

    void RemoteStartMovement(ros::pike::logic::MotionDirection dir) override;

    void RemoteStopMovement() override;

    void RemoteStartRotation(ros::pike::logic::MotionDirection dir) override;

    void RemoteStopRotation() override;

    // RotatorOutput

    void RotateError(const std::error_code& ec) override;

private:
    void SetMotionEnabled(bool enabled);

    void SetMotionEnabled(bool forward_enabled, bool backward_enabled, bool ccw_enabled, bool cw_enabled);

    ros::pike::modules::MainView* view_{nullptr};

    ros::pike::logic::Pike* pike_{nullptr};
    ros::pike::logic::OngoingReader* ongoingReader_{nullptr};
    ros::pike::logic::Slicer* slicer_{nullptr};
    ros::pike::logic::SliceMsrMapper* sliceMsrMapper_{nullptr};
    ros::pike::logic::RemoteServer* remote_{nullptr};

    // Поток регистрации slice
    std::thread slice_thread_;
    // Токен останова потока slice
    std::atomic_bool slice_cancel_token_{false};

    // Номер выбранной камеры
    uint8_t selected_camera_{1};

    // Путь сохранения slice (сохраняется на момент начала slice)
    std::string slice_dest_path_;

    // Путь сохранения видео (сохраняется на момент начала записи)
    std::string rec_dest_path_;
    // Время начала записи видео
    std::chrono::system_clock::time_point rec_start_time_;
    // Пройденная дистанция на момент начала записи видео, мм
    double_t rec_start_distance_{0};
    // Положение на момент начала записи видео, °
    double_t rec_start_angle_{0};
};

}}}
