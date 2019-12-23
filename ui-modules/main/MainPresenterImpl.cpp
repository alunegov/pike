#include <MainPresenterImpl.h>

#include <cassert>
#include <chrono>
#include <sstream>
#include <thread>

namespace ros { namespace pike { namespace modules {

MainPresenterImpl::MainPresenterImpl(ros::devices::Pike* pike, ros::pike::logic::OngoingReader* ongoingReader,
        ros::pike::logic::Slicer* slicer) :
    pike_{pike},
    ongoingReader_{ongoingReader},
    slicer_(slicer)
{
    assert(ongoingReader != nullptr);
    ongoingReader_->SetOutput(this);
}

MainPresenterImpl::~MainPresenterImpl()
{
    if (slice_thread_.joinable()) {
        slice_cancel_token_ = true;
        // TODO: при вызове из ui-потока возможен deadlock?
        slice_thread_.join();
    }

    delete ongoingReader_;
    delete slicer_;

    // удаляемые выше зависят от pike_
    delete pike_;
}

void MainPresenterImpl::SetView(ros::pike::modules::MainView* view)
{
    view_ = view;
}

void MainPresenterImpl::OnShow()
{
    ongoingReader_->Start();
}

void MainPresenterImpl::StartMoving(ros::devices::MoverDirection dir)
{
    pike_->mover()->SetDirection(dir);

    pike_->mover()->Start();

    if (view_ != nullptr) {
        SetMotionEnabled(dir == ros::devices::MoverDirection::Forward, dir == ros::devices::MoverDirection::Backward,
                false, false);
        view_->SetSliceEnabled(false);
    }
}

void MainPresenterImpl::StopMoving()
{
    pike_->mover()->Stop();

    if (view_ != nullptr) {
        SetMotionEnabled(true);
        view_->SetSliceEnabled(true);
    }
}

void MainPresenterImpl::StartRotation(ros::devices::RotatorDirection dir)
{
    pike_->rotator()->SetDirection(dir);
    pike_->rotator()->SetSpeed(ros::devices::RotatorSpeed::High);

    pike_->rotator()->Start();

    if (view_ != nullptr) {
        SetMotionEnabled(false, false, dir == ros::devices::RotatorDirection::CCW,
                dir == ros::devices::RotatorDirection::CW);
        view_->SetSliceEnabled(false);
    }
}

void MainPresenterImpl::StopRotation()
{
    pike_->rotator()->Stop();

    if (view_ != nullptr) {
        SetMotionEnabled(true);
        view_->SetSliceEnabled(true);
    }
}

void MainPresenterImpl::ResetDistanceClicked()
{
    pike_->odometer()->Reset();

    if (view_ != nullptr) {
        const auto distance = pike_->odometer()->Get();
        view_->SetDistance(distance);
    }
}

void MainPresenterImpl::StartSlice()
{
    if (slice_thread_.joinable()) {
        slice_cancel_token_ = true;
        slice_thread_.join();
    }

    if (view_ != nullptr) {
        SetMotionEnabled(false);
    }

    ongoingReader_->IdleDepth(true);

    slice_cancel_token_ = false;

    slice_thread_ = std::thread{[this]() {
        // TODO: consume slice msr result from Read
        slicer_->Read(slice_cancel_token_, this);

        //std::this_thread::sleep_for(std::chrono::seconds{4});
        
        ongoingReader_->IdleDepth(false);

        if (view_ != nullptr) {
            view_->RunOnUiThread([this]() {
                if (view_ != nullptr) {
                    view_->SetSliceEnabled(true);
                    view_->SliceCompleted();
                    SetMotionEnabled(true);
                }
            });
        }

        if (slice_cancel_token_) {
            return;
        }
        
        // TODO: отобразить и сохранить результат
        if (view_ != nullptr) {
            view_->RunOnUiThread([this]() {
                if (view_ != nullptr) {
                    SetMotionEnabled(false);
                }
            });
        }
    }};
}

void MainPresenterImpl::StopSlice()
{
    assert(slice_thread_.joinable());

    slice_cancel_token_ = true;
    //slice_thread_.join();

    // выполнение всех остальных действий выполняется в потоке (типа reactive), на это время блокируем кнопку
    view_->SetSliceEnabled(false);
}

void MainPresenterImpl::Camera1Clicked()
{
    selected_camera_ = 1;
}

void MainPresenterImpl::Camera2Clicked()
{
    selected_camera_ = 2;
}

void MainPresenterImpl::StartRec()
{
    rec_start_time_ = std::chrono::steady_clock::now();
    rec_start_distance_ = pike_->odometer()->Get();
    rec_start_angle_ = pike_->inclinometer()->Get();

    view_->SetCamera1Enabled(false);
    view_->SetCamera2Enabled(false);
}

void MainPresenterImpl::StopRec()
{
    const auto stop_time = std::chrono::steady_clock::now();

    std::stringstream ss;
    ss << rec_start_distance_ << "-" << selected_camera_ << "-" << "rec_start_time_" << "-" << "stop_time" << "-" << rec_start_angle_;
    const std::string file_name = ss.str();

    view_->SetCamera1Enabled(true);
    view_->SetCamera2Enabled(true);
}

void MainPresenterImpl::PhotoClicked()
{
    if (view_ != nullptr) {
        const std::string dest_path = std::move(view_->GetDestPath());
    }

    const auto start_time = std::chrono::steady_clock::now();
    const double_t start_distance = pike_->odometer()->Get();
    const double_t start_angle = pike_->inclinometer()->Get();

    std::stringstream ss;
    ss << start_distance << "-" << selected_camera_ << "-" << "start_time" << "-" << start_angle;
    const std::string file_name = ss.str();
}

void MainPresenterImpl::AdcTick(double_t distance, double_t angle, int16_t depth)
{
    if (view_ != nullptr) {
        view_->RunOnUiThread([this, distance, angle, depth]() {
            if (view_ != nullptr) {
                view_->SetDistance(distance);
                view_->SetAngle(angle);

                // TODO: при оцифровке сечения здесь будет INT16_MIN
                view_->SetDepth(depth);
            }
        });
    }
}

void MainPresenterImpl::AdcTick_Values(const std::vector<uint16_t>& channels, const int16_t* values,
        size_t values_count, double_t adc_to_volt)
{
    if (view_ != nullptr) {
        view_->RunOnUiThread([this, channels, values, values_count, adc_to_volt]() {
            if (view_ != nullptr) {
                view_->SetAdcChannels(channels, values, values_count, adc_to_volt);
            }
        });
    }
}

void MainPresenterImpl::TtlInTick(bool ender1, bool ender2)
{
    if (view_ != nullptr) {
        view_->RunOnUiThread([this, ender1, ender2]() {
            if (view_ != nullptr) {
                view_->SetEnders(ender1, ender2);
            }
        });
    }
}

void MainPresenterImpl::SliceTick(double_t angle, int16_t depth)
{
    if (view_ != nullptr) {
        view_->RunOnUiThread([this, angle, depth]() {
            if (view_ != nullptr) {
                view_->UpdateSliceDepth(angle, depth);
            }
        });
    }
}

void MainPresenterImpl::SetMotionEnabled(bool enabled)
{
    SetMotionEnabled(enabled, enabled, enabled, enabled);
}

void MainPresenterImpl::SetMotionEnabled(bool forward_enabled, bool backward_enabled, bool ccw_enabled, bool cw_enabled)
{
    assert(view_ != nullptr);

    view_->SetMoveForwardEnabled(forward_enabled);
    view_->SetMoveBackwardEnabled(backward_enabled);
    view_->SetRotateCcwEnabled(ccw_enabled);
    view_->SetRotateCwEnabled(cw_enabled);
}

}}}
