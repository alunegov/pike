#include <MainPresenterImpl.h>

#include <cassert>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <thread>

namespace ros { namespace pike { namespace modules {

MainPresenterImpl::MainPresenterImpl(ros::devices::Pike* pike, ros::pike::logic::OngoingReader* ongoingReader,
        ros::pike::logic::Slicer* slicer, ros::pike::logic::SliceMsrMapper* sliceMsrMapper,
        ros::pike::logic::RemoteServer* remote) :
    pike_{pike},
    ongoingReader_{ongoingReader},
    slicer_(slicer),
    sliceMsrMapper_{sliceMsrMapper},
    remote_{remote}
{
    assert(pike != nullptr);
    assert(ongoingReader != nullptr);
    assert(slicer != nullptr);
    assert(sliceMsrMapper != nullptr);
    assert(remote != nullptr);

    ongoingReader_->SetOutput(this);
    remote_->SetOutput(this);
}

MainPresenterImpl::~MainPresenterImpl()
{
    if (ongoingReader_ != nullptr) {
        ongoingReader_->Stop();
    }
    if (remote_ != nullptr) {
        remote_->Stop();
    }

    if (slice_thread_.joinable()) {
        slice_cancel_token_ = true;
        slice_thread_.join();
    }
}

void MainPresenterImpl::SetView(ros::pike::modules::MainView* view)
{
    // TODO: lock view_
    view_ = view;
}

void MainPresenterImpl::OnShow()
{
    ongoingReader_->Start();
    remote_->Start();
}

void MainPresenterImpl::StartMoving(ros::devices::MoverDirection dir)
{
    if (pike_->InMotion()) {
        // TODO: ui notify on reject?
        return;
    }
    assert(!pike_->IsSlicing());

    pike_->mover()->SetDirection(dir);

    pike_->mover()->Start();

    pike_->SetInMotion(true);

    if (view_ != nullptr) {
        // TODO: ui notify on start from remote
        SetMotionEnabled(dir == ros::devices::MoverDirection::Forward, dir == ros::devices::MoverDirection::Backward,
                false, false);
        view_->SetSliceEnabled(false);
    }
}

void MainPresenterImpl::StopMoving()
{
    pike_->mover()->Stop();

    pike_->SetInMotion(false);

    if (view_ != nullptr) {
        // TODO: ui notify on stop from remote
        SetMotionEnabled(true);
        view_->SetSliceEnabled(true);
    }
}

void MainPresenterImpl::StartRotation(ros::devices::RotatorDirection dir)
{
    if (pike_->InMotion()) {
        // TODO: ui notify on reject?
        return;
    }
    assert(!pike_->IsSlicing());

    pike_->rotator()->SetDirection(dir);
    pike_->rotator()->SetSpeed(ros::devices::RotatorSpeed::High);

    pike_->rotator()->Start();

    pike_->SetInMotion(true);

    if (view_ != nullptr) {
        // TODO: ui notify on start from remote
        SetMotionEnabled(false, false, dir == ros::devices::RotatorDirection::CCW,
                dir == ros::devices::RotatorDirection::CW);
        view_->SetSliceEnabled(false);
    }
}

void MainPresenterImpl::StopRotation()
{
    pike_->rotator()->Stop();

    pike_->SetInMotion(false);

    if (view_ != nullptr) {
        // TODO: ui notify on stop from remote
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

void MainPresenterImpl::StartSlice(std::string dest_path)
{
    assert(!pike_->InMotion());
    assert(!pike_->IsSlicing());
    assert(!slice_thread_.joinable());

    if (view_ != nullptr) {
        view_->SetSliceMsr({0}, {1});
        SetMotionEnabled(false);
    }

    slice_dest_path_ = std::move(dest_path);

    ongoingReader_->IdleDepth(true);

    slice_cancel_token_ = false;

    slice_thread_ = std::thread{[this]() {
        auto slice_msr = std::move(slicer_->Read(slice_cancel_token_, this));

        // чтобы не зависеть от возможных будущих изменений slice_cancel_token_
        const bool not_canceled = !slice_cancel_token_;

        if (not_canceled && slice_msr.ok) {
            sliceMsrMapper_->Save(slice_msr);
        }

        //std::this_thread::sleep_for(std::chrono::seconds{4});
        
        ongoingReader_->IdleDepth(false);

        pike_->SetIsSlicing(false);

        // TODO: lock view_
        if (view_ != nullptr) {
            view_->RunOnUiThread([this, not_canceled, slice_msr = std::move(slice_msr)]() {
                if (view_ != nullptr) {
                    // slice_msr - глубокая копия захваченного значения снаружи
                    if (not_canceled && slice_msr.ok) {
                        view_->SetSliceMsr(slice_msr.angles, slice_msr.depths);
                    }
                    view_->SetSliceEnabled(true);
                    view_->SliceCompleted();
                    SetMotionEnabled(true);
                }
            });
        }
    }};

    pike_->SetIsSlicing(true);
}

void MainPresenterImpl::StopSlice()
{
    assert(pike_->IsSlicing());
    assert(slice_thread_.joinable());

    slice_cancel_token_ = true;
    slice_thread_.join();  // gui поток заблокируется на время остановки slice_thread_
    // выполнение всех действий по завершению slice выполняется в потоке (типа reactive)
}

void MainPresenterImpl::Camera1Clicked()
{
    selected_camera_ = 1;
}

void MainPresenterImpl::Camera2Clicked()
{
    selected_camera_ = 2;
}

void MainPresenterImpl::StartRec(std::string dest_path)
{
    rec_dest_path_ = std::move(dest_path);
    rec_start_time_ = std::chrono::system_clock::now();
    rec_start_distance_ = pike_->odometer()->Get();
    rec_start_angle_ = pike_->inclinometer()->Get();

    // TODO: start recording from selected_camera_

    if (view_ != nullptr) {
        view_->SetCamera1Enabled(false);
        view_->SetCamera2Enabled(false);
    }
}

void MainPresenterImpl::StopRec()
{
    const auto start_time_t = std::chrono::system_clock::to_time_t(rec_start_time_);

    const auto stop_time = std::chrono::system_clock::now();
    const auto stop_time_t = std::chrono::system_clock::to_time_t(stop_time);

    std::stringstream ss;
    ss << rec_dest_path_ << (rec_dest_path_.empty() ? "./" : "/")
            << rec_start_distance_ << "-"
            << static_cast<int>(selected_camera_) << "-"
            << std::put_time(localtime(&start_time_t), "%c") << "-"
            << std::put_time(localtime(&stop_time_t), "%c") << "-"
            << rec_start_angle_ << ".video";
    const std::string file_name = ss.str();

    // TODO: stop recording and save recorded as video (encoded? h264/vc-1?)

    if (view_ != nullptr) {
        view_->SetCamera1Enabled(true);
        view_->SetCamera2Enabled(true);
    }
}

void MainPresenterImpl::PhotoClicked(std::string dest_path)
{
    const auto start_time = std::chrono::system_clock::now();
    const auto start_time_t = std::chrono::system_clock::to_time_t(start_time);
    const double_t start_distance = pike_->odometer()->Get();
    const double_t start_angle = pike_->inclinometer()->Get();

    std::stringstream ss;
    ss << dest_path << (dest_path.empty() ? "./" : "/")
            << start_distance << "-"
            << static_cast<int>(selected_camera_) << "-"
            << std::put_time(localtime(&start_time_t), "%c") << "-"
            << start_angle << ".photo";
    const std::string file_name = ss.str();

    // TODO: save buffer from selected_camera_ as image
}

void MainPresenterImpl::RemoteStartMoving(ros::pike::logic::MotionDirection dir)
{
    // TODO: lock view_
    if (view_ != nullptr) {
        view_->RunOnUiThread([this] {
            if (view_ != nullptr) {
                view_->SetMoveForward(true);
                StartMoving(ros::devices::MoverDirection::Forward);
            }
        });
    }
}

void MainPresenterImpl::RemoteStopMoving()
{}

void MainPresenterImpl::RemoteStartRotation(ros::pike::logic::MotionDirection dir)
{}

void MainPresenterImpl::RemoteStopRotation()
{}

void MainPresenterImpl::AdcTick(double_t distance, double_t angle, int16_t depth)
{
    // TODO: lock view_
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
    // TODO: lock view_
    if (view_ != nullptr) {
        view_->RunOnUiThread([this, channels, values, values_count, adc_to_volt]() {
            // TODO: копия values для асинхронного отображения в виде, можно через кольцевой буфер
            if (view_ != nullptr) {
                view_->SetAdcChannels(channels, values, values_count, adc_to_volt);
            }
        });
    }
}

void MainPresenterImpl::TtlInTick(bool ender1, bool ender2)
{
    // TODO: lock view_
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
    // TODO: lock view_
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
