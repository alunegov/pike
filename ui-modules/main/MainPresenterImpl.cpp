#include <MainPresenterImpl.h>

#include <cassert>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <thread>

namespace ros { namespace pike { namespace modules {

MainPresenterImpl::MainPresenterImpl(ros::pike::logic::Pike* pike, ros::pike::logic::OngoingReader* ongoingReader,
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

    pike_->rotator()->SetOutput(this);
    ongoingReader_->SetOutput(this);
    remote_->SetOutput(this);
}

MainPresenterImpl::~MainPresenterImpl()
{
    if (pike_ != nullptr) {
        if (pike_->IsMoving()) {
            const auto mover_stop_opt = pike_->mover()->Stop();
            if (!mover_stop_opt) {
                // TODO: log
                //"mover stop error: " + mover_stop_opt.error().message()
            }
            // TODO: уместно ли сбрасывать is_moving в случае ошибки mover()->Stop()?
            pike_->SetIsMoving(false);
        }

        if (pike_->IsRotating()) {
            pike_->rotator()->Stop();
            pike_->SetIsRotating(false);
        }

        //pike_->rotator()->SetOutput(nullptr);
    }

    if (ongoingReader_ != nullptr) {
        ongoingReader_->Stop();

        //ongoingReader_->SetOutput(nullptr);
    }
    if (remote_ != nullptr) {
        remote_->Stop();
        
        //remote_->SetOutput(nullptr);
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

void MainPresenterImpl::StartMovement(ros::devices::MoverDirection dir)
{
    if (pike_->IsMoving()) {
        return;
    }
    assert(!pike_->IsRotating() && !pike_->IsSlicing());

    const bool is_fwd = dir == ros::devices::MoverDirection::Forward;
    InternalStartMovement(is_fwd, false);
}

void MainPresenterImpl::StopMovement()
{
    if (!pike_->IsMoving()) {
        return;
    }
    assert(!pike_->IsRotating() && !pike_->IsSlicing());

    InternalStopMovement();
}

void MainPresenterImpl::StartRotation(ros::devices::RotatorDirection dir)
{
    if (pike_->IsRotating()) {
        return;
    }
    assert(!pike_->IsMoving() && !pike_->IsSlicing());

    const bool is_ccw = dir == ros::devices::RotatorDirection::CCW;
    InternalStartRotation(is_ccw, true);
}

void MainPresenterImpl::StopRotation()
{
    if (!pike_->IsRotating()) {
        return;
    }
    assert(!pike_->IsMoving() && !pike_->IsSlicing());

    InternalStopRotation();
}

void MainPresenterImpl::ResetDistanceClicked()
{
    pike_->odometer()->Reset();

    if (view_ != nullptr) {
        const auto distance = pike_->odometer()->Get();
        view_->SetDistance(distance);
    }
}

void MainPresenterImpl::StartSlice(const std::string& dest_path)
{
    assert(!pike_->InMotion());
    assert(!pike_->IsSlicing());
    assert(!slice_thread_.joinable());

    if (view_ != nullptr) {
        view_->SetSliceMsr({0}, {1});
        SetMotionEnabled(false);
    }

    slice_dest_path_ = dest_path;

    ongoingReader_->IdleDepth(true);
    pike_->SetIsSlicing(true);

    slice_cancel_token_ = false;

    // TODO: std::async on threadpool?
    slice_thread_ = std::thread{[this]() {
        auto slice_msr = slicer_->Read(slice_cancel_token_, this);

        // чтобы не зависеть от возможных будущих изменений slice_cancel_token_
        const bool canceled = slice_cancel_token_;

        if (!canceled && slice_msr.ok) {
            sliceMsrMapper_->Save(slice_msr);
        }

        ongoingReader_->IdleDepth(false);
        pike_->SetIsSlicing(false);

        // TODO: lock view_
        if (view_ != nullptr) {
            view_->RunOnUiThread([this, canceled, slice_msr = std::move(slice_msr)]() {
                if (view_ != nullptr) {
                    if (!canceled && slice_msr.ok) {
                        view_->SetSliceMsr(slice_msr.angles, slice_msr.depths);
                    } else {
                        if (!canceled) {
                            view_->SetStatusMsg("slice error: " + slice_msr.ec.message());
                        }
                    }
                    view_->SetSliceEnabled(true);
                    view_->SliceCompleted();
                    SetMotionEnabled(true);
                }
            });
        }
    }};
}

void MainPresenterImpl::StopSlice()
{
    //assert(pike_->IsSlicing());  // при ошибке slicer поток уже подходит к завершению (на строке view_->SliceCompleted())
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

void MainPresenterImpl::StartRec(const std::string& dest_path)
{
    rec_dest_path_ = dest_path;
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

void MainPresenterImpl::PhotoClicked(const std::string& dest_path)
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

void MainPresenterImpl::AdcTick(double_t distance, double_t angle)
{
    // TODO: lock view_
    if (view_ != nullptr) {
        view_->RunOnUiThread([this, distance, angle]() {
            if (view_ != nullptr) {
                view_->SetDistance(distance);
                view_->SetAngle(angle);
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

void MainPresenterImpl::AdcError(const std::error_code& ec)
{
    // TODO: lock view_
    if (view_ != nullptr) {
        view_->RunOnUiThread([this, ec]() {
            if (view_ != nullptr) {
                view_->SetStatusMsg("adc read error: " + ec.message());
                // TODO: поток чтения АЦП завершён - нужно заблокировать весь функционал
            }
        });
    }
}

void MainPresenterImpl::DepthTick(int16_t depth)
{
    // TODO: lock view_
    if (view_ != nullptr) {
        view_->RunOnUiThread([this, depth]() {
            if (view_ != nullptr) {
                view_->SetDepth(depth);
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

void MainPresenterImpl::RemoteStartMovement(ros::pike::logic::MotionDirection dir)
{
    // TODO: lock view_
    if (view_ != nullptr) {
        view_->RunOnUiThread([this, dir] {
            if (pike_->InMotion() || pike_->IsSlicing()) {
                return;
            }

            const bool is_fwd = dir == ros::pike::logic::MotionDirection::Inc;
            InternalStartMovement(is_fwd, true);
        });
    }
}

void MainPresenterImpl::RemoteStopMovement()
{
    // TODO: lock view_
    if (view_ != nullptr) {
        view_->RunOnUiThread([this] {
            if (!pike_->IsMoving() || pike_->IsSlicing()) {
                return;
            }
            assert(!pike_->IsRotating());

            InternalStopMovement();
        });
    }
}

void MainPresenterImpl::RemoteStartRotation(ros::pike::logic::MotionDirection dir)
{
    // TODO: lock view_
    if (view_ != nullptr) {
        view_->RunOnUiThread([this, dir] {
            if (pike_->InMotion() || pike_->IsSlicing()) {
                return;
            }
            
            const bool is_ccw = dir == ros::pike::logic::MotionDirection::Dec;
            InternalStartRotation(is_ccw, true);
        });
    }
}

void MainPresenterImpl::RemoteStopRotation()
{
    // TODO: lock view_
    if (view_ != nullptr) {
        view_->RunOnUiThread([this] {
            if (!pike_->IsRotating() || pike_->IsSlicing()) {
                return;
            }
            assert(!pike_->IsMoving());

            InternalStopRotation();
        });
    }
}

void MainPresenterImpl::RotateError(const std::error_code& ec)
{
    // TODO: lock view_
    if (view_ != nullptr) {
        view_->RunOnUiThread([this, ec] {
            // TODO: ui
            if (view_ != nullptr) {
                view_->SetStatusMsg("rotator step error: " + ec.message());
            }

            // из-за очерёдности обработки UI-сообщений здесь мы можем оказаться уже после StopRotation
            if (!pike_->IsRotating()) {
                return;
            }

            InternalStopRotation();
        });
    }
}

void MainPresenterImpl::InternalStartMovement(bool is_fwd, bool remote)
{
    const auto mover_dir = is_fwd ? ros::devices::MoverDirection::Forward : ros::devices::MoverDirection::Backward;
    pike_->mover()->SetDirection(mover_dir);
    const auto mover_start_opt = pike_->mover()->Start();
    if (!mover_start_opt) {
        // TODO: log, ui?
        if (view_ != nullptr) {
            view_->SetStatusMsg("mover start error: " + mover_start_opt.error().message());
        }
        return;
    }

    pike_->SetIsMoving(true);

    if (view_ != nullptr) {
        if (remote) {
            if (is_fwd) {
                view_->SetMoveForward(true);
            } else {
                view_->SetMoveBackward(true);
            }
        }
        SetMotionEnabled(is_fwd, !is_fwd, false, false);
        view_->SetSliceEnabled(false);
    }
}

void MainPresenterImpl::InternalStopMovement()
{
    const auto mover_stop_opt = pike_->mover()->Stop();
    if (!mover_stop_opt) {
        // TODO: log, ui?
        if (view_ != nullptr) {
            view_->SetStatusMsg("mover stop error: " + mover_stop_opt.error().message());
        }
        return;
    }

    pike_->SetIsMoving(false);

    if (view_ != nullptr) {
        // сбрасываем возможное состояние кнопки после remote
        view_->SetMoveForward(false);
        view_->SetMoveBackward(false);
        SetMotionEnabled(true);
        view_->SetSliceEnabled(true);
    }
}

void MainPresenterImpl::InternalStartRotation(bool is_ccw, bool remote)
{
    const auto rotator_dir = is_ccw ? ros::devices::RotatorDirection::CCW : ros::devices::RotatorDirection::CW;
    pike_->rotator()->SetDirection(rotator_dir);
    pike_->rotator()->SetSpeed(ros::devices::RotatorSpeed::High);
    const auto rotator_start_opt = pike_->rotator()->Start();
    if (!rotator_start_opt) {
        // TODO: log, ui?
        if (view_ != nullptr) {
            view_->SetStatusMsg("rotator start error: " + rotator_start_opt.error().message());
        }
        return;
    }
    // после старта вращения могут прийти сообщения через RotateError

    pike_->SetIsRotating(true);

    if (view_ != nullptr) {
        if (remote) {
            if (is_ccw) {
                view_->SetRotateCcw(true);
            } else {
                view_->SetRotateCw(true);
            }
        }
        SetMotionEnabled(false, false, is_ccw, !is_ccw);
        view_->SetSliceEnabled(false);
    }
}

void MainPresenterImpl::InternalStopRotation()
{
    pike_->rotator()->Stop();

    pike_->SetIsRotating(false);

    if (view_ != nullptr) {
        // сбрасываем возможное состояние кнопки после remote
        view_->SetRotateCcw(false);
        view_->SetRotateCw(false);
        SetMotionEnabled(true);
        view_->SetSliceEnabled(true);
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
