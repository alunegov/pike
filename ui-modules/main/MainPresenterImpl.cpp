#include <MainPresenterImpl.h>

#include <cassert>
#include <chrono>
#include <future>
#include <thread>

namespace ros { namespace pike { namespace modules {

MainPresenterImpl::~MainPresenterImpl()
{
    if (slice_thread_.joinable()) {
        slice_cancel_token_ = true;
        slice_thread_.join();
    }

    if (depth_thread_.joinable()) {
        depth_cancel_token_ = true;
        depth_thread_.join();
    }

    delete movementAndOrientationReader_;
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
    /*movementAndOrientationReader_->Start([this](int32_t distance, double_t angle) {
        if (view_ != nullptr) {
            view_->SetDistance(distance);
            view_->SetAngle(angle);
        }
    });*/

    depth_cancel_token_ = false;

    depth_thread_ = std::thread{[this]() {
        while (!depth_cancel_token_) {
            if (!depth_idle_token_) {
                const auto depth = pike_->depthometer()->Read();
                if (view_ != nullptr) {
                    view_->SetDepth(depth);
                }
            }

            std::this_thread::sleep_for(std::chrono::milliseconds{1000});
        }
    }};
}

void MainPresenterImpl::StartMoving(ros::devices::MoverDirection dir)
{
    pike_->mover()->SetDirection(dir);

    pike_->mover()->Start();
}

void MainPresenterImpl::StopMoving()
{
    pike_->mover()->Stop();
}

void MainPresenterImpl::StartRotation(ros::devices::RotatorDirection dir)
{
    pike_->rotator()->SetDirection(dir);
    pike_->rotator()->SetSpeed(ros::devices::RotatorSpeed::High);

    pike_->rotator()->Start();
}

void MainPresenterImpl::StopRotation()
{
    pike_->rotator()->Stop();
}

void MainPresenterImpl::SliceClicked()
{
    if (!slice_thread_.joinable()) {
        // TODO: выставить "запись" у кнопки
        
        depth_idle_token_ = true;
        slice_cancel_token_ = false;

        slice_thread_ = std::thread{[this]() {
            slicer_->Read(slice_cancel_token_, [this](double_t angle, int16_t depth) {
                if (view_ != nullptr) {
                    view_->UpdateSliceDepth(angle, depth);
                }
            });
            
            depth_idle_token_ = false;

            if (slice_cancel_token_) {
                return;
            }

            // TODO: убрать "запись" у кнопки
            // TODO: отобразить и сохранить результат
        }};
    } else {
        slice_cancel_token_ = true;
        slice_thread_.join();

        depth_idle_token_ = false;

        // TODO: убрать "запись" у кнопки
    }
}

void MainPresenterImpl::ResetDistance()
{
    pike_->odometer()->Reset();

    if (view_ != nullptr) {
        const auto distance = pike_->odometer()->Get();
        view_->SetDistance(distance);
    }
}

}}}
