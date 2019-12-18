#include <MainPresenterImpl.h>

#include <cassert>
#include <chrono>
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
        
        ongoingReader_->IdleDepth(true);

        slice_cancel_token_ = false;

        slice_thread_ = std::thread{[this]() {
            slicer_->Read(slice_cancel_token_, this);
            
            ongoingReader_->IdleDepth(false);

            if (slice_cancel_token_) {
                return;
            }

            // TODO: убрать "запись" у кнопки
            // TODO: отобразить и сохранить результат
        }};
    } else {
        slice_cancel_token_ = true;
        slice_thread_.join();

        ongoingReader_->IdleDepth(false);

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

void MainPresenterImpl::AdcTick(double_t distance, double_t angle, int16_t depth)
{
    if (view_ != nullptr) {
        view_->SetDistance(distance);
        view_->SetAngle(angle);

        // не идёт оцифровка сечения?
        if (depth != INT16_MIN) {
            view_->SetDepth(depth);
        }
    }
}

void MainPresenterImpl::AdcTick_Values(const std::vector<uint16_t>& channels, const int16_t* values,
        size_t values_count, double_t adc_to_volt)
{
    if (view_ != nullptr) {
        view_->SetAdcChannels(channels, values, values_count, adc_to_volt);
    }
}

void MainPresenterImpl::TtlInTick(bool ender1, bool ender2)
{
    if (view_ != nullptr) {
        view_->SetEnders(ender1, ender2);
    }
}

void MainPresenterImpl::SliceTick(double_t angle, int16_t depth)
{
    if (view_ != nullptr) {
        view_->UpdateSliceDepth(angle, depth);
    }
}

}}}
