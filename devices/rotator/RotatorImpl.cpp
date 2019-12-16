#include <RotatorImpl.h>

#include <cassert>
#include <thread>

namespace ros { namespace devices {

RotatorImpl::~RotatorImpl()
{
    if (rotate_thread_.joinable()) {
        rotate_cancel_token_ = true;
        rotate_thread_.join();
    }
}

void RotatorImpl::SetDirection(RotatorDirection direction)
{
    direction_ = direction;
}

void RotatorImpl::SetSpeed(RotatorSpeed speed)
{
    speed_ = speed;
}

void RotatorImpl::Start()
{
    assert(!rotate_thread_.joinable());

    Enable();

    applyDirection();

    applySpeed();

    // запуск потока, генерирующего step
    rotate_cancel_token_ = false;

    rotate_thread_ = std::thread{[this]() {
        while (!rotate_cancel_token_) {
            Step();
        }

        // оставляем enable
    }};
}

void RotatorImpl::Stop()
{
    assert(rotate_thread_.joinable());

    // останов (и завершение) потока, генерирующего step
    rotate_cancel_token_ = true;
    rotate_thread_.join();
}

void RotatorImpl::Rotate(size_t steps_count)
{
    assert(!rotate_thread_.joinable());

    Enable();

    applyDirection();

    applySpeed();

    while (steps_count > 0) {
        Step();

        steps_count--;
    }

    // оставляем enable
}

void RotatorImpl::Enable()
{
    daq_->TtlOut_ClrPin(enable_pin_);
    // delay MIN 650 nanosec to STEP
}

void RotatorImpl::Disable()
{
    daq_->TtlOut_SetPin(enable_pin_);
}

void RotatorImpl::Step()
{
    daq_->TtlOut_SetPin(step_pin_);
    // delay MIN 1.9 microsec

    daq_->TtlOut_ClrPin(step_pin_);
    // delay MIN 1.9 microsec
}

void RotatorImpl::applyDirection()
{
    switch (direction_) {
    case RotatorDirection::CW:
        daq_->TtlOut_ClrPin(direction_pin_);
        break;
    case RotatorDirection::CCW:
        daq_->TtlOut_SetPin(direction_pin_);
        break;
    default:
        assert(false);
        break;
    }
    // delay MIN 650 nanosec
}

void RotatorImpl::applySpeed()
{
    switch (speed_) {
    case RotatorSpeed::Low:
        daq_->TtlOut_ClrPin(m0_pin_);
        break;
    case RotatorSpeed::High:
        daq_->TtlOut_SetPin(m0_pin_);
        break;
    default:
        assert(false);
        break;
    }
    // delay MIN 650 nanosec
}

}}
