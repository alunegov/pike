#include <Rotator.h>

#include <cassert>
#include <thread>

namespace ros { namespace devices {

void Rotator::SetDirection(RotatorDirection direction)
{
    direction_ = direction;
}

void Rotator::SetSpeed(RotatorSpeed speed)
{
    speed_ = speed;
}

void Rotator::Start()
{
    assert(!rotating);

    // запуск потока, генерирующего step
    rotating = true;

    std::thread{[&]() {
        Enable();

        applyDirection();

        applySpeed();

        while (rotating) {
            Step();
        }

        // оставляем enable
    }}.detach();
}

void Rotator::Stop()
{
    // останов (и завершение) потока, генерирующего step
    rotating = false;
}

void Rotator::Rotate(size_t steps_count)
{
    assert(!rotating);

    Enable();

    applyDirection();

    applySpeed();

    while (steps_count > 0) {
        Step();

        steps_count--;
    }

    // оставляем enable
}

void Rotator::Enable()
{
    daq_->TtlOut_ClrPin(1 << enable_pin_);
    // delay MIN 650 nanosec to STEP
}

void Rotator::Disable()
{
    daq_->TtlOut_SetPin(1 << enable_pin_);
}

void Rotator::Step()
{
    daq_->TtlOut_SetPin(1 << step_pin_);
    // delay MIN 1.9 microsec

    daq_->TtlOut_ClrPin(1 << step_pin_);
    // delay MIN 1.9 microsec
}

void Rotator::applyDirection()
{
    switch (direction_) {
    case RotatorDirection::CW:
        daq_->TtlOut_ClrPin(1 << direction_pin_);
        break;
    case RotatorDirection::CCW:
        daq_->TtlOut_SetPin(1 << direction_pin_);
        break;
    default:
        assert(false);
        break;
    }
    // delay MIN 650 nanosec
}

void Rotator::applySpeed()
{
    switch (speed_) {
    case RotatorSpeed::Low:
        daq_->TtlOut_ClrPin(1 << m0_pin_);
        break;
    case RotatorSpeed::High:
        daq_->TtlOut_SetPin(1 << m0_pin_);
        break;
    default:
        assert(false);
        break;
    }
    // delay MIN 650 nanosec
}

}}
