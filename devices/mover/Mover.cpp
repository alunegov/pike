#include <Mover.h>

#include <cassert>

namespace ros { namespace devices {

void Mover::SetDirection(MoverDirection direction)
{
    direction_ = direction;
}

void Mover::Start()
{
    Stop();
    // TODO: delay?

    applyDirection();

    daq_->TtlOut_SetPin(1 << pwm_pin_);
}

void Mover::Stop()
{
    daq_->TtlOut_ClrPin(1 << pwm_pin_);
}

void Mover::applyDirection()
{
    switch (direction_) {
    case MoverDirection::Forward:
        daq_->TtlOut_ClrPin(1 << dir_pin_);
        break;
    case MoverDirection::Backward:
        daq_->TtlOut_SetPin(1 << dir_pin_);
        break;
    default:
        assert(false);
        break;
    }
}

}}
