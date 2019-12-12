#include <Mover.h>

#include <cassert>

namespace ros { namespace devices {

Mover::~Mover()
{
    Stop();
}

void Mover::SetDirection(MoverDirection direction)
{
    direction_ = direction;
}

void Mover::Start()
{
    Stop();
    // TODO: delay?

    applyDirection();

    daq_->TtlOut_SetPin(pwm_pin_);
}

void Mover::Stop()
{
    daq_->TtlOut_ClrPin(pwm_pin_);
}

void Mover::applyDirection()
{
    switch (direction_) {
    case MoverDirection::Forward:
        daq_->TtlOut_ClrPin(dir_pin_);
        break;
    case MoverDirection::Backward:
        daq_->TtlOut_SetPin(dir_pin_);
        break;
    default:
        assert(false);
        break;
    }
}

}}
