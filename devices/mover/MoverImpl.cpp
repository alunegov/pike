#include <MoverImpl.h>

#include <cassert>

namespace ros { namespace devices {

MoverImpl::~MoverImpl()
{
    Stop();
}

void MoverImpl::SetDirection(MoverDirection direction)
{
    direction_ = direction;
}

void MoverImpl::Start()
{
    Stop();
    // TODO: delay?

    applyDirection();

    daq_->TtlOut_SetPin(pwm_pin_);
}

void MoverImpl::Stop()
{
    daq_->TtlOut_ClrPin(pwm_pin_);
}

void MoverImpl::applyDirection()
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
