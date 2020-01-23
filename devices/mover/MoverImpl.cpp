#include <MoverImpl.h>

#include <cassert>

namespace ros { namespace devices {

MoverImpl::~MoverImpl()
{
    if (daq_ != nullptr) {
        NonVirtualStop();
    }
}

void MoverImpl::SetDirection(MoverDirection direction)
{
    direction_ = direction;
}

tl::expected<void, std::error_code> MoverImpl::Start()
{
    return Stop()  // TODO: delay?
        .and_then([this]() { return applyDirection(); })
        .and_then([this]() { return daq_->TtlOut_SetPin(pwm_pin_); });
}

tl::expected<void, std::error_code> MoverImpl::Stop()
{
    return NonVirtualStop();
}

tl::expected<void, std::error_code> MoverImpl::NonVirtualStop()
{
    return daq_->TtlOut_ClrPin(pwm_pin_);
}

tl::expected<void, std::error_code> MoverImpl::applyDirection()
{
    tl::expected<void, std::error_code> res;

    switch (direction_) {
    case MoverDirection::Forward:
        res = daq_->TtlOut_ClrPin(dir_pin_);
        break;
    case MoverDirection::Backward:
        res = daq_->TtlOut_SetPin(dir_pin_);
        break;
    default:
        assert(false);
        res = tl::make_unexpected(std::make_error_code(std::errc::bad_address));
        break;
    }

    return res;
}

}}
