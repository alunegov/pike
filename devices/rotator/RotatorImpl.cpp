#include <RotatorImpl.h>

#include <cassert>
#include <thread>

#include <error_devices.hpp>

namespace ros { namespace devices {

RotatorImpl::~RotatorImpl()
{
    if (rotate_thread_.joinable()) {
        rotate_cancel_token_ = true;
        rotate_thread_.join();
    }
}

void RotatorImpl::SetOutput(RotatorOutput* output)
{
    assert(!rotate_thread_.joinable());
    assert(output != nullptr);
    _output = output;
}

void RotatorImpl::SetDirection(RotatorDirection direction)
{
    direction_ = direction;
}

void RotatorImpl::SetSpeed(RotatorSpeed speed)
{
    speed_ = speed;
}

uint32_t RotatorImpl::StepsIn360()
{
    // определяем по заданной, а не выставленной скорости
    switch (speed_) {
    case RotatorSpeed::Low:
        return steps_per_msr_;
    case RotatorSpeed::High:
        return steps_per_view_;
    default:
        assert(false);
        return 1;
    }
}

tl::expected<void, std::error_code> RotatorImpl::Start()
{
    assert(_output != nullptr);
    assert(!rotate_thread_.joinable());

    return Enable()
        .and_then([this]() { return PreStep(); })
        .map([this]() {
            // запуск потока, генерирующего step
            rotate_cancel_token_ = false;

            // TODO: std::async on threadpool?
            rotate_thread_ = std::thread{[this]() {
                while (!rotate_cancel_token_) {
                    const auto step_opt = Step();
                    if (!step_opt) {
                        // TODO: log and return/output?
                        _output->RotateError(step_opt.error());
                        break;  // while, thread exit
                    }
                }

                // оставляем enable
            }};
        });
}

void RotatorImpl::Stop()
{
    assert(rotate_thread_.joinable());

    // останов (и завершение) потока, генерирующего step
    rotate_cancel_token_ = true;
    rotate_thread_.join();
}

tl::expected<void, std::error_code> RotatorImpl::Rotate(size_t steps_count)
{
    assert(!rotate_thread_.joinable());

    return Enable()
        .and_then([this]() { return PreStep(); })
        .and_then([this, steps_count]() mutable -> tl::expected<void, std::error_code> {
            while (steps_count > 0) {
                const auto step_opt = Step();
                if (!step_opt) {
                    return tl::make_unexpected(step_opt.error());
                }

                steps_count--;
            }

            return {};
        });

    // оставляем enable
}

tl::expected<void, std::error_code> RotatorImpl::Enable()
{
    //assert(!rotate_thread_.joinable());

    return daq_->TtlOut_ClrPin(enable_pin_);  // delay MIN 650 nanosec to STEP
}

tl::expected<void, std::error_code> RotatorImpl::Disable()
{
    //assert(!rotate_thread_.joinable());

    return daq_->TtlOut_SetPin(enable_pin_);
}

tl::expected<void, std::error_code> RotatorImpl::PreStep()
{
    //assert(!rotate_thread_.joinable());

    return applyDirection()
        .and_then([this]() { return applySpeed(); });
}

tl::expected<void, std::error_code> RotatorImpl::Step()
{
    return daq_->TtlOut_SetPin(step_pin_)  // delay MIN 1.9 microsec
        //.and_then([this]() -> tl::expected<void, std::error_code> { return tl::make_unexpected(ros::make_error_code(ros::error_devices_rotator::generic)); })
        .and_then([this]() { return daq_->TtlOut_ClrPin(step_pin_); });  // delay MIN 1.9 microsec
}

tl::expected<void, std::error_code> RotatorImpl::applyDirection()
{
    tl::expected<void, std::error_code> res;

    switch (direction_) {
    case RotatorDirection::CW:
        res = daq_->TtlOut_ClrPin(direction_pin_);
        break;
    case RotatorDirection::CCW:
        res = daq_->TtlOut_SetPin(direction_pin_);
        break;
    default:
        assert(false);
        res = tl::make_unexpected(ros::make_error_code(ros::error_devices_rotator::invalid_direction));
        break;
    }
    // delay MIN 650 nanosec

    return res;
}

tl::expected<void, std::error_code> RotatorImpl::applySpeed()
{
    tl::expected<void, std::error_code> res;

    switch (speed_) {
    case RotatorSpeed::Low:
        res = daq_->TtlOut_ClrPin(m0_pin_);
        break;
    case RotatorSpeed::High:
        res = daq_->TtlOut_SetPin(m0_pin_);
        break;
    default:
        assert(false);
        res = tl::make_unexpected(ros::make_error_code(ros::error_devices_rotator::invalid_speed));
        break;
    }
    // delay MIN 650 nanosec

    return res;
}

}}
