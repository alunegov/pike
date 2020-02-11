#pragma once

#include <atomic>
#include <cstdint>
#include <cmath>
#include <thread>

#include <Rotator.h>

#include <DAQ.h>

namespace ros { namespace devices {

// Вращение измерительного блока с помощью шагового двигателя (TI DRV8825)
class RotatorImpl : public Rotator
{
public:
    RotatorImpl() = delete;

    RotatorImpl(ros::dc::DAQ* daq, uint16_t enable_pin, uint16_t step_pin, uint16_t direction_pin, uint16_t m0_pin,
            uint32_t steps_per_msr, uint32_t steps_per_view) :
        daq_{daq},
        enable_pin_{enable_pin},
        step_pin_{step_pin},
        direction_pin_{direction_pin},
        m0_pin_{m0_pin},
        steps_per_msr_{steps_per_msr},
        steps_per_view_{steps_per_view}
    {}

    ~RotatorImpl() override;

    // Rotator

    void SetOutput(RotatorOutput* output) override;

    void SetDirection(RotatorDirection direction) override;

    void SetSpeed(RotatorSpeed speed) override;

    uint32_t StepsIn360() const override;

    tl::expected<void, std::error_code> Start() override;

    void Stop() override;

    tl::expected<void, std::error_code> Rotate(size_t steps_count = 1) override;

    tl::expected<void, std::error_code> Enable() override;

    tl::expected<void, std::error_code> Disable() override;

    tl::expected<void, std::error_code> PreStep() override;

    tl::expected<void, std::error_code> Step() override;

private:
    void NonVirtualStop();

    tl::expected<void, std::error_code> ApplyDirection();

    tl::expected<void, std::error_code> ApplySpeed();

    ros::dc::DAQ* daq_{nullptr};
    uint16_t enable_pin_{0};
    uint16_t step_pin_{0};
    uint16_t direction_pin_{0};
    uint16_t m0_pin_{0};
    uint32_t steps_per_msr_{0};
    uint32_t steps_per_view_{0};

    RotatorOutput* _output{nullptr};

    RotatorDirection direction_{RotatorDirection::CW};
    RotatorSpeed speed_{RotatorSpeed::Low};

    std::thread rotate_thread_;
    std::atomic_bool rotate_cancel_token_{false};
};

}}
