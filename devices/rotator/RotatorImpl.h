#pragma once

#include <atomic>
#include <cstdint>
#include <thread>

#include <Rotator.h>

#include <DAQ.h>

namespace ros { namespace devices {

// Вращение измерительного блока с помощью шагового двигателя (TI DRV8825)
class RotatorImpl : public Rotator {
public:
    RotatorImpl() = delete;

    RotatorImpl(ros::dc::DAQ* daq, uint16_t enable_pin, uint16_t step_pin, uint16_t direction_pin, uint16_t m0_pin) :
        daq_{daq},
        enable_pin_{enable_pin},
        step_pin_{step_pin},
        direction_pin_{direction_pin},
        m0_pin_{m0_pin}
    {}

    ~RotatorImpl() override;

    void SetDirection(RotatorDirection direction) override;

    void SetSpeed(RotatorSpeed speed) override;

    void Start() override;

    void Stop() override;

    void Rotate(size_t steps_count = 1) override;

    void Enable() override;

    void Disable() override;

    void Step() override;

private:
    void applyDirection();

    void applySpeed();

    ros::dc::DAQ* daq_{nullptr};
    uint16_t enable_pin_{0};
    uint16_t step_pin_{0};
    uint16_t direction_pin_{0};
    uint16_t m0_pin_{0};

    RotatorDirection direction_{RotatorDirection::CW};
    RotatorSpeed speed_{RotatorSpeed::Low};

    std::thread rotate_thread_;
    std::atomic_bool rotate_cancel_token_{false};
};

}}
