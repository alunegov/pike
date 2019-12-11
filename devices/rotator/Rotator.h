#pragma once

#include <atomic>
#include <cstdint>

#include <LCardDevice.h>

namespace ros { namespace devices {

// Ќаправление вращени€
enum class RotatorDirection {
    CW,   // по часовой стрелке (по направлению движени€)
    CCW,  // против часовой стрелки (по направлению движени€)
};

// —корость вращени€
enum class RotatorSpeed {
    Low,   // 1/32 step
    High,  // 1/2 step
};

// ¬ращение измерительного блока с помощью шагового двигател€ (TI DRV8825)
class Rotator {
public:
    Rotator() = delete;

    Rotator(ros::dc::lcard::LCardDevice* daq, uint16_t enable_pin, uint16_t step_pin, uint16_t direction_pin,
        uint16_t m0_pin) :
        daq_{daq},
        enable_pin_{enable_pin},
        step_pin_{step_pin},
        direction_pin_{direction_pin},
        m0_pin_{m0_pin}
    {}

    void SetDirection(RotatorDirection direction);

    void SetSpeed(RotatorSpeed speed);

    void Start();

    void Stop();

    void Rotate(size_t steps_count = 1);

    void Enable();

    void Disable();

    void Step();

private:
    void applyDirection();

    void applySpeed();

    ros::dc::lcard::LCardDevice* daq_{nullptr};
    uint16_t enable_pin_{0};
    uint16_t step_pin_{0};
    uint16_t direction_pin_{0};
    uint16_t m0_pin_{0};

    RotatorDirection direction_{RotatorDirection::CW};
    RotatorSpeed speed_{RotatorSpeed::Low};

    std::atomic_bool rotating{false};
};

}}
