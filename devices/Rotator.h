#pragma once

namespace ros { namespace devices {

// Направление вращения
enum class RotatorDirection
{
    CW,   // по часовой стрелке (по направлению движения)
    CCW,  // против часовой стрелки (по направлению движения)
};

// Скорость вращения
enum class RotatorSpeed
{
    Low,   // 1/32 step
    High,  // 1/2 step
};

// Вращение измерительного блока
class Rotator
{
public:
    virtual ~Rotator() = default;

    virtual void SetDirection(RotatorDirection direction) = 0;

    virtual void SetSpeed(RotatorSpeed speed) = 0;

    virtual void Start() = 0;

    virtual void Stop() = 0;

    virtual void Rotate(size_t steps_count = 1) = 0;

    virtual void Enable() = 0;

    virtual void Disable() = 0;

    virtual void Step() = 0;
};

}}
