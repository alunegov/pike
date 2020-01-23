#pragma once

#include <cmath>
#include <system_error>

#include <tl/expected.hpp>

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

class RotatorOutput
{
public:
    virtual ~RotatorOutput() = default;

    virtual void RotateError(const std::error_code& ec) = 0;
};

// Вращение измерительного блока
class Rotator
{
public:
    virtual ~Rotator() = default;

    virtual void SetOutput(RotatorOutput* output) = 0;

    // Задаёт направление вращения
    // Фактически выставляется только в Start или Rotate.
    virtual void SetDirection(RotatorDirection direction) = 0;

    // Задаёт скорость вращения
    // Фактически выставляется только в Start или Rotate.
    virtual void SetSpeed(RotatorSpeed speed) = 0;

    // Возвращает количество шагов для поворота на 360°
    // Возвращает количество для заданной, а не выставленной скорости - это значение может быть неверным, если позже
    // для вращения используется Step.
    virtual uint32_t StepsIn360() = 0;

    // Запускает вращение
    // Перед началом выставляет направление и скорость.
    // Как Rotate, только асинхронно и пока не остановят.
    virtual tl::expected<void, std::error_code> Start() = 0;

    // Останавливает вращение
    virtual void Stop() = 0;

    // Поворачивает на указанное число шагов
    // Перед началом выставляет направление и скорость.
    // Как Start, только синхронно и на указанное кол-во шагов.
    virtual tl::expected<void, std::error_code> Rotate(size_t steps_count = 1) = 0;

    // Разрешает вращение (low-level)
    virtual tl::expected<void, std::error_code> Enable() = 0;

    // Запрещает вращение (low-level)
    virtual tl::expected<void, std::error_code> Disable() = 0;

    // Поворачивает на один шаг (low-level)
    virtual tl::expected<void, std::error_code> Step() = 0;
};

}}
