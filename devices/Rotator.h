#pragma once

#include <cmath>

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
    virtual void Start() = 0;

    // Останавливает вращение
    virtual void Stop() = 0;

    // Поворачивает на указанное число шагов
    // Перед началом выставляет направление и скорость.
    // Как Start, только синхронно и на указанное кол-во шагов.
    virtual void Rotate(size_t steps_count = 1) = 0;

    // Разрешает вращение (low-level)
    virtual void Enable() = 0;

    // Запрещает вращение (low-level)
    virtual void Disable() = 0;

    // Поворачивает на один шаг (low-level)
    virtual void Step() = 0;
};

}}
