#pragma once

namespace ros { namespace devices {

// Направление движения
enum class MoverDirection {
    Forward,   // вперёд
    Backward,  // назад
};

// Перемещение вперёд-назад
class Mover
{
public:
    virtual ~Mover() = default;

    // Задаёт направление перемещения
    // Фактически выставляется только в Start.
    virtual void SetDirection(MoverDirection direction) = 0;

    // Запускает перемещение
    // Перед началом останавливает возможное перемещение и выставляет направление.
    virtual void Start() = 0;

    // Останавливает перемещение
    virtual void Stop() = 0;
};

}}
