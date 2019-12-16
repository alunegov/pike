#pragma once

namespace ros { namespace devices {

// Направление движения
enum class MoverDirection {
    Forward,   // вперёд
    Backward,  // назад
};

// Перемещение вперёд-назад
class Mover {
public:
    virtual ~Mover() = default;

    virtual void SetDirection(MoverDirection direction) = 0;

    virtual void Start() = 0;

    virtual void Stop() = 0;
};

}}
