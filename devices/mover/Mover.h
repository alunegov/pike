#pragma once

#include <cstdint>

#include <LCardDevice.h>

namespace ros { namespace devices {

// Направление движения
enum class MoverDirection {
    Forward,   // вперёд
    Backward,  // назад
};

// Перемещение вперёд-назад с помощью коллекторного двигателя
// Частота импульсов PWM 10кГц. Мы используем режим максимальной скорости (единичный коэфф. заполнения). Перед сменой
// направления нужно обязательно останавливать двигатель.
class Mover {
public:
    Mover() = delete;

    Mover(ros::dc::lcard::LCardDevice* daq, uint16_t pwm_pin, uint16_t dir_pin) :
        daq_{daq},
        pwm_pin_{pwm_pin},
        dir_pin_{dir_pin}
    {}

    void SetDirection(MoverDirection direction);

    void Start();

    void Stop();

private:
    void applyDirection();

    ros::dc::lcard::LCardDevice* daq_{nullptr};
    uint16_t pwm_pin_{0};
    uint16_t dir_pin_{0};

    MoverDirection direction_{MoverDirection::Forward};
};

}}
