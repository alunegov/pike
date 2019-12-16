#pragma once

#include <cstdint>

#include <DAQ.h>
#include <Depthometer.h>
#include <Ender.h>
#include <Inclinometer.h>
#include <Mover.h>
#include <Odometer.h>
#include <Rotator.h>

namespace ros { namespace devices {

// "Пика"
class Pike {
public:
    virtual ~Pike() = default;

    virtual ros::dc::DAQ* daq() const = 0;

    // "Концевой" датчик 1
    virtual ros::devices::Ender* ender1() const = 0;

    // "Концевой" датчик 2
    virtual ros::devices::Ender* ender2() const = 0;

    // Вращатель измерительного блока с глубинометром и камерами
    virtual ros::devices::Rotator* rotator() const = 0;

    // Перемещатель
    virtual ros::devices::Mover* mover() const = 0;

    // Одометр
    virtual ros::devices::Odometer* odometer() const = 0;

    // Инклинометр
    virtual ros::devices::Inclinometer* inclinometer() const = 0;

    // Глубинометр
    virtual ros::devices::Depthometer* depthometer() const = 0;
};

}}
