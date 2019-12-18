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
class Pike
{
public:
    virtual ~Pike() = default;

    // Возвращает плату АЦП/ЦАП/ТТЛ
    virtual ros::dc::DAQ* daq() const = 0;

    // Возвращает "концевой" датчик 1
    virtual ros::devices::Ender* ender1() const = 0;

    // Возвращает "концевой" датчик 2
    virtual ros::devices::Ender* ender2() const = 0;

    // Возвращает вращатель измерительного блока с глубинометром и камерами
    virtual ros::devices::Rotator* rotator() const = 0;

    // Возвращает перемещатель
    virtual ros::devices::Mover* mover() const = 0;

    // Возвращает одометр
    virtual ros::devices::Odometer* odometer() const = 0;

    // Возвращает инклинометр
    virtual ros::devices::Inclinometer* inclinometer() const = 0;

    // Возвращает глубинометр
    virtual ros::devices::Depthometer* depthometer() const = 0;
};

}}
