#pragma once

#include <cstdint>

#include <LCardDevice.h>

namespace ros { namespace devices {

// "Концевой" датчик
class Ender {
public:
    Ender() = delete;

    Ender(ros::dc::lcard::LCardDevice* daq, uint16_t pin) :
        daq_{daq},
        pin_{pin}
    {}

    bool Read();

private:
    ros::dc::lcard::LCardDevice* daq_{nullptr};
    uint16_t pin_{0};
};

}}
