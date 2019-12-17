#pragma once

#include <cstdint>

#include <Ender.h>

#include <DAQ.h>

namespace ros { namespace devices {

// "Концевой" датчик
class EnderImpl : public Ender {
public:
    EnderImpl() = delete;

    EnderImpl(ros::dc::DAQ* daq, uint16_t pin) :
        daq_{daq},
        pin_{pin}
    {}

    // Ender

    bool Read() override;

private:
    ros::dc::DAQ* daq_{nullptr};
    uint16_t pin_{0};
};

}}
