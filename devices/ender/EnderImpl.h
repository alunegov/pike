#pragma once

#include <cstdint>

#include <Ender.h>

#include <DAQ.h>

namespace ros { namespace devices {

// "��������" ������
class EnderImpl : public Ender
{
public:
    EnderImpl() = delete;

    EnderImpl(ros::dc::DAQ* daq, uint16_t pin) :
        _daq{daq},
        _pin{pin}
    {}

    // Ender

    void Update(uint16_t ttl_in) override;

    bool Get() override;

    bool Read() override;

private:
    ros::dc::DAQ* _daq{nullptr};
    uint16_t _pin{0};

    bool _state{false};
};

}}
