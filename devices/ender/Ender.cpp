#include <Ender.h>

#include <cstdint>

namespace ros { namespace devices {

bool Ender::Read()
{
    const uint16_t ttl_in = daq_->TtlIn();
    return (ttl_in & (1u << pin_)) != 0;
}

}}
