#include <Ender.h>

#include <cstdint>

namespace ros { namespace devices {

bool Ender::Read()
{
    const auto ttl_in = daq_->TtlIn();
    return (ttl_in & (1 << pin_)) != 0;
}

}}
