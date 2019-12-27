#include <PikeImpl.h>

#include <cstdint>

namespace ros { namespace devices {

void PikeImpl::ReadAndUpdateTtlIn()
{
    const uint16_t ttl_in = daq()->TtlIn();
    ender1()->Update(ttl_in);
    ender2()->Update(ttl_in);
}

}}
