#include <EnderImpl.h>

#include <cstdint>

namespace ros { namespace devices {

void EnderImpl::Update(uint16_t ttl_in)
{
    _state = (ttl_in & (1u << _pin)) != 0;
}

bool EnderImpl::Get()
{
    return _state;
}

bool EnderImpl::Read()
{
    const uint16_t ttl_in = _daq->TtlIn();
    Update(ttl_in);
    return Get();
}

}}
