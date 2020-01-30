#include <EnderImpl.h>

#include <cstdint>

namespace ros { namespace devices {

void EnderImpl::Update(uint16_t ttl_in)
{
    _state = (ttl_in & (1u << _pin)) != 0;
}

bool EnderImpl::Get() const
{
    return _state;
}

tl::expected<bool, std::error_code> EnderImpl::Read()
{
    return _daq->TtlIn()
        .map([this](uint16_t ttl_in) {
            Update(ttl_in);
            return Get();
        });
}

}}
