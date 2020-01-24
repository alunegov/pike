#include <PikeImpl.h>

#include <cstdint>

namespace ros { namespace pike { namespace logic {

tl::expected<void, std::error_code> PikeImpl::ReadAndUpdateTtlIn()
{
    return daq()->TtlIn()
        .map([this](uint16_t ttl_in) {
            ender1()->Update(ttl_in);
            ender2()->Update(ttl_in);
        });
}

}}}
