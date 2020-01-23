#pragma once

#include <cstdint>
#include <system_error>

#include <tl/expected.hpp>

namespace ros { namespace devices {

// Датчик расстояния/глубины
class Depthometer
{
public:
    virtual ~Depthometer() = default;

    virtual tl::expected<int16_t, std::error_code> Read() = 0;
};

}}
