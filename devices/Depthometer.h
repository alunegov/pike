#pragma once

#include <cstdint>

namespace ros { namespace devices {

// Датчик расстояния/глубины
class Depthometer
{
public:
    virtual ~Depthometer() = default;

    virtual int16_t Read() = 0;
};

}}
