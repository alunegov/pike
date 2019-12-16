#pragma once

#include <cstdint>

#include <ceSerial.h>

#include <Depthometer.h>

namespace ros { namespace devices {

// Датчик расстояния/глубины (FASTUS CD22)
struct CD22 : public Depthometer {
public:
    CD22() = delete;

    explicit CD22(const ce::ceSerial& transport)
        : transport_{transport}
    {}

    ~CD22() override;

    int16_t Read() override;

private:
    ce::ceSerial transport_;
};

}}
