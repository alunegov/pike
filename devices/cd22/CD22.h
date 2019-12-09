#pragma once

#include <cstdint>

#include <ceSerial.h>

namespace ros { namespace devices {

struct CD22 {
public:
    explicit CD22(ce::ceSerial transport)
        : transport_(transport)
    {}

    int16_t ReadDist();
private:
    ce::ceSerial transport_;
};

}}

