#pragma once

#include <cstdint>

#include <ceSerial.h>

namespace ros { namespace devices {

// ������ ����������/������� (FASTUS CD22)
struct CD22 {
public:
    CD22() = delete;

    explicit CD22(ce::ceSerial transport)
        : transport_{std::move(transport)}
    {}

    int16_t Read();

private:
    ce::ceSerial transport_;
};

}}
