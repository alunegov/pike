#pragma once

#include <cstdint>

namespace ros { namespace devices {

// ������ ����������/�������
class Depthometer
{
public:
    virtual ~Depthometer() = default;

    virtual int16_t Read() = 0;
};

}}
