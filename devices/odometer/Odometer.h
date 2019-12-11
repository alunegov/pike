#pragma once

#include <cstdint>
#include <vector>

namespace ros { namespace devices {

// Одометр/курвиметр/датчик пройденного расстояния (ЛИР-119)
class Odometer {
public:
    Odometer() = delete;

    Odometer(uint16_t a_channel, uint16_t b_channel) :
        a_channel_{a_channel},
        b_channel_{b_channel}
    {}

    void Update(const std::vector<int16_t>& values);

    int32_t Get();

private:
    uint16_t a_channel_{0};
    uint16_t b_channel_{0};
};

}}
